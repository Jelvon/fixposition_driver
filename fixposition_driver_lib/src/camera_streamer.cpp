/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition camera streamer (PBx-A1 / Eagle camera streaming over HTTP)
 */

/* LIBC/STL */
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <thread>

/* EXTERNAL */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/time.hpp>

/* PACKAGE */
#include "fixposition_driver_lib/camera_streamer.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

using namespace fpsdk::common;
using namespace fpsdk::common::cam;

CameraStreamer::CameraStreamer(const CameraParams& params) /* clang-format off */ :
    params_  { params },
    worker_  { "cam_" + params.name_, std::bind(&CameraStreamer::Worker, this) }  // clang-format on
{}

CameraStreamer::~CameraStreamer() { StopStreamer(); }

void CameraStreamer::AddFrameObserver(FrameObserver observer) { frame_observers_.push_back(observer); }

#if FPSDK_USE_FFMPEG
void CameraStreamer::AddImageObserver(ImageObserver observer) { image_observers_.push_back(observer); }
#endif

// ---------------------------------------------------------------------------------------------------------------------

bool CameraStreamer::StartStreamer() {
    // Only video streams. The _IMG streams are raw NV12 (24..28 MiB/s per camera), not worth it here.
    if ((params_.data_type_ != CamDataType::HIRES_VID) && (params_.data_type_ != CamDataType::LORES_VID)) {
        ERROR("CameraStreamer(%s) unsupported stream %s (only HIRES_VID and LORES_VID)", params_.name_.c_str(),
              CamDataTypeToStr(params_.data_type_));
        return false;
    }
    if ((params_.cam_id_ == CamId::UNSPECIFIED) || (params_.timeout_ < 0.1) || params_.host_.empty()) {
        ERROR("CameraStreamer(%s) bad params", params_.name_.c_str());
        return false;
    }
#if !FPSDK_USE_FFMPEG
    if (params_.decode_) {
        WARNING(
            "CameraStreamer(%s) decode requested, but the Fixposition SDK was built without FFmpeg. Only raw "
            "video frames will be available.",
            params_.name_.c_str());
    }
#endif
    return worker_.Start();
}

void CameraStreamer::StopStreamer() {
    if (worker_.GetStatus() == worker_.Status::RUNNING) {
        worker_.Stop();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool CameraStreamer::Worker() {
    INFO("CameraStreamer(%s) running (%s %s %s)", params_.name_.c_str(), params_.host_.c_str(),
         CamIdToStr(params_.cam_id_), CamDataTypeToStr(params_.data_type_));

    const uint32_t delay_ms = (uint32_t)(params_.reconnect_delay_ * 1e3);

    while (!worker_.ShouldAbort()) {
        // A fresh CamStream for each connection attempt (its HTTP parser state cannot be reused)
        auto stream = CreateCamStream({params_.name_, params_.host_, params_.cam_id_, params_.data_type_,
                                       1 /* rate: must be 1 for _VID */, time::Duration::FromSec(params_.timeout_)});
        if (!stream) {
            ERROR("CameraStreamer(%s) cannot create stream", params_.name_.c_str());
            return false;
        }

        if (stream->Connect()) {
            stats_.n_connect_++;
            INFO("CameraStreamer(%s) connected", params_.name_.c_str());
            if (!Stream(*stream)) {
                stream->Disconnect();
            }
        }

        if (worker_.ShouldAbort()) {
            break;
        }
        WARNING("CameraStreamer(%s) not connected, retrying in %.1f seconds...", params_.name_.c_str(),
                params_.reconnect_delay_);
        if (worker_.Sleep(delay_ms) == thread::WaitRes::WOKEN) {
            break;
        }
    }

    INFO("CameraStreamer(%s) stopped", params_.name_.c_str());
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

bool CameraStreamer::Stream(CamStream& stream) {
    // The read timeout of CamStream does not work (the SDK reads synchronously), so a stalled connection (e.g. cable
    // unplugged, sensor frozen) would block NextFrame() forever, and so StopStreamer(). A watchdog thread closes the
    // connection (which unblocks NextFrame()) if there is no data for a while, or if the streamer is to be stopped.
    using Clock = std::chrono::steady_clock;
    std::atomic<int64_t> last_data{Clock::now().time_since_epoch().count()};
    std::atomic<bool> finished{false};
    std::atomic<bool> stalled{false};
    bool disconnected = false;
    const auto stall_timeout =
        std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(params_.timeout_));
    std::thread watchdog([&]() {
        while (!finished) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            const auto idle = Clock::now() - Clock::time_point(Clock::duration(last_data.load()));
            if (!finished && (worker_.ShouldAbort() || (idle > stall_timeout))) {
                stalled = !worker_.ShouldAbort();
                stream.Disconnect();
                disconnected = true;
                return;
            }
        }
    });

    bool seen_iframe = !params_.wait_iframe_;
    bool have_seq = false;
    uint64_t last_seq = 0;
    uint64_t last_ts = 0;   // [ns] time of the previous frame
    double interval = 0.0;  // [ns] estimated frame interval

#if FPSDK_USE_FFMPEG
    video::VideoFrameDecoderPtr decoder;
#endif

    while (!worker_.ShouldAbort()) {
        // Fresh object for each frame: CamStream::NextFrame() only overwrites the fields it finds in the meta data,
        // anything else (e.g. valid_, frm_) would be left over from the previous frame.
        CamData data;
        last_data = Clock::now().time_since_epoch().count();  // the watchdog measures the time in NextFrame()
        if (!stream.NextFrame(data)) {
            break;  // lost connection, bad data, stalled (closed by the watchdog)
        }

        // Not a (known) video frame
        const bool is_h265 = (data.fmt_ == CamDataFmt::H265_NAL);
        const bool is_h264 = (data.fmt_ == CamDataFmt::H264_NAL);
        if (!data.valid_ || !(is_h265 || is_h264)) {
            stats_.n_dropped_++;
            WARNING_THR(5000, "CameraStreamer(%s) dropping unsupported frame (valid=%d fmt=%s)", params_.name_.c_str(),
                        data.valid_, CamDataFmtToStr(data.fmt_));
            continue;
        }

        // Lost frames? The frame times (mid-exposure) are the reliable indicator: the frame interval is constant (e.g.
        // 40ms at 25Hz). The sequence number is only a counter and is not always continuous even if no frames are
        // missing (seen: a skipped and then a repeated number), so it is only counted.
        if (have_seq && (data.seq_ != (last_seq + 1))) {
            stats_.n_seq_anomalies_++;
        }
        have_seq = true;
        last_seq = data.seq_;
        if (data.ts_ != 0) {
            if (last_ts != 0) {
                const double step = (data.ts_ > last_ts) ? (double)(data.ts_ - last_ts) : 0.0;
                if (interval > 0.0 && step > (1.5 * interval)) {
                    const uint64_t lost = (uint64_t)((step / interval) + 0.5) - 1;
                    stats_.n_lost_ += lost;
                    WARNING("CameraStreamer(%s) lost %" PRIu64 " frame(s) (frame time step %.1f ms, expected %.1f ms)",
                            params_.name_.c_str(), lost, step * 1e-6, interval * 1e-6);
                } else if (step > 0.0) {
                    interval = (interval > 0.0) ? ((0.9 * interval) + (0.1 * step)) : step;
                }
            }
            last_ts = data.ts_;
        }

        // A decoder (or a client) can only start at an I-frame
        if (!seen_iframe) {
            if (data.frm_ != CamDataFrm::I_FRAME) {
                stats_.n_dropped_++;
                continue;
            }
            seen_iframe = true;
        }

        stats_.n_frames_++;
        for (auto& observer : frame_observers_) {
            observer(data);
        }

#if FPSDK_USE_FFMPEG
        if (params_.decode_) {
            if (!decoder) {
                decoder = video::CreateVideoFrameDecoder(
                    {params_.name_, is_h265 ? video::VideoCodec::H265 : video::VideoCodec::H264,
                     params_.decode_mono_ ? video::PixelFmt::Y8 : video::PixelFmt::RGB24, params_.decode_scale_,
                     video::HwAccelFromStrOr(params_.decode_accel_.c_str(), video::HwAccel::AUTO),
                     video::ScalingQual::BICUBIC});
                if (!decoder) {
                    ERROR("CameraStreamer(%s) cannot create decoder", params_.name_.c_str());
                    break;  // bad params, there's no point in retrying quickly
                }
            }
            const auto img = decoder->DecodeFrame(data.data_);
            if (img) {
                stats_.n_images_++;
                for (auto& observer : image_observers_) {
                    observer(data, *img);
                }
            } else {
                stats_.n_decode_fail_++;
                // A broken decoder cannot recover, discard it. We need a new I-frame to start a new one.
                if (!decoder->IsOkay()) {
                    WARNING("CameraStreamer(%s) decoder failed, restarting at next I-frame", params_.name_.c_str());
                    decoder.reset();
                    seen_iframe = false;
                }
            }
        }
#endif
    }

    finished = true;
    watchdog.join();
    if (stalled) {
        WARNING("CameraStreamer(%s) no data for %.1f seconds, closing connection", params_.name_.c_str(),
                params_.timeout_);
    }
    return disconnected;
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
