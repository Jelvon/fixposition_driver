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
#include <cinttypes>

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
{
}

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
        WARNING("CameraStreamer(%s) decode requested, but the Fixposition SDK was built without FFmpeg. Only raw "
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
        auto stream = CreateCamStream({ params_.name_, params_.host_, params_.cam_id_, params_.data_type_,
                                        1 /* rate: must be 1 for _VID */,
                                        time::Duration::FromSec(params_.timeout_) });
        if (!stream) {
            ERROR("CameraStreamer(%s) cannot create stream", params_.name_.c_str());
            return false;
        }

        if (stream->Connect()) {
            stats_.n_connect_++;
            INFO("CameraStreamer(%s) connected", params_.name_.c_str());
            Stream(*stream);
            stream->Disconnect();
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

void CameraStreamer::Stream(CamStream& stream) {
    bool seen_iframe = !params_.wait_iframe_;
    bool have_seq = false;
    uint64_t last_seq = 0;

#if FPSDK_USE_FFMPEG
    video::VideoFrameDecoderPtr decoder;
#endif

    while (!worker_.ShouldAbort()) {
        // Fresh object for each frame: CamStream::NextFrame() only overwrites the fields it finds in the meta data,
        // anything else (e.g. valid_, frm_) would be left over from the previous frame.
        CamData data;
        if (!stream.NextFrame(data)) {
            break;  // lost connection, bad data, timeout
        }

        // Not a (known) video frame
        const bool is_h265 = (data.fmt_ == CamDataFmt::H265_NAL);
        const bool is_h264 = (data.fmt_ == CamDataFmt::H264_NAL);
        if (!data.valid_ || !(is_h265 || is_h264)) {
            stats_.n_dropped_++;
            WARNING_THR(5000, "CameraStreamer(%s) dropping unsupported frame (valid=%d fmt=%s)",
                        params_.name_.c_str(), data.valid_, CamDataFmtToStr(data.fmt_));
            continue;
        }

        // Lost frames? (seq is the sensor's frame counter)
        if (have_seq && (data.seq_ != (last_seq + 1))) {
            stats_.n_seq_gaps_++;
            WARNING("CameraStreamer(%s) frame sequence gap: %" PRIu64 " -> %" PRIu64, params_.name_.c_str(), last_seq,
                    data.seq_);
        }
        have_seq = true;
        last_seq = data.seq_;

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
                    { params_.name_, is_h265 ? video::VideoCodec::H265 : video::VideoCodec::H264,
                      params_.decode_mono_ ? video::PixelFmt::Y8 : video::PixelFmt::RGB24, params_.decode_scale_,
                      video::HwAccelFromStrOr(params_.decode_accel_.c_str(), video::HwAccel::AUTO),
                      video::ScalingQual::BICUBIC });
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
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
