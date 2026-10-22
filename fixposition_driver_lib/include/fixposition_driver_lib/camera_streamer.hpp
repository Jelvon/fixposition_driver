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
 *
 * ROS-independent. One CameraStreamer instance handles one camera of one sensor: it connects to the sensor's
 * /api/v2/camera/data HTTP API (via fpsdk::common::cam::CamStream), reconnects when the connection is lost, drops
 * everything before the first I-frame, and hands out the received frames (and optionally the decoded images) to
 * observers. This is the camera counterpart of FixpositionDriver (which handles the FP_A/NMEA/NOV_B byte stream).
 */

#ifndef __FIXPOSITION_DRIVER_LIB_CAMERA_STREAMER_HPP__
#define __FIXPOSITION_DRIVER_LIB_CAMERA_STREAMER_HPP__

/* LIBC/STL */
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

/* EXTERNAL */
#include <boost/core/noncopyable.hpp>
#include <fpsdk_common/cam.hpp>
#include <fpsdk_common/thread.hpp>
#include <fpsdk_common/video.hpp>

/* PACKAGE */

namespace fixposition {
/* ****************************************************************************************************************** */

/**
 * @brief Parameters for one camera stream
 */
struct CameraParams {
    std::string name_ = "cam1";                                      //!< Name (for logging, and topic names)
    std::string host_ = "10.0.2.1";                                  //!< Sensor host name or IP address
    fpsdk::common::cam::CamId cam_id_ = fpsdk::common::cam::CamId::CAM1;  //!< Which camera
    //! Which stream. Only the video streams (HIRES_VID, LORES_VID) are supported by this class.
    fpsdk::common::cam::CamDataType data_type_ = fpsdk::common::cam::CamDataType::HIRES_VID;
    double reconnect_delay_ = 5.0;  //!< Wait time [s] before reconnecting after the connection was lost
    double timeout_ = 1.0;          //!< Timeout [s] for receiving data, must be >= 0.1. Also limits StopStreamer() latency
    bool wait_iframe_ = true;       //!< Drop all frames until the first I-frame (after each (re-)connect)

    // Decoding (only effective if the Fixposition SDK was built with FFmpeg, see FPSDK_USE_FFMPEG)
    bool decode_ = false;           //!< Decode the video frames and notify image observers
    bool decode_mono_ = false;      //!< Decode to greyscale (Y8) instead of RGB (RGB24). Cheaper.
    double decode_scale_ = 1.0;     //!< Scale factor for the decoded image, 0.1-1.0
    std::string decode_accel_ = "auto";  //!< Hw acceleration "auto", "sw" or "hw"
};

/**
 * @brief Counters, for diagnostics
 */
struct CameraStats {
    std::atomic<uint64_t> n_connect_{ 0 };     //!< Successful connects
    std::atomic<uint64_t> n_frames_{ 0 };      //!< Frames delivered to the frame observers
    std::atomic<uint64_t> n_dropped_{ 0 };     //!< Frames dropped (while waiting for the first I-frame, invalid frames)
    std::atomic<uint64_t> n_lost_{ 0 };        //!< Lost frames, detected from gaps in the frame times
    std::atomic<uint64_t> n_seq_anomalies_{ 0 }; //!< Discontinuities in the sensor's frame counter (not necessarily lost frames)
    std::atomic<uint64_t> n_images_{ 0 };      //!< Images decoded
    std::atomic<uint64_t> n_decode_fail_{ 0 }; //!< Frames that failed to decode
};

/**
 * @brief Camera streamer
 */
class CameraStreamer : private boost::noncopyable {
   public:
    /**
     * @brief Constructor
     *
     * @param[in]  params  Parameters
     */
    explicit CameraStreamer(const CameraParams& params);

    /**
     * @brief Destructor (stops the streamer)
     */
    virtual ~CameraStreamer();

    //! Observer for received frames. Called in the streamer thread. For video this is one encoded frame (all NAL
    //! units for one picture), starting at an I-frame (see CameraParams::wait_iframe_).
    using FrameObserver = std::function<void(const fpsdk::common::cam::CamData&)>;

    /**
     * @brief Add observer for received frames
     *
     * @note Add observers before StartStreamer(). The observers are called from the streamer thread, they must not
     *       block for long (the TCP stream backs up) and must do their own locking if needed.
     */
    void AddFrameObserver(FrameObserver observer);

#if FPSDK_USE_FFMPEG
    //! Observer for decoded images. Called in the streamer thread, after the frame observers.
    using ImageObserver =
        std::function<void(const fpsdk::common::cam::CamData&, const fpsdk::common::video::ImageData&)>;

    /**
     * @brief Add observer for decoded images (only if CameraParams::decode_)
     */
    void AddImageObserver(ImageObserver observer);
#endif

    /**
     * @brief Start the streamer thread
     *
     * This returns immediately. Connecting (and re-connecting) is done in the streamer thread, so a sensor that is
     * not (yet) reachable is not an error here.
     *
     * @returns true if the thread was started (false: bad params, or already started)
     */
    bool StartStreamer();

    /**
     * @brief Stop the streamer thread (blocks for up to CameraParams::timeout_)
     */
    void StopStreamer();

    //! Get the counters
    const CameraStats& GetStats() const { return stats_; }

    //! Get the parameters
    const CameraParams& GetParams() const { return params_; }

   private:
    CameraParams params_;
    CameraStats stats_;
    fpsdk::common::thread::Thread worker_;
    std::vector<FrameObserver> frame_observers_;
#if FPSDK_USE_FFMPEG
    std::vector<ImageObserver> image_observers_;
#endif

    bool Worker();                                       //!< Thread function: connect, stream, reconnect
    void Stream(fpsdk::common::cam::CamStream& stream);  //!< Receive frames until the connection is lost
};

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_CAMERA_STREAMER_HPP__
