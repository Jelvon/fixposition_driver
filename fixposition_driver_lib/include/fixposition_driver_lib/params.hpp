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
 * @brief Parameters
 */

#ifndef __FIXPOSITION_DRIVER_LIB_PARAMS_HPP__
#define __FIXPOSITION_DRIVER_LIB_PARAMS_HPP__

/* LIBC/STL */
#include <string>
#include <vector>

/* EXTERNAL */
#include <fpsdk_common/parser/fpa.hpp>

/* PACKAGE */

namespace fixposition {
/* ****************************************************************************************************************** */

/**
 * @brief Parameters for the Fixposition driver
 *
 * See launch/config.yaml for documentation
 */
struct DriverParams {
    std::string stream_;
    double reconnect_delay_ = 5.0;
    double delay_warning_ = 0.01;
    std::vector<std::string> messages_;
    fpsdk::common::parser::fpa::FpaEpoch nmea_epoch_ = fpsdk::common::parser::fpa::FpaEpoch::UNSPECIFIED;
    bool fusion_epoch_ = true;
    bool raw_output_ = false;
    bool cov_warning_ = false;
    bool nav2_mode_ = false;

    enum class VelTopicType { UNSPECIFIED, TWIST, TWISTWITHCOV, ODOMETRY };
    bool converter_enabled_ = false;
    VelTopicType converter_topic_type_ = VelTopicType::UNSPECIFIED;
    std::string converter_input_topic_;
    double converter_scale_factor_ = 1.0;
    bool converter_use_x_ = false;
    bool converter_use_y_ = false;
    bool converter_use_z_ = false;

    std::string output_ns_;
    std::string speed_topic_;
    std::string corr_topic_;
    std::string qos_type_;

    // Camera streaming (video from the cameras of a PBx-A1 sensor, via the sensor's HTTP API)
    bool camera_enabled_ = false;                          //!< Enable camera streaming
    std::string camera_host_;                              //!< Sensor host for the camera API, empty = host from stream_
    std::vector<std::string> camera_cams_ = { "CAM1" };    //!< Cameras to stream (CAM1, CAM2, ...)
    std::string camera_data_ = "HIRES_VID";                //!< Which stream (HIRES_VID or LORES_VID)
    bool camera_raw_output_ = true;                        //!< Publish the encoded video frames
    bool camera_decode_ = false;                           //!< Publish decoded images (needs FFmpeg in the SDK)
    bool camera_decode_mono_ = false;                      //!< Decode to greyscale instead of RGB
    double camera_decode_scale_ = 1.0;                     //!< Scale factor for decoded images (0.1 - 1.0)
    std::string camera_decode_accel_ = "auto";             //!< Decoder hw acceleration: "auto", "sw", "hw"

    // Check if entry is in messges_
    bool MessageEnabled(const std::string& message_name) const;

    /**
     * @brief Get the host name for the camera API
     *
     * @returns camera_host_ if set, otherwise the host part of a "tcpcli://<host>:<port>" stream_. Empty if neither
     *          is available (e.g. for a serial stream_).
     */
    std::string CameraHost() const;
};

bool StrToEpoch(const std::string& str, fpsdk::common::parser::fpa::FpaEpoch& epoch);

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_LIB_PARAMS_HPP__
