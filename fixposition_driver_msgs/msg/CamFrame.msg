# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# One encoded video frame from a PBx-A1 camera stream (/api/v2/camera/data, HIRES_VID or LORES_VID)
#
# fpsdk::common::cam::CamData

std_msgs/Header header    # stamp = image time (middle of exposure, ROS time = POSIX), frame_id = camera frame
uint8   cam_id            # Camera: 1..4 (CAM1..CAM4)
string  data_type         # "HIRES_VID" or "LORES_VID"
string  format            # "H265_NAL" or "H264_NAL" (Annex-B, all NAL units of one picture)
bool    keyframe          # true = I-frame (decodable without previous frames, has VPS/SPS/PPS)
uint64  seq               # Sensor frame counter
uint32  width             # [px]
uint32  height            # [px]
float64 exposure          # Exposure duration [s]
uint8[] data              # Encoded frame
