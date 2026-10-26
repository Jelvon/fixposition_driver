# Fixposition ROS2 Driver

[ROS](https://www.ros.org/) Driver for [Fixposition Vision-RTK 2](https://www.fixposition.com/product).

See the [main README.md](../README.md) for the documentation.

## Camera streaming (optional)

The driver can stream the video of the cameras of a PBx-A1 sensor into ROS 2. It is disabled by default.

### Usage

Set in `launch/config.yaml` (see the comments there):

```yaml
camera:
    enabled: true
    host: ""            # "" = use the host of the tcpcli:// "stream"
    cams: ["CAM1"]      # CAM1 .. CAM4
    data: "HIRES_VID"   # or LORES_VID
    raw_output: true    # publish the encoded frames
    decode: false       # publish decoded images (needs the Fixposition SDK built with FFmpeg)
```

Topics, per camera (`<output_ns>/cam1/...`):

| Topic       | Type                                | Content                                                             |
| ----------- | ----------------------------------- | ------------------------------------------------------------------- |
| `video`     | `fixposition_driver_msgs/CamFrame`  | One encoded H.264/H.265 frame (Annex-B) plus meta data. Reliable QoS |
| `image_raw` | `sensor_msgs/Image`                 | Decoded frame, `rgb8` (or `mono8`). Only with `decode: true`        |

`header.stamp` is the image time (middle of exposure) as reported by the sensor (its system clock, which is aligned
with GNSS time, in the recordings we checked to within the resolution of the check of about 40 ms), `header.frame_id` is the camera name (`cam1`, ...).
The stamp does not depend on when the frame arrives (network and encoding delay are in the order of 50 ms) or on the
clock of the computer running the driver. `CamFrame.keyframe` marks I-frames: a subscriber can only start decoding at one.

### Design

- **Separate from the message parser.** The camera data does not come through the byte stream that the `Parser`
  handles (FP_A, NMEA, NOV_B). It is a separate HTTP client of the sensor, so this is not a new parser but a new
  component next to `FixpositionDriver`.
- **`CameraStreamer`** (`fixposition_driver_lib`, no ROS dependency) handles one camera: one thread, one connection
  (`fpsdk::common::cam::CamStream`), reconnect with `reconnect_delay` (a watchdog closes a connection that stays open but silent for 3 s), observers for frames and decoded images. Like
  the rest of the lib it only hands out data; `fixposition_driver_ros2` turns it into messages.
- **Start at an I-frame.** After every (re-)connect all frames before the first I-frame are dropped, so downstream
  decoders never see a stream that starts mid-GOP.
- **Lost frames are visible.** Missing frames are detected from gaps in the frame times (the frame interval is constant)
  and are counted and logged, never hidden. The sensor's frame counter (`seq`) is passed on, but not used for this: it
  can jump without a frame being lost.
- **Encoded by default.** Publishing the encoded stream is cheap and lossless. Decoding is optional, runs in the
  streamer thread, and needs FFmpeg in the SDK. Without FFmpeg the driver builds and runs as before and offers
  the encoded stream only.
- **Clean shutdown.** The streamers are stopped before the publishers are destroyed.

### Limitations

- Only the video streams (`HIRES_VID`, `LORES_VID`), not the raw image streams.
- A camera can only be streamed once at a time (e.g. not together with the sensor's web interface), and the
  sensor must be connected via Ethernet.
- The decoder runs in the streamer thread: if it is too slow, the stream backs up and frames are lost on the
  sensor side (seen as lost frames in the log).

### Testing

`CameraStreamer` can be tested without a sensor against a small mock of the sensor's camera endpoint
(a raw H.264/H.265 file, or the camera data of a `.fpl` recording extracted with `fpltool extract -e jsonl`).
The tests check: normal streaming, join in the middle of a GOP, reconnect, a stalled (open but silent) connection, lost frames, `seq` jumps, decoding, and stop latency.
