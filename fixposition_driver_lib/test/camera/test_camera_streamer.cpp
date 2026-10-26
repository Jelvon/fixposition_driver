// Test harness for CameraStreamer against mock_sensor.py (no ROS). Not part of the driver build.
//   test_camera_streamer <host> <seconds> [decode] [expect_first_frm_I]
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fpsdk_common/time.hpp>
#include <mutex>
#include <thread>

#include "fixposition_driver_lib/camera_streamer.hpp"

using namespace fixposition;
using namespace fpsdk::common;

int main(int argc, char** argv) {
    if (argc < 3) {
        return 2;
    }
    CameraParams p;
    p.name_ = "cam1";
    p.host_ = argv[1];
    p.reconnect_delay_ = 1.0;
    p.timeout_ = 1.0;
    p.decode_ = (argc > 3) && (std::strcmp(argv[3], "decode") == 0);
    const double secs = std::atof(argv[2]);

    CameraStreamer cs(p);
    std::mutex mtx;
    uint64_t n = 0, n_i = 0;
    bool first_is_i = false, first = true, ts_ok = true;
    double lat_sum = 0.0;
    cs.AddFrameObserver([&](const cam::CamData& d) {
        std::lock_guard<std::mutex> l(mtx);
        if (first) {
            first = false;
            first_is_i = (d.frm_ == cam::CamDataFrm::I_FRAME);
            std::printf("first frame: seq=%lu frm=%s fmt=%s %ux%u dt=%u ns size=%zu\n", d.seq_,
                        cam::CamDataFrmToStr(d.frm_), cam::CamDataFmtToStr(d.fmt_), d.width_, d.height_, d.dt_,
                        d.data_.size());
        }
        n++;
        n_i += (d.frm_ == cam::CamDataFrm::I_FRAME);
        const double lat = (time::Time::FromClockRealtime() - time::Time::FromPosixNs(d.ts_)).GetSec() * 1e3;
        lat_sum += lat;
        if (d.ts_ == 0 || lat < -5 || lat > 1000) {
            ts_ok = false;
        }
    });
#if FPSDK_USE_FFMPEG
    uint64_t n_img = 0;
    int w = 0, h = 0;
    cs.AddImageObserver([&](const cam::CamData&, const video::ImageData& img) {
        std::lock_guard<std::mutex> l(mtx);
        n_img++;
        w = img.width_;
        h = img.height_;
        if (img.data_.size() != (size_t)img.width_ * img.height_ * 3) {
            std::printf("BAD IMAGE SIZE\n");
        }
    });
#endif
    if (!cs.StartStreamer()) {
        std::printf("start failed\n");
        return 1;
    }
    const auto t0 = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(secs * 1000)));
    const auto t_stop0 = std::chrono::steady_clock::now();
    cs.StopStreamer();
    const double stop_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_stop0).count();
    (void)t0;

    const auto& st = cs.GetStats();
    std::printf(
        "RESULT frames=%lu iframes=%lu first_is_I=%d connects=%lu dropped=%lu lost=%lu seq_anomalies=%lu "
        "mean_latency_ms=%.1f ts_ok=%d "
        "stop_ms=%.0f\n",
        (unsigned long)n, (unsigned long)n_i, first_is_i, (unsigned long)st.n_connect_.load(),
        (unsigned long)st.n_dropped_.load(), (unsigned long)st.n_lost_.load(),
        (unsigned long)st.n_seq_anomalies_.load(), n ? lat_sum / n : 0.0, ts_ok, stop_ms);
#if FPSDK_USE_FFMPEG
    std::printf("RESULT images=%lu decode_fail=%lu size=%dx%d\n", (unsigned long)n_img,
                (unsigned long)st.n_decode_fail_.load(), w, h);
#endif
    return 0;
}
