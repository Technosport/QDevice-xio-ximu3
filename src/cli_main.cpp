#include "qdevice_ximu3/QDeviceAdapter.hpp"

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

int main() {
    using namespace qdevice_ximu3;

    QDeviceConfig cfg;
    cfg.sample_rate_hz = 400.0;
    cfg.resample_mode = ResampleMode::Linear;
    cfg.channels = ChannelGroup::DefaultBiomechanics;
    cfg.auto_discover_wifi = true;
    cfg.discovery_timeout_ms = 1200;

    QDeviceAdapter adapter;
    std::cout << "[qdevice-ximu3] Scanning for x-IMU3 devices (Wi-Fi / USB)...\n";
    if (!adapter.initialize(cfg)) {
        std::cout << "[qdevice-ximu3] No x-IMU3 devices found on network/USB.\n";
        return 0;
    }

    const auto channels = adapter.getChannels();
    std::cout << "[qdevice-ximu3] Registered " << channels.size() << " QTM Generic channels:\n";
    for (const auto& ch : channels) {
        std::cout << "  - " << ch.name << " (" << ch.unit << ")\n";
    }

    if (!adapter.connect()) {
        std::cerr << "[qdevice-ximu3] Failed to open x-IMU3 connection(s).\n";
        return 1;
    }

    adapter.startAcquisition();
    std::cout << "[qdevice-ximu3] Streaming 10 batches of 10 frames @ " << cfg.sample_rate_hz << " Hz...\n";

    std::vector<float> frame_buf;
    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        if (adapter.readFrames(10, frame_buf) && !frame_buf.empty()) {
            std::cout << "Batch " << i << ": " << frame_buf.size()
                      << " floats (ch0=" << frame_buf[0] << ")\n";
        }
    }

    adapter.stopAcquisition();
    adapter.disconnect();
    return 0;
}
