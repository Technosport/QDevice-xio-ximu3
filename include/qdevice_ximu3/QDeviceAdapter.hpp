#pragma once

#include "qdevice_ximu3/ImuDeviceManager.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace qdevice_ximu3 {

struct QDeviceConfig {
    double sample_rate_hz = 400.0;
    ResampleMode resample_mode = ResampleMode::Linear;
    ChannelGroup channels = ChannelGroup::DefaultBiomechanics;
    bool auto_discover_wifi = true;
    uint32_t discovery_timeout_ms = 1200;
};

/// Clean lifecycle adapter between Qualisys QDevice SDK (0.10) and ImuDeviceManager.
/// Once QDevice SDK headers are placed in third_party/qdevice_sdk/, QDevicePlugin.cpp
/// forwards QTM's DLL callbacks directly to this class.
class QDeviceAdapter {
public:
    QDeviceAdapter();
    ~QDeviceAdapter();

    bool initialize(const QDeviceConfig& config);
    bool connect();
    void disconnect();

    bool startAcquisition();
    void stopAcquisition();

    /// Called periodically by QTM's acquisition thread to pull `num_frames` samples.
    bool readFrames(size_t num_frames, std::vector<float>& out_interleaved);

    std::vector<ChannelDescriptor> getChannels() const;
    bool isConnected() const;
    bool isAcquiring() const;

private:
    QDeviceConfig config_{};
    ImuDeviceManager manager_;
    bool acquiring_ = false;
};

} // namespace qdevice_ximu3
