#include "qdevice_ximu3/QDeviceAdapter.hpp"

namespace qdevice_ximu3 {

QDeviceAdapter::QDeviceAdapter() = default;

QDeviceAdapter::~QDeviceAdapter() {
    stopAcquisition();
    disconnect();
}

bool QDeviceAdapter::initialize(const QDeviceConfig& config) {
    config_ = config;
    manager_.clearDevices();

    if (config_.auto_discover_wifi) {
        const auto found = manager_.discoverDevices(config_.discovery_timeout_ms);
        for (size_t i = 0; i < found.size(); ++i) {
            const auto& d = found[i];
            ManagedImuConfig dev_cfg;
            dev_cfg.label = d.device_name.empty()
                ? ("IMU_" + std::to_string(i + 1))
                : (d.device_name + "_" + d.serial_number);
            dev_cfg.serial_number = d.serial_number;
            dev_cfg.transport     = d.transport;
            dev_cfg.address       = (d.transport == TransportType::Usb) ? d.port_name : d.ip_address;
            // Swap send/receive ports: announcement's udp_receive is where the IMU listens, udp_send is where it transmits.
            dev_cfg.send_port     = d.udp_receive_port;
            dev_cfg.receive_port  = d.udp_send_port;
            dev_cfg.channels      = config_.channels;
            manager_.addDevice(dev_cfg);
        }
    }

    return manager_.deviceCount() > 0;
}

bool QDeviceAdapter::connect() {
    return manager_.connectAll() > 0;
}

void QDeviceAdapter::disconnect() {
    acquiring_ = false;
    manager_.disconnectAll();
}

bool QDeviceAdapter::startAcquisition() {
    if (!isConnected()) {
        return false;
    }
    manager_.resetBuffers();
    acquiring_ = true;
    return true;
}

void QDeviceAdapter::stopAcquisition() {
    acquiring_ = false;
}

bool QDeviceAdapter::readFrames(size_t num_frames, std::vector<float>& out_interleaved) {
    if (!acquiring_) {
        return false;
    }
    return manager_.readUniformFrames(
        config_.sample_rate_hz,
        num_frames,
        config_.resample_mode,
        out_interleaved);
}

std::vector<ChannelDescriptor> QDeviceAdapter::getChannels() const {
    return manager_.getAllChannelDescriptors();
}

bool QDeviceAdapter::isConnected() const {
    return manager_.connectedCount() > 0;
}

bool QDeviceAdapter::isAcquiring() const {
    return acquiring_;
}

} // namespace qdevice_ximu3
