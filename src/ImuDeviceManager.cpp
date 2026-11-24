#include "qdevice_ximu3/ImuDeviceManager.hpp"

#include "Ximu3.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <thread>

namespace qdevice_ximu3 {

struct ImuDeviceManager::DeviceEntry {
    ManagedImuConfig config;
    std::unique_ptr<ImuSampleBuffer> buffer;
    std::unique_ptr<ximu3::ConnectionConfig> conn_config;
    std::unique_ptr<ximu3::Connection> connection;
    bool connected = false;

    // Callbacks must outlive the connection because ximu3::Connection stores &callback.
    std::function<void(ximu3::XIMU3_QuaternionMessage)> cb_quat;
    std::function<void(ximu3::XIMU3_EulerAnglesMessage)> cb_euler;
    std::function<void(ximu3::XIMU3_InertialMessage)> cb_inertial;
    std::function<void(ximu3::XIMU3_MagnetometerMessage)> cb_mag;
    std::function<void(ximu3::XIMU3_LinearAccelerationMessage)> cb_lin_acc;
    std::function<void(ximu3::XIMU3_HighGAccelerometerMessage)> cb_high_g;
    std::function<void(ximu3::XIMU3_SyncMessage)> cb_sync;
    std::function<void(ximu3::XIMU3_BatteryMessage)> cb_battery;
    std::function<void(ximu3::XIMU3_RssiMessage)> cb_rssi;
};

ImuDeviceManager::ImuDeviceManager() = default;

ImuDeviceManager::~ImuDeviceManager() {
    disconnectAll();
}

std::vector<DiscoveredImu> ImuDeviceManager::discoverDevices(uint32_t wait_ms) {
    std::vector<DiscoveredImu> results;

    // 1. Wi-Fi discovery via UDP NetworkAnnouncement
    bool announcement_started = false;
    {
        ximu3::NetworkAnnouncement announcement;
        if (announcement.getResult() == ximu3::XIMU3_ResultOk) {
            announcement_started = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
            const auto messages = announcement.getMessages();
            for (const auto& msg : messages) {
                DiscoveredImu dev;
                dev.device_name      = msg.device_name;
                dev.serial_number    = msg.serial_number;
                dev.ip_address       = msg.ip_address;
                dev.tcp_port         = msg.tcp_port;
                dev.udp_send_port    = msg.udp_send;
                dev.udp_receive_port = msg.udp_receive;
                dev.transport        = TransportType::Udp;
                dev.rssi_pct         = msg.rssi;
                dev.battery_pct      = msg.battery;
                results.push_back(std::move(dev));
            }
        }
    }
    if (announcement_started) {
        // NetworkAnnouncement::Drop sets *dropped = true without joining its 1 ms polling thread;
        // wait briefly so the thread exits before the host can FreeLibrary() this DLL.
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }

    // 2. Wired USB / Serial discovery via PortScanner
    const auto usb_devices = ximu3::PortScanner::scanFilter(ximu3::XIMU3_PortTypeUsb);
    for (const auto& d : usb_devices) {
        DiscoveredImu dev;
        dev.device_name   = d.device_name;
        dev.serial_number = d.serial_number;
        dev.port_name     = d.usb_connection_config.port_name;
        dev.transport     = TransportType::Usb;
        results.push_back(std::move(dev));
    }

    return results;
}

void ImuDeviceManager::addDevice(const ManagedImuConfig& config) {
    auto entry = std::make_unique<DeviceEntry>();
    entry->config = config;
    entry->buffer = std::make_unique<ImuSampleBuffer>(config.channels);
    devices_.push_back(std::move(entry));
}

void ImuDeviceManager::clearDevices() {
    disconnectAll();
    devices_.clear();
}

size_t ImuDeviceManager::connectAll() {
    size_t ok_count = 0;

    for (auto& entry : devices_) {
        if (entry->connected) {
            ++ok_count;
            continue;
        }

        switch (entry->config.transport) {
            case TransportType::Udp:
                entry->conn_config = std::make_unique<ximu3::UdpConnectionConfig>(
                    entry->config.address, entry->config.send_port, entry->config.receive_port);
                break;
            case TransportType::Tcp:
                entry->conn_config = std::make_unique<ximu3::TcpConnectionConfig>(
                    entry->config.address, entry->config.send_port);
                break;
            case TransportType::Usb:
                entry->conn_config = std::make_unique<ximu3::UsbConnectionConfig>(
                    entry->config.address);
                break;
            case TransportType::Serial:
                entry->conn_config = std::make_unique<ximu3::SerialConnectionConfig>(
                    entry->config.address, 115200, false);
                break;
        }

        entry->connection = std::make_unique<ximu3::Connection>(*entry->conn_config);
        ImuSampleBuffer* buf = entry->buffer.get();

        entry->cb_quat = [buf](ximu3::XIMU3_QuaternionMessage m) {
            buf->pushQuaternion(m.timestamp, m.w, m.x, m.y, m.z);
        };
        entry->cb_euler = [buf](ximu3::XIMU3_EulerAnglesMessage m) {
            buf->pushEuler(m.timestamp, m.roll, m.pitch, m.yaw);
        };
        entry->cb_inertial = [buf](ximu3::XIMU3_InertialMessage m) {
            buf->pushInertial(m.timestamp,
                              m.gyroscope_x, m.gyroscope_y, m.gyroscope_z,
                              m.accelerometer_x, m.accelerometer_y, m.accelerometer_z);
        };
        entry->cb_mag = [buf](ximu3::XIMU3_MagnetometerMessage m) {
            buf->pushMagnetometer(m.timestamp, m.x, m.y, m.z);
        };
        entry->cb_lin_acc = [buf](ximu3::XIMU3_LinearAccelerationMessage m) {
            buf->pushLinearAcceleration(m.timestamp, m.x, m.y, m.z);
        };
        entry->cb_high_g = [buf](ximu3::XIMU3_HighGAccelerometerMessage m) {
            buf->pushHighGAccelerometer(m.timestamp, m.x, m.y, m.z);
        };
        entry->cb_sync = [buf](ximu3::XIMU3_SyncMessage m) {
            buf->pushSync(m.timestamp, m.edge);
        };
        entry->cb_battery = [buf](ximu3::XIMU3_BatteryMessage m) {
            buf->pushBattery(m.timestamp, m.percentage);
        };
        entry->cb_rssi = [buf](ximu3::XIMU3_RssiMessage m) {
            buf->pushRssi(m.timestamp, m.percentage);
        };

        entry->connection->addQuaternionCallback(entry->cb_quat);
        entry->connection->addEulerAnglesCallback(entry->cb_euler);
        entry->connection->addInertialCallback(entry->cb_inertial);
        entry->connection->addMagnetometerCallback(entry->cb_mag);
        entry->connection->addLinearAccelerationCallback(entry->cb_lin_acc);
        entry->connection->addHighGAccelerometerCallback(entry->cb_high_g);
        entry->connection->addSyncCallback(entry->cb_sync);
        entry->connection->addBatteryCallback(entry->cb_battery);
        entry->connection->addRssiCallback(entry->cb_rssi);

        if (entry->connection->open() == ximu3::XIMU3_ResultOk) {
            entry->connected = true;
            ++ok_count;
            if (entry->config.transport == TransportType::Udp) {
                // Firmware v2.6.0+ defaults udp_ip_address to "0.0.0.0" (auto-detect host IP
                // on first incoming UDP packet). Ping immediately so the sensor locks onto
                // this PC's IP and starts streaming UDP data right away.
                entry->connection->ping();
            }
            // Configure sensor message rates to hardware maximums (400 Hz Inertial/AHRS,
            // 20 Hz Mag, 5 Hz Battery, 1 Hz RSSI, and 3200 Hz High-G when enabled).
            int high_g_div = 0;
            if (entry->config.enable_high_g) {
                const float r = entry->config.high_g_rate_hz;
                if (r >= 2400.0f)      high_g_div = 1; // 3200 Hz
                else if (r >= 1200.0f) high_g_div = 2; // 1600 Hz
                else if (r >= 600.0f)  high_g_div = 4; // 800 Hz
                else                   high_g_div = 8; // 400 Hz
            }
            std::vector<std::string> rate_cmds = {
                "{\"inertial_message_rate_divisor\":1}",
                "{\"ahrs_message_rate_divisor\":1}",
                "{\"magnetometer_message_rate_divisor\":1}",
                "{\"battery_message_rate_divisor\":1}",
                "{\"rssi_message_rate_divisor\":1}",
                "{\"high_g_accelerometer_message_rate_divisor\":" + std::to_string(high_g_div) + "}"
            };
            entry->connection->sendCommandsAsync(
                rate_cmds,
                [](const std::vector<std::optional<ximu3::CommandMessage>>&) {},
                1, 250);
        } else {
            entry->connection.reset();
            entry->conn_config.reset();
        }
    }

    return ok_count;
}

void ImuDeviceManager::disconnectAll() {
    for (auto& entry : devices_) {
        if (entry->connection) {
            entry->connection->close();
            entry->connection.reset();
        }
        entry->conn_config.reset();
        entry->connected = false;
    }
}

size_t ImuDeviceManager::connectedCount() const {
    return static_cast<size_t>(std::count_if(
        devices_.begin(), devices_.end(),
        [](const auto& e) { return e->connected; }));
}

size_t ImuDeviceManager::deviceCount() const {
    return devices_.size();
}

std::vector<ChannelDescriptor> ImuDeviceManager::getAllChannelDescriptors() const {
    std::vector<ChannelDescriptor> all;
    for (const auto& entry : devices_) {
        auto desc = buildChannelDescriptors(entry->config.label, entry->config.channels);
        all.insert(all.end(), desc.begin(), desc.end());
    }
    return all;
}

size_t ImuDeviceManager::totalChannelCount() const {
    size_t total = 0;
    for (const auto& entry : devices_) {
        total += entry->buffer->channelCount();
    }
    return total;
}

void ImuDeviceManager::resetBuffers() {
    shared_cursor_initialized_ = false;
    shared_cursor_us_ = 0.0;
    high_g_cursor_initialized_ = false;
    high_g_cursor_us_ = 0.0;
    for (auto& entry : devices_) {
        entry->buffer->clear();
        if (entry->connected && entry->connection && entry->config.transport == TransportType::Udp) {
            entry->connection->pingAsync([](std::optional<ximu3::XIMU3_PingResponse>) {});
        }
    }
}

bool ImuDeviceManager::readUniformFrames(double sample_rate_hz,
                                         size_t num_frames,
                                         ResampleMode mode,
                                         std::vector<float>& out_interleaved) {
    if (devices_.empty() || sample_rate_hz <= 0.0 || num_frames == 0) {
        return false;
    }

    const size_t total_ch = totalChannelCount();
    out_interleaved.assign(num_frames * total_ch, 0.0f);

    // Find the slowest/most-conservative latest timestamp across all active IMUs
    // so that all IMUs are sampled at the EXACT same synchronized network timestamp.
    uint64_t min_latest_ts = UINT64_MAX;
    bool any_data = false;
    for (const auto& entry : devices_) {
        const uint64_t ts = entry->buffer->latestTimestampUs();
        if (ts > 0) {
            min_latest_ts = std::min(min_latest_ts, ts);
            any_data = true;
        }
    }

    if (!any_data) {
        return true;
    }

    constexpr double kJitterMarginUs = 25'000.0; // 25 ms anti-jitter buffer
    constexpr double kMaxLagUs       = 150'000.0; // 150 ms max lag before catching up
    const double step_us = 1'000'000.0 / sample_rate_hz;
    const double batch_us = static_cast<double>(num_frames) * step_us;

    if (!shared_cursor_initialized_) {
        const double target_start = static_cast<double>(min_latest_ts) - batch_us - kJitterMarginUs;
        shared_cursor_us_ = (target_start > 0.0) ? target_start : 0.0;
        shared_cursor_initialized_ = true;
    } else {
        const double max_allowed = static_cast<double>(min_latest_ts);
        if (shared_cursor_us_ + batch_us < max_allowed - kMaxLagUs) {
            shared_cursor_us_ = max_allowed - kJitterMarginUs - batch_us;
        }
        if (shared_cursor_us_ > max_allowed) {
            shared_cursor_us_ = max_allowed;
        }
        if (shared_cursor_us_ < 0.0) {
            shared_cursor_us_ = 0.0;
        }
    }

    for (size_t f = 0; f < num_frames; ++f) {
        const uint64_t query_ts = static_cast<uint64_t>(
            std::min(static_cast<double>(min_latest_ts), std::max(0.0, shared_cursor_us_)));

        size_t ch_offset = 0;
        for (const auto& entry : devices_) {
            const size_t dev_ch = entry->buffer->channelCount();
            std::vector<float> sample;
            sample.reserve(dev_ch);
            if (entry->buffer->sampleAt(query_ts, mode, sample)) {
                for (size_t c = 0; c < dev_ch; ++c) {
                    out_interleaved[f * total_ch + ch_offset + c] = sample[c];
                }
            }
            ch_offset += dev_ch;
        }

        shared_cursor_us_ += step_us;
    }

    return true;
}

bool ImuDeviceManager::readUniformHighGFrames(double high_g_rate_hz,
                                              size_t num_frames,
                                              ResampleMode mode,
                                              std::vector<float>& out_interleaved) {
    if (devices_.empty() || high_g_rate_hz <= 0.0 || num_frames == 0) {
        return false;
    }

    const size_t stride = devices_.size() * 3;
    out_interleaved.assign(num_frames * stride, 0.0f);

    uint64_t min_latest_ts = UINT64_MAX;
    bool any_data = false;
    for (const auto& entry : devices_) {
        if (!entry->config.enable_high_g) continue;
        const uint64_t ts = entry->buffer->latestHighGTimestampUs();
        if (ts > 0) {
            min_latest_ts = std::min(min_latest_ts, ts);
            any_data = true;
        }
    }

    if (!any_data) {
        return true;
    }

    constexpr double kJitterMarginUs = 25'000.0;
    constexpr double kMaxLagUs       = 150'000.0;
    const double step_us = 1'000'000.0 / high_g_rate_hz;
    const double batch_us = static_cast<double>(num_frames) * step_us;

    if (!high_g_cursor_initialized_) {
        const double target_start = static_cast<double>(min_latest_ts) - batch_us - kJitterMarginUs;
        high_g_cursor_us_ = (target_start > 0.0) ? target_start : 0.0;
        high_g_cursor_initialized_ = true;
    } else {
        const double max_allowed = static_cast<double>(min_latest_ts);
        if (high_g_cursor_us_ + batch_us < max_allowed - kMaxLagUs) {
            high_g_cursor_us_ = max_allowed - kJitterMarginUs - batch_us;
        }
        if (high_g_cursor_us_ > max_allowed) {
            high_g_cursor_us_ = max_allowed;
        }
        if (high_g_cursor_us_ < 0.0) {
            high_g_cursor_us_ = 0.0;
        }
    }

    for (size_t f = 0; f < num_frames; ++f) {
        const uint64_t query_ts = static_cast<uint64_t>(
            std::min(static_cast<double>(min_latest_ts), std::max(0.0, high_g_cursor_us_)));

        for (size_t d = 0; d < devices_.size(); ++d) {
            if (!devices_[d]->config.enable_high_g) continue;
            float gx = 0.0f, gy = 0.0f, gz = 0.0f;
            if (devices_[d]->buffer->sampleHighGAt(query_ts, mode, gx, gy, gz)) {
                out_interleaved[f * stride + d * 3 + 0] = gx;
                out_interleaved[f * stride + d * 3 + 1] = gy;
                out_interleaved[f * stride + d * 3 + 2] = gz;
            }
        }

        high_g_cursor_us_ += step_us;
    }

    return true;
}

std::vector<SyncEvent> ImuDeviceManager::drainSyncEvents(size_t device_index) {
    if (device_index >= devices_.size()) {
        return {};
    }
    return devices_[device_index]->buffer->drainSyncEvents();
}

uint64_t ImuDeviceManager::totalSyncEventsReceived() const {
    uint64_t total = 0;
    for (const auto& entry : devices_) {
        total += entry->buffer->syncEventsReceived();
    }
    return total;
}

std::optional<int> ImuDeviceManager::querySerialMode(size_t device_index) const {
    if (device_index >= devices_.size() || !devices_[device_index]->connected ||
        !devices_[device_index]->connection) {
        return std::nullopt;
    }
    const auto resp = devices_[device_index]->connection->sendCommand("{\"serial_mode\":null}", 2, 250);
    if (!resp.has_value() || resp->error.has_value() || resp->value.empty()) {
        return std::nullopt;
    }
    try {
        return std::stoi(resp->value);
    } catch (...) {
        return std::nullopt;
    }
}

} // namespace qdevice_ximu3
