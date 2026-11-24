#pragma once

#include "qdevice_ximu3/ChannelLayout.hpp"
#include "qdevice_ximu3/ImuSampleBuffer.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace qdevice_ximu3 {

enum class TransportType {
    Udp,
    Tcp,
    Usb,
    Serial,
};

struct DiscoveredImu {
    std::string device_name;
    std::string serial_number;
    std::string ip_address;
    uint16_t tcp_port = 0;
    uint16_t udp_send_port = 0;
    uint16_t udp_receive_port = 0;
    std::string port_name; // USB / Serial
    TransportType transport = TransportType::Udp;
    int rssi_pct = 0;
    int battery_pct = 0;
};

struct ManagedImuConfig {
    std::string label;        // e.g. "Pelvis", "LeftFoot", or serial number
    std::string serial_number;
    TransportType transport = TransportType::Udp;
    std::string address;      // IP address (UDP/TCP) or COM port (USB/Serial)
    uint16_t send_port = 0;   // UDP send port or TCP port
    uint16_t receive_port = 0;// UDP receive port
    ChannelGroup channels = ChannelGroup::DefaultBiomechanics;
    bool enable_high_g = false;
    float high_g_rate_hz = 3200.0f;
};

/// Manages discovery, connection, and multi-sensor streaming for N x-IMU3 units.
class ImuDeviceManager {
public:
    ImuDeviceManager();
    ~ImuDeviceManager();

    ImuDeviceManager(const ImuDeviceManager&) = delete;
    ImuDeviceManager& operator=(const ImuDeviceManager&) = delete;

    /// Discovers x-IMU3 devices via UDP NetworkAnnouncement (Wi-Fi) and PortScanner (USB).
    std::vector<DiscoveredImu> discoverDevices(uint32_t wait_ms = 1200);

    /// Adds a sensor configuration before calling connectAll().
    void addDevice(const ManagedImuConfig& config);
    void clearDevices();

    /// Opens connections to all configured x-IMU3 devices. Returns number of connected devices.
    size_t connectAll();
    void disconnectAll();

    size_t connectedCount() const;
    size_t deviceCount() const;

    /// Returns the combined QTM channel descriptors across all configured IMUs.
    std::vector<ChannelDescriptor> getAllChannelDescriptors() const;

    /// Total number of channels per frame across all configured IMUs.
    size_t totalChannelCount() const;

    /// Clears buffered history on all IMUs (call when QTM starts a capture).
    void resetBuffers();

    /// Samples the latest synchronized state across all IMUs at `sample_rate_hz`
    /// for `num_frames` starting from each IMU's current aligned timestamp.
    /// Outputs `num_frames * totalChannelCount()` floats in channel-major frame order.
    bool readUniformFrames(double sample_rate_hz,
                           size_t num_frames,
                           ResampleMode mode,
                           std::vector<float>& out_interleaved);

    /// Samples High-G accelerometer (X, Y, Z) across all IMUs at `high_g_rate_hz`.
    /// Outputs `num_frames * deviceCount() * 3` floats in frame-major, device-major order.
    bool readUniformHighGFrames(double high_g_rate_hz,
                                size_t num_frames,
                                ResampleMode mode,
                                std::vector<float>& out_interleaved);

    /// Drains hardware sync events (BNC Sync Accessory) for a given device index.
    std::vector<SyncEvent> drainSyncEvents(size_t device_index);

    /// Total hardware sync events received across all devices since last resetBuffers().
    uint64_t totalSyncEventsReceived() const;

    /// Queries the sensor's `serial_mode` setting (4 = Sync). Returns nullopt if unreachable.
    std::optional<int> querySerialMode(size_t device_index) const;

private:
    struct DeviceEntry;
    std::vector<std::unique_ptr<DeviceEntry>> devices_;
    bool shared_cursor_initialized_ = false;
    double shared_cursor_us_ = 0.0;
    bool high_g_cursor_initialized_ = false;
    double high_g_cursor_us_ = 0.0;
};

} // namespace qdevice_ximu3
