#include "qdevice_ximu3/ImuDeviceManager.hpp"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include <qualisys/qdevice-api/qdevice_api.h>
#include <qualisys/qdevice-api/qdevice_utilities.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

using clock_t_qd = std::chrono::steady_clock;

constexpr const char* INTEGRATION_NAME    = "xio Technologies xIMU3";
constexpr const char* INTEGRATION_DESC    = "Wireless & USB IMU integration for x-io x-IMU3 (AHRS + Inertial + BNC Sync)";
constexpr const char* INTEGRATION_URL     = "https://x-io.co.uk/x-imu3/";
constexpr const char* INTEGRATION_VERSION = "0.2.1";

struct PluginState {
    std::mutex mutex;
    qd_state state = qd_state_disconnected;
    float sample_rate_hz = 400.0f;
    float high_g_sample_rate_hz = 3200.0f;
    std::int32_t discovery_timeout_ms = 1200;
    bool include_euler = false;
    bool include_high_g = false;
    bool include_magnetometer = true;
    bool include_linear_accel = false;
    bool include_diagnostics = true;
    bool bnc_hardware_sync = false;
    bool wifi_software_sync = true;

    qdevice_ximu3::ImuDeviceManager manager;
    std::vector<qdevice_ximu3::DiscoveredImu> discovered;
    std::vector<qdevice_ximu3::ChannelDescriptor> channels;
    std::vector<size_t> device_ch_offsets;
    std::vector<size_t> device_ch_counts;
    std::unordered_set<std::string> sync_only_serials;

    clock_t_qd::time_point stream_start_time{};
    std::optional<clock_t_qd::time_point> stream_stop_time{};
    std::uint64_t samples_read = 0;
    std::uint64_t high_g_samples_read = 0;
    std::string last_error;
};

qdevice_ximu3::ChannelGroup activeChannelGroup(const PluginState& st) {
    auto g = qdevice_ximu3::ChannelGroup::DefaultBiomechanics;
    if (st.include_euler)        g = g | qdevice_ximu3::ChannelGroup::Euler;
    if (st.include_magnetometer) g = g | qdevice_ximu3::ChannelGroup::Magnetometer;
    if (st.include_linear_accel) g = g | qdevice_ximu3::ChannelGroup::LinearAccel;
    if (st.include_diagnostics)  g = g | qdevice_ximu3::ChannelGroup::Diagnostics;
    return g;
}

PluginState& getState() {
    static PluginState instance;
    return instance;
}

qd_channel_unit toQdUnit(const std::string& unit) {
    if (unit == "deg")   return qd_channel_unit_degrees;
    if (unit == "deg/s") return qd_channel_unit_degrees_per_second;
    if (unit == "g")     return qd_channel_unit_g_forces;
    if (unit == "uT")    return qd_channel_unit_microteslas;
    return qd_channel_unit_none;
}

qd_call_status okStatus() {
    return { qd_call_status_type_ok, nullptr };
}

qd_call_status errorStatus(const std::string& msg) {
    auto& st = getState();
    st.last_error = msg;
    return { qd_call_status_type_error, st.last_error.c_str() };
}

std::string deviceSerial(const qdevice_ximu3::DiscoveredImu& d, size_t idx) {
    return d.serial_number.empty() ? ("XIMU3-" + std::to_string(idx + 1)) : d.serial_number;
}

void applyDevicePropertiesLocked(PluginState& st, const struct qd_configuration* config) {
    if (!config) return;
    for (size_t i = 0; i < config->devices.size; ++i) {
        const auto& dev = config->devices.data[i];
        if (!dev.serial_number.data || dev.serial_number.size == 0) continue;
        const std::string serial(dev.serial_number.data, dev.serial_number.size);
        bool sync_only = (st.sync_only_serials.count(serial) > 0);
        try {
            QDevice::Utilities::try_assign_property_value("Sync Only", dev.properties, sync_only);
        } catch (...) {
            continue;
        }
        if (sync_only) {
            st.sync_only_serials.insert(serial);
        } else {
            st.sync_only_serials.erase(serial);
        }
    }
}

void rebuildDevicesLocked(PluginState& st) {
    st.manager.clearDevices();
    st.device_ch_offsets.clear();
    st.device_ch_counts.clear();
    const auto normal_groups = activeChannelGroup(st);
    size_t offset = 0;
    for (size_t i = 0; i < st.discovered.size(); ++i) {
        const auto& d = st.discovered[i];
        const std::string serial = deviceSerial(d, i);
        const bool is_sync_only = (st.sync_only_serials.count(serial) > 0);
        const auto groups = is_sync_only ? qdevice_ximu3::ChannelGroup::Sync : normal_groups;

        qdevice_ximu3::ManagedImuConfig cfg;
        cfg.label = ""; // Empty label so channel names are pure Title Case ("Quat W", "Gyro X", ...)
        cfg.serial_number = d.serial_number;
        cfg.transport     = d.transport;
        cfg.address       = (d.transport == qdevice_ximu3::TransportType::Usb) ? d.port_name : d.ip_address;
        // Swap send/receive ports: announcement's udp_receive is where the IMU listens, udp_send is where it transmits.
        cfg.send_port     = d.udp_receive_port;
        cfg.receive_port  = d.udp_send_port;
        cfg.channels       = groups;
        cfg.enable_high_g  = st.include_high_g && !is_sync_only;
        cfg.high_g_rate_hz = st.high_g_sample_rate_hz;
        st.manager.addDevice(cfg);

        const size_t ch_count = qdevice_ximu3::buildChannelDescriptors("", groups).size();
        st.device_ch_offsets.push_back(offset);
        st.device_ch_counts.push_back(ch_count);
        offset += ch_count;
    }
    st.channels = st.manager.getAllChannelDescriptors();
}

void runDiscoveryLocked(PluginState& st) {
    st.discovered = st.manager.discoverDevices(static_cast<uint32_t>(st.discovery_timeout_ms));
    rebuildDevicesLocked(st);
}

qd_call_status QD_STDCALL actionLocateDevices(const struct qd_configuration* config) {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);
    if (st.state != qd_state_disconnected) {
        return errorStatus("Disconnect before locating devices");
    }
    if (config) {
        try {
            QDevice::Utilities::try_assign_property_value("Sample Rate Hz", config->properties, st.sample_rate_hz);
            QDevice::Utilities::try_assign_property_value("High-G Sample Rate Hz", config->properties, st.high_g_sample_rate_hz);
            QDevice::Utilities::try_assign_property_value("Discovery Timeout Ms", config->properties, st.discovery_timeout_ms);
            QDevice::Utilities::try_assign_property_value("Include High-G Accel", config->properties, st.include_high_g);
            QDevice::Utilities::try_assign_property_value("Include Magnetometer", config->properties, st.include_magnetometer);
            QDevice::Utilities::try_assign_property_value("Include Diagnostics", config->properties, st.include_diagnostics);
            QDevice::Utilities::try_assign_property_value("BNC Hardware Sync", config->properties, st.bnc_hardware_sync);
            QDevice::Utilities::try_assign_property_value("Wi-Fi Software Sync", config->properties, st.wifi_software_sync);
        } catch (...) {}
    }
    applyDevicePropertiesLocked(st, config);
    runDiscoveryLocked(st);
    return okStatus();
}

} // namespace

qd_call_status qd_get_integration_version(struct qd_static_chars* version) {
    *version = { INTEGRATION_VERSION, std::strlen(INTEGRATION_VERSION) };
    return okStatus();
}

qd_call_status qd_get_name(struct qd_static_chars* name) {
    *name = { INTEGRATION_NAME, std::strlen(INTEGRATION_NAME) };
    return okStatus();
}

qd_call_status qd_get_description(struct qd_static_chars* desc) {
    *desc = { INTEGRATION_DESC, std::strlen(INTEGRATION_DESC) };
    return okStatus();
}

qd_call_status qd_get_manufacturer_url(struct qd_static_chars* url) {
    *url = { INTEGRATION_URL, std::strlen(INTEGRATION_URL) };
    return okStatus();
}

qd_call_status qd_get_contracts(struct qd_contracts* contracts) {
    *contracts = { nullptr, 0 };
    return okStatus();
}

qd_call_status qd_get_category(qd_device_category* category) {
    *category = qd_device_category_imu;
    return okStatus();
}

qd_call_status qd_initialize() {
#ifdef _WIN32
    // x-IMU3 Rust API spawns detached worker threads (PortScanner/Connection/NetworkAnnouncement);
    // pin the DLL so FreeLibrary() during qdevice_validator runs never unmaps code under them.
    HMODULE self_mod = nullptr;
    GetModuleHandleExW(
        GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN,
        reinterpret_cast<LPCWSTR>(&qd_initialize),
        &self_mod);
#endif
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);
    st.state = qd_state_disconnected;
    st.samples_read = 0;
    return okStatus();
}

qd_call_status qd_deinitialize() {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);
    st.manager.disconnectAll();
    st.manager.clearDevices();
    st.discovered.clear();
    st.channels.clear();
    st.state = qd_state_disconnected;
    return okStatus();
}

qd_call_status qd_get_actions(struct qd_actions* actions) {
    static constexpr const char ACTION_NAME[] = "Locate Devices";
    static const qd_action kActions[] = {
        { { ACTION_NAME, sizeof(ACTION_NAME) - 1 }, &actionLocateDevices }
    };
    *actions = { kActions, 1 };
    return okStatus();
}

qd_call_status qd_get_configuration(struct qd_configuration* configuration) {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);

    if (st.state == qd_state_disconnected && st.discovered.empty()) {
        runDiscoveryLocked(st);
    } else if (st.state == qd_state_disconnected) {
        rebuildDevicesLocked(st);
    }

    *configuration = QDevice::Utilities::create_configuration(8, st.discovered.size());

    configuration->properties.data[0] = QDevice::Utilities::create_property(
        "Sample Rate Hz",
        false,
        qd_property_type_float32,
        std::to_string(st.sample_rate_hz),
        "400",
        "");

    configuration->properties.data[1] = QDevice::Utilities::create_property(
        "High-G Sample Rate Hz",
        false,
        qd_property_type_float32,
        std::to_string(st.high_g_sample_rate_hz),
        "3200",
        "");

    configuration->properties.data[2] = QDevice::Utilities::create_property(
        "Discovery Timeout Ms",
        false,
        qd_property_type_int32,
        std::to_string(st.discovery_timeout_ms),
        "1200",
        "");

    configuration->properties.data[3] = QDevice::Utilities::create_property(
        "Include High-G Accel", false, qd_property_type_boolean,
        st.include_high_g ? "true" : "false", "false", "");

    configuration->properties.data[4] = QDevice::Utilities::create_property(
        "Include Magnetometer", false, qd_property_type_boolean,
        st.include_magnetometer ? "true" : "false", "true", "");

    configuration->properties.data[5] = QDevice::Utilities::create_property(
        "Include Diagnostics", false, qd_property_type_boolean,
        st.include_diagnostics ? "true" : "false", "true", "");

    configuration->properties.data[6] = QDevice::Utilities::create_property(
        "BNC Hardware Sync", false, qd_property_type_boolean,
        st.bnc_hardware_sync ? "true" : "false", "false", "");

    configuration->properties.data[7] = QDevice::Utilities::create_property(
        "Wi-Fi Software Sync", false, qd_property_type_boolean,
        st.wifi_software_sync ? "true" : "false", "true", "");

    for (size_t d = 0; d < st.discovered.size(); ++d) {
        const auto& dev = st.discovered[d];
        const std::string serial = deviceSerial(dev, d);
        const bool is_sync_only = (st.sync_only_serials.count(serial) > 0);
        const bool has_high_g   = st.include_high_g && !is_sync_only;
        const size_t ch_offset  = st.device_ch_offsets[d];
        const size_t ch_count   = st.device_ch_counts[d];
        const size_t total_dev_ch = ch_count + (has_high_g ? 3 : 0);

        configuration->devices.data[d] = QDevice::Utilities::create_device(
            qd_device_category_imu, serial, total_dev_ch, 2);

        const std::string endpoint = (dev.transport == qdevice_ximu3::TransportType::Usb)
            ? dev.port_name
            : dev.ip_address;

        configuration->devices.data[d].properties.data[0] = QDevice::Utilities::create_property(
            "Endpoint", true, qd_property_type_string_utf8, endpoint, endpoint, "");

        configuration->devices.data[d].properties.data[1] = QDevice::Utilities::create_property(
            "Sync Only", false, qd_property_type_boolean,
            is_sync_only ? "true" : "false", "false", "");

        for (size_t c = 0; c < ch_count; ++c) {
            const auto& ch = st.channels[ch_offset + c];
            configuration->devices.data[d].channels.data[c] = QDevice::Utilities::create_channel(
                ch.name,
                qd_sample_encoding_float32,
                toQdUnit(ch.unit),
                st.sample_rate_hz);
        }

        if (has_high_g) {
            configuration->devices.data[d].channels.data[ch_count + 0] = QDevice::Utilities::create_channel(
                "HighG Acc X", qd_sample_encoding_float32, qd_channel_unit_g_forces, st.high_g_sample_rate_hz);
            configuration->devices.data[d].channels.data[ch_count + 1] = QDevice::Utilities::create_channel(
                "HighG Acc Y", qd_sample_encoding_float32, qd_channel_unit_g_forces, st.high_g_sample_rate_hz);
            configuration->devices.data[d].channels.data[ch_count + 2] = QDevice::Utilities::create_channel(
                "HighG Acc Z", qd_sample_encoding_float32, qd_channel_unit_g_forces, st.high_g_sample_rate_hz);
        }
    }

    return okStatus();
}

qd_call_status qd_free_configuration(struct qd_configuration* configuration) {
    QDevice::Utilities::free_configuration(*configuration);
    return okStatus();
}

qd_call_status qd_connect(const struct qd_configuration* config) {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);

    if (st.state != qd_state_disconnected) {
        return errorStatus("Device already connected");
    }

    try {
        QDevice::Utilities::try_assign_property_value("Sample Rate Hz", config->properties, st.sample_rate_hz);
        QDevice::Utilities::try_assign_property_value("High-G Sample Rate Hz", config->properties, st.high_g_sample_rate_hz);
        QDevice::Utilities::try_assign_property_value("Discovery Timeout Ms", config->properties, st.discovery_timeout_ms);
        QDevice::Utilities::try_assign_property_value("Include High-G Accel", config->properties, st.include_high_g);
        QDevice::Utilities::try_assign_property_value("Include Magnetometer", config->properties, st.include_magnetometer);
        QDevice::Utilities::try_assign_property_value("Include Diagnostics", config->properties, st.include_diagnostics);
        QDevice::Utilities::try_assign_property_value("BNC Hardware Sync", config->properties, st.bnc_hardware_sync);
        QDevice::Utilities::try_assign_property_value("Wi-Fi Software Sync", config->properties, st.wifi_software_sync);
        applyDevicePropertiesLocked(st, config);
    } catch (const std::exception& ex) {
        return errorStatus(std::string("Invalid configuration: ") + ex.what());
    }

    if (st.bnc_hardware_sync == st.wifi_software_sync) {
        return errorStatus("Select exactly one sync mode: either 'BNC Hardware Sync' OR 'Wi-Fi Software Sync'.");
    }

    runDiscoveryLocked(st);
    if (st.manager.deviceCount() == 0) {
        return errorStatus("No x-IMU3 devices found");
    }

    if (st.manager.connectAll() == 0) {
        return errorStatus("Failed to connect to any x-IMU3 device");
    }

    if (st.bnc_hardware_sync) {
        bool any_sync_mode = false;
        const bool has_sync_only = !st.sync_only_serials.empty();
        for (size_t d = 0; d < st.discovered.size(); ++d) {
            const std::string serial = deviceSerial(st.discovered[d], d);
            const bool is_sync_only = (st.sync_only_serials.count(serial) > 0);
            if (has_sync_only && !is_sync_only) {
                continue;
            }
            const auto mode = st.manager.querySerialMode(d);
            if (mode.has_value() && *mode == 4) {
                any_sync_mode = true;
            } else if (has_sync_only && is_sync_only) {
                st.manager.disconnectAll();
                return errorStatus("BNC Hardware Sync error: Sync Only device " + serial +
                                   " does not have serial_mode = 4 (Sync). Configure it in x-IMU3 GUI or switch to Wi-Fi Software Sync.");
            }
        }
        if (!any_sync_mode) {
            st.manager.disconnectAll();
            return errorStatus("BNC Hardware Sync error: no connected x-IMU3 has serial_mode = 4 (Sync). Enable Sync on the base IMU or switch to Wi-Fi Software Sync.");
        }
    }

    st.state = qd_state_connected;
    return okStatus();
}

qd_call_status qd_disconnect() {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);

    if (st.state == qd_state_disconnected) {
        return errorStatus("Device not connected");
    }

    st.stream_stop_time.reset();
    st.manager.disconnectAll();
    st.state = qd_state_disconnected;
    return okStatus();
}

qd_call_status qd_start_streaming() {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);

    if (st.state != qd_state_connected) {
        return errorStatus("Can't start stream, device not connected");
    }
    if (st.channels.empty()) {
        return errorStatus("Can't start stream, no channels configured");
    }

    st.manager.resetBuffers();
    st.stream_start_time = clock_t_qd::now();
    st.stream_stop_time.reset();
    st.samples_read = 0;
    st.high_g_samples_read = 0;
    st.state = qd_state_streaming;
    return okStatus();
}

qd_call_status qd_stop_streaming() {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);

    if (st.state != qd_state_streaming) {
        return errorStatus("Can't stop stream, device not streaming");
    }

    st.stream_stop_time = clock_t_qd::now();
    st.state = qd_state_connected;
    return okStatus();
}

qd_call_status qd_get_state(qd_state* state) {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);
    *state = st.state;
    return okStatus();
}

qd_call_status qd_read_samples(qd_sample_push_fn callback) {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);

    clock_t_qd::time_point effective_now{};
    if (st.state == qd_state_streaming) {
        effective_now = clock_t_qd::now();
    } else if (st.state == qd_state_connected && st.stream_stop_time.has_value()) {
        effective_now = *st.stream_stop_time;
    } else {
        return okStatus();
    }

    const double elapsed_sec = std::chrono::duration<double>(effective_now - st.stream_start_time).count();
    if (st.bnc_hardware_sync && st.state == qd_state_streaming &&
        elapsed_sec >= 2.5 && st.manager.totalSyncEventsReceived() == 0) {
        return errorStatus("BNC Hardware Sync error: no BNC sync signal received within 2.5s. Check BNC cable and QTM Synchronization -> Sync Out (use Camera Frequency), or switch to Wi-Fi Software Sync.");
    }
    const auto target_total = static_cast<std::uint64_t>(std::max(0.0, elapsed_sec * static_cast<double>(st.sample_rate_hz)));
    const auto high_g_target = st.include_high_g
        ? static_cast<std::uint64_t>(std::max(0.0, elapsed_sec * static_cast<double>(st.high_g_sample_rate_hz)))
        : 0;

    const size_t num_frames = (target_total > st.samples_read)
        ? static_cast<size_t>(target_total - st.samples_read)
        : 0;
    const size_t high_g_frames = (st.include_high_g && high_g_target > st.high_g_samples_read)
        ? static_cast<size_t>(high_g_target - st.high_g_samples_read)
        : 0;

    const size_t total_ch = st.channels.size();
    if (total_ch == 0 || st.discovered.empty()) {
        if (st.state != qd_state_streaming) {
            st.stream_stop_time.reset();
        }
        return okStatus();
    }

    if (num_frames > 0) {
        std::vector<float> interleaved;
        if (st.manager.readUniformFrames(
                static_cast<double>(st.sample_rate_hz),
                num_frames,
                qdevice_ximu3::ResampleMode::Linear,
                interleaved)) {
            std::vector<float> ch_buf(num_frames);

            for (size_t d = 0; d < st.discovered.size(); ++d) {
                const std::string serial = deviceSerial(st.discovered[d], d);
                const size_t ch_offset = st.device_ch_offsets[d];
                const size_t ch_count  = st.device_ch_counts[d];

                for (size_t c = 0; c < ch_count; ++c) {
                    const size_t global_ch = ch_offset + c;
                    for (size_t f = 0; f < num_frames; ++f) {
                        ch_buf[f] = interleaved[f * total_ch + global_ch];
                    }

                    qd_samples samples{};
                    samples.start_sample_number = st.samples_read;
                    samples.encoding            = qd_sample_encoding_float32;
                    samples.data                = reinterpret_cast<std::uint8_t*>(ch_buf.data());
                    samples.size                = num_frames;

                    callback(serial.c_str(), st.channels[global_ch].name.c_str(), samples);
                }
            }
            st.samples_read += num_frames;
        }
    }

    if (high_g_frames > 0) {
        std::vector<float> hg_interleaved;
        if (st.manager.readUniformHighGFrames(
                static_cast<double>(st.high_g_sample_rate_hz),
                high_g_frames,
                qdevice_ximu3::ResampleMode::Linear,
                hg_interleaved)) {
            static constexpr const char* kHighGNames[3] = { "HighG Acc X", "HighG Acc Y", "HighG Acc Z" };
            const size_t stride = st.discovered.size() * 3;
            std::vector<float> hg_buf(high_g_frames);

            for (size_t d = 0; d < st.discovered.size(); ++d) {
                const std::string serial = deviceSerial(st.discovered[d], d);
                if (st.sync_only_serials.count(serial) > 0) continue;

                for (size_t axis = 0; axis < 3; ++axis) {
                    for (size_t f = 0; f < high_g_frames; ++f) {
                        hg_buf[f] = hg_interleaved[f * stride + d * 3 + axis];
                    }

                    qd_samples samples{};
                    samples.start_sample_number = st.high_g_samples_read;
                    samples.encoding            = qd_sample_encoding_float32;
                    samples.data                = reinterpret_cast<std::uint8_t*>(hg_buf.data());
                    samples.size                = high_g_frames;

                    callback(serial.c_str(), kHighGNames[axis], samples);
                }
            }
            st.high_g_samples_read += high_g_frames;
        }
    }

    if (st.state != qd_state_streaming) {
        st.stream_stop_time.reset();
    }
    return okStatus();
}
