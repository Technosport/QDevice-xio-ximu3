#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace qdevice_ximu3 {

/// Bitmask of sensor data groups to expose as QTM Generic channels per x-IMU3 unit.
enum class ChannelGroup : uint32_t {
    None        = 0,
    Quaternion  = 1 << 0, // QuatW, QuatX, QuatY, QuatZ
    Euler       = 1 << 1, // Roll, Pitch, Yaw (deg)
    Inertial    = 1 << 2, // GyroX/Y/Z (deg/s), AccX/Y/Z (g)
    Magnetometer= 1 << 3, // MagX/Y/Z (a.u.)
    LinearAccel = 1 << 4, // LinAccX/Y/Z (g)
    Sync        = 1 << 5, // SyncEdge (0.0 / 1.0 from BNC Sync Accessory)
    Diagnostics = 1 << 6, // BatteryPct, RssiPct

    DefaultBiomechanics = Quaternion | Inertial | Sync,
    All                 = Quaternion | Euler | Inertial | Magnetometer | LinearAccel | Sync | Diagnostics,
};

inline ChannelGroup operator|(ChannelGroup a, ChannelGroup b) {
    return static_cast<ChannelGroup>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline bool hasGroup(ChannelGroup mask, ChannelGroup flag) {
    return (static_cast<uint32_t>(mask) & static_cast<uint32_t>(flag)) != 0;
}

struct ChannelDescriptor {
    std::string name;
    std::string unit;
    float min_range = -1000.0f;
    float max_range = 1000.0f;
};

/// Generates deterministic QTM channel descriptors for a given IMU label and enabled groups.
/// If `imu_label` is empty, returns clean Title Case names required by QDevice Validator.
inline std::vector<ChannelDescriptor> buildChannelDescriptors(const std::string& imu_label, ChannelGroup groups) {
    std::vector<ChannelDescriptor> channels;
    const std::string prefix = imu_label.empty() ? "" : (imu_label + " ");

    if (hasGroup(groups, ChannelGroup::Quaternion)) {
        channels.push_back({prefix + "Quat W", "norm", -1.0f, 1.0f});
        channels.push_back({prefix + "Quat X", "norm", -1.0f, 1.0f});
        channels.push_back({prefix + "Quat Y", "norm", -1.0f, 1.0f});
        channels.push_back({prefix + "Quat Z", "norm", -1.0f, 1.0f});
    }
    if (hasGroup(groups, ChannelGroup::Euler)) {
        channels.push_back({prefix + "Roll",  "deg", -180.0f, 180.0f});
        channels.push_back({prefix + "Pitch", "deg", -90.0f,  90.0f});
        channels.push_back({prefix + "Yaw",   "deg", -180.0f, 180.0f});
    }
    if (hasGroup(groups, ChannelGroup::Inertial)) {
        channels.push_back({prefix + "Gyro X", "deg/s", -2000.0f, 2000.0f});
        channels.push_back({prefix + "Gyro Y", "deg/s", -2000.0f, 2000.0f});
        channels.push_back({prefix + "Gyro Z", "deg/s", -2000.0f, 2000.0f});
        channels.push_back({prefix + "Acc X",  "g",     -24.0f,   24.0f});
        channels.push_back({prefix + "Acc Y",  "g",     -24.0f,   24.0f});
        channels.push_back({prefix + "Acc Z",  "g",     -24.0f,   24.0f});
    }
    if (hasGroup(groups, ChannelGroup::Magnetometer)) {
        channels.push_back({prefix + "Mag X", "uT", -1300.0f, 1300.0f});
        channels.push_back({prefix + "Mag Y", "uT", -1300.0f, 1300.0f});
        channels.push_back({prefix + "Mag Z", "uT", -1300.0f, 1300.0f});
    }
    if (hasGroup(groups, ChannelGroup::LinearAccel)) {
        channels.push_back({prefix + "Lin Acc X", "g", -24.0f, 24.0f});
        channels.push_back({prefix + "Lin Acc Y", "g", -24.0f, 24.0f});
        channels.push_back({prefix + "Lin Acc Z", "g", -24.0f, 24.0f});
    }
    if (hasGroup(groups, ChannelGroup::Sync)) {
        channels.push_back({prefix + "Sync Edge", "bool", 0.0f, 1.0f});
    }
    if (hasGroup(groups, ChannelGroup::Diagnostics)) {
        channels.push_back({prefix + "Battery Pct", "%", 0.0f, 100.0f});
        channels.push_back({prefix + "Rssi Pct",    "%", 0.0f, 100.0f});
    }

    return channels;
}

} // namespace qdevice_ximu3
