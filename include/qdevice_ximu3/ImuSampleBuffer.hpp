#pragma once

#include "ChannelLayout.hpp"

#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <vector>

namespace qdevice_ximu3 {

enum class ResampleMode {
    ZeroOrderHold,
    Linear,
};

struct ImuStateSnapshot {
    uint64_t timestamp_us = 0;

    // Quaternion
    float quat_w = 1.0f, quat_x = 0.0f, quat_y = 0.0f, quat_z = 0.0f;
    // Euler (deg)
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    // Inertial
    float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
    float acc_x = 0.0f, acc_y = 0.0f, acc_z = 0.0f;
    // Magnetometer
    float mag_x = 0.0f, mag_y = 0.0f, mag_z = 0.0f;
    // Linear Acceleration
    float lin_acc_x = 0.0f, lin_acc_y = 0.0f, lin_acc_z = 0.0f;
    // Hardware Sync (BNC accessory)
    float sync_edge = 0.0f;
    // Diagnostics
    float battery_pct = 0.0f;
    float rssi_pct = 0.0f;
};

struct SyncEvent {
    uint64_t imu_timestamp_us = 0;
    float edge = 0.0f;
};

struct HighGSample {
    uint64_t timestamp_us = 0;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/// Thread-safe per-IMU buffer that ingests asynchronous x-IMU3 callbacks and
/// resamples them onto a uniform QTM sample grid (e.g. 100 Hz, 200 Hz, 400 Hz).
class ImuSampleBuffer {
public:
    explicit ImuSampleBuffer(ChannelGroup groups = ChannelGroup::DefaultBiomechanics,
                             size_t max_history = 4096);

    void setChannelGroups(ChannelGroup groups);
    ChannelGroup channelGroups() const;
    size_t channelCount() const;

    // Ingestion methods called from x-IMU3 SDK background threads
    void pushQuaternion(uint64_t timestamp_us, float w, float x, float y, float z);
    void pushEuler(uint64_t timestamp_us, float roll, float pitch, float yaw);
    void pushInertial(uint64_t timestamp_us,
                      float gx, float gy, float gz,
                      float ax, float ay, float az);
    void pushMagnetometer(uint64_t timestamp_us, float mx, float my, float mz);
    void pushLinearAcceleration(uint64_t timestamp_us, float ax, float ay, float az);
    void pushHighGAccelerometer(uint64_t timestamp_us, float ax, float ay, float az);
    void pushSync(uint64_t timestamp_us, float edge);
    void pushBattery(uint64_t timestamp_us, float percentage);
    void pushRssi(uint64_t timestamp_us, float percentage);

    /// Resets buffered history (e.g. on acquisition start).
    void clear();

    /// Returns the latest snapshot timestamp in microseconds, or 0 if empty.
    uint64_t latestTimestampUs() const;

    /// Returns the latest High-G accelerometer timestamp in microseconds, or 0 if empty.
    uint64_t latestHighGTimestampUs() const;

    /// Samples the High-G accelerometer buffer at a specific IMU timestamp (in microseconds).
    bool sampleHighGAt(uint64_t query_timestamp_us,
                       ResampleMode mode,
                       float& out_x, float& out_y, float& out_z) const;

    /// Samples the buffer at a specific IMU timestamp (in microseconds).
    /// Returns false if no data has been received yet.
    bool sampleAt(uint64_t query_timestamp_us,
                  ResampleMode mode,
                  std::vector<float>& out_channels) const;

    /// Generates `num_frames` uniformly spaced frames starting at `start_timestamp_us`
    /// with step `dt_us = 1'000'000 / sample_rate_hz`.
    /// Appends `num_frames * channelCount()` floats to `out_interleaved`.
    bool sampleUniformGrid(uint64_t start_timestamp_us,
                           double sample_rate_hz,
                           size_t num_frames,
                           ResampleMode mode,
                           std::vector<float>& out_interleaved) const;

    /// Returns and clears recorded hardware sync events (for clock offset / drift tracking).
    std::vector<SyncEvent> drainSyncEvents();

    /// Total number of hardware sync events received since the last clear().
    uint64_t syncEventsReceived() const;

private:
    float syncEdgeAtLocked(uint64_t query_timestamp_us) const;
    void recordSnapshotLocked(uint64_t timestamp_us);
    void appendSnapshotToVector(const ImuStateSnapshot& snap, std::vector<float>& out) const;
    static ImuStateSnapshot interpolateSnapshots(const ImuStateSnapshot& a,
                                                 const ImuStateSnapshot& b,
                                                 double alpha);

    mutable std::mutex mutex_;
    ChannelGroup groups_;
    size_t max_history_;
    ImuStateSnapshot current_state_{};
    bool has_data_ = false;
    uint64_t sync_events_received_ = 0;
    std::deque<ImuStateSnapshot> history_;
    std::deque<HighGSample> high_g_history_;
    size_t max_high_g_history_ = 16384;
    std::deque<SyncEvent> sync_history_;
    std::vector<SyncEvent> sync_events_;
};

} // namespace qdevice_ximu3
