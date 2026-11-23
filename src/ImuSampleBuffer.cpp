#include "qdevice_ximu3/ImuSampleBuffer.hpp"

#include <algorithm>
#include <cmath>

namespace qdevice_ximu3 {

ImuSampleBuffer::ImuSampleBuffer(ChannelGroup groups, size_t max_history)
    : groups_(groups), max_history_(std::max<size_t>(max_history, 16)) {}

void ImuSampleBuffer::setChannelGroups(ChannelGroup groups) {
    std::lock_guard<std::mutex> lock(mutex_);
    groups_ = groups;
}

ChannelGroup ImuSampleBuffer::channelGroups() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return groups_;
}

size_t ImuSampleBuffer::channelCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t count = 0;
    if (hasGroup(groups_, ChannelGroup::Quaternion))   count += 4;
    if (hasGroup(groups_, ChannelGroup::Euler))        count += 3;
    if (hasGroup(groups_, ChannelGroup::Inertial))     count += 6;
    if (hasGroup(groups_, ChannelGroup::Magnetometer)) count += 3;
    if (hasGroup(groups_, ChannelGroup::LinearAccel))  count += 3;
    if (hasGroup(groups_, ChannelGroup::Sync))         count += 1;
    if (hasGroup(groups_, ChannelGroup::Diagnostics))  count += 2;
    return count;
}

void ImuSampleBuffer::recordSnapshotLocked(uint64_t timestamp_us) {
    current_state_.timestamp_us = timestamp_us;
    has_data_ = true;

    if (!history_.empty() && history_.back().timestamp_us == timestamp_us) {
        history_.back() = current_state_;
    } else if (!history_.empty() && timestamp_us < history_.back().timestamp_us) {
        auto it = std::lower_bound(
            history_.begin(), history_.end(), timestamp_us,
            [](const ImuStateSnapshot& s, uint64_t ts) { return s.timestamp_us < ts; });
        history_.insert(it, current_state_);
    } else {
        history_.push_back(current_state_);
    }

    while (history_.size() > max_history_) {
        history_.pop_front();
    }
}

void ImuSampleBuffer::pushQuaternion(uint64_t timestamp_us, float w, float x, float y, float z) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_state_.quat_w = w;
    current_state_.quat_x = x;
    current_state_.quat_y = y;
    current_state_.quat_z = z;
    recordSnapshotLocked(timestamp_us);
}

void ImuSampleBuffer::pushEuler(uint64_t timestamp_us, float roll, float pitch, float yaw) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_state_.roll = roll;
    current_state_.pitch = pitch;
    current_state_.yaw = yaw;
    recordSnapshotLocked(timestamp_us);
}

void ImuSampleBuffer::pushInertial(uint64_t timestamp_us,
                                   float gx, float gy, float gz,
                                   float ax, float ay, float az) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_state_.gyro_x = gx;
    current_state_.gyro_y = gy;
    current_state_.gyro_z = gz;
    current_state_.acc_x = ax;
    current_state_.acc_y = ay;
    current_state_.acc_z = az;
    recordSnapshotLocked(timestamp_us);
}

void ImuSampleBuffer::pushMagnetometer(uint64_t timestamp_us, float mx, float my, float mz) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_state_.mag_x = mx;
    current_state_.mag_y = my;
    current_state_.mag_z = mz;
    recordSnapshotLocked(timestamp_us);
}

void ImuSampleBuffer::pushLinearAcceleration(uint64_t timestamp_us, float ax, float ay, float az) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_state_.lin_acc_x = ax;
    current_state_.lin_acc_y = ay;
    current_state_.lin_acc_z = az;
    recordSnapshotLocked(timestamp_us);
}

void ImuSampleBuffer::pushSync(uint64_t timestamp_us, float edge) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Update sync state & event log without injecting a stale inertial snapshot into history_.
    current_state_.sync_edge = edge;
    sync_events_.push_back({timestamp_us, edge});
    sync_history_.push_back({timestamp_us, edge});
    while (sync_history_.size() > max_history_) {
        sync_history_.pop_front();
    }
    if (!has_data_) {
        recordSnapshotLocked(timestamp_us);
    }
}

void ImuSampleBuffer::pushBattery(uint64_t /*timestamp_us*/, float percentage) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_state_.battery_pct = percentage;
}

void ImuSampleBuffer::pushRssi(uint64_t /*timestamp_us*/, float percentage) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_state_.rssi_pct = percentage;
}

void ImuSampleBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    history_.clear();
    sync_history_.clear();
    sync_events_.clear();
    current_state_ = ImuStateSnapshot{};
    has_data_ = false;
}

uint64_t ImuSampleBuffer::latestTimestampUs() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_data_ ? current_state_.timestamp_us : 0;
}

void ImuSampleBuffer::appendSnapshotToVector(const ImuStateSnapshot& snap, std::vector<float>& out) const {
    if (hasGroup(groups_, ChannelGroup::Quaternion)) {
        out.push_back(snap.quat_w);
        out.push_back(snap.quat_x);
        out.push_back(snap.quat_y);
        out.push_back(snap.quat_z);
    }
    if (hasGroup(groups_, ChannelGroup::Euler)) {
        out.push_back(snap.roll);
        out.push_back(snap.pitch);
        out.push_back(snap.yaw);
    }
    if (hasGroup(groups_, ChannelGroup::Inertial)) {
        out.push_back(snap.gyro_x);
        out.push_back(snap.gyro_y);
        out.push_back(snap.gyro_z);
        out.push_back(snap.acc_x);
        out.push_back(snap.acc_y);
        out.push_back(snap.acc_z);
    }
    if (hasGroup(groups_, ChannelGroup::Magnetometer)) {
        out.push_back(snap.mag_x);
        out.push_back(snap.mag_y);
        out.push_back(snap.mag_z);
    }
    if (hasGroup(groups_, ChannelGroup::LinearAccel)) {
        out.push_back(snap.lin_acc_x);
        out.push_back(snap.lin_acc_y);
        out.push_back(snap.lin_acc_z);
    }
    if (hasGroup(groups_, ChannelGroup::Sync)) {
        out.push_back(snap.sync_edge);
    }
    if (hasGroup(groups_, ChannelGroup::Diagnostics)) {
        out.push_back(snap.battery_pct);
        out.push_back(snap.rssi_pct);
    }
}

ImuStateSnapshot ImuSampleBuffer::interpolateSnapshots(const ImuStateSnapshot& a,
                                                       const ImuStateSnapshot& b,
                                                       double alpha) {
    const float t = static_cast<float>(std::clamp(alpha, 0.0, 1.0));
    const auto lerp = [t](float v0, float v1) { return v0 + (v1 - v0) * t; };

    ImuStateSnapshot res{};
    res.timestamp_us = a.timestamp_us + static_cast<uint64_t>(
        static_cast<double>(b.timestamp_us - a.timestamp_us) * std::clamp(alpha, 0.0, 1.0));

    // Shortest-path normalized linear interpolation (NLERP) for quaternions
    float bw = b.quat_w, bx = b.quat_x, by = b.quat_y, bz = b.quat_z;
    const float dot = a.quat_w * bw + a.quat_x * bx + a.quat_y * by + a.quat_z * bz;
    if (dot < 0.0f) {
        bw = -bw; bx = -bx; by = -by; bz = -bz;
    }
    float qw = lerp(a.quat_w, bw);
    float qx = lerp(a.quat_x, bx);
    float qy = lerp(a.quat_y, by);
    float qz = lerp(a.quat_z, bz);
    const float norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm > 1e-6f) {
        res.quat_w = qw / norm;
        res.quat_x = qx / norm;
        res.quat_y = qy / norm;
        res.quat_z = qz / norm;
    } else {
        res.quat_w = 1.0f;
    }

    res.roll        = lerp(a.roll, b.roll);
    res.pitch       = lerp(a.pitch, b.pitch);
    res.yaw         = lerp(a.yaw, b.yaw);
    res.gyro_x      = lerp(a.gyro_x, b.gyro_x);
    res.gyro_y      = lerp(a.gyro_y, b.gyro_y);
    res.gyro_z      = lerp(a.gyro_z, b.gyro_z);
    res.acc_x       = lerp(a.acc_x, b.acc_x);
    res.acc_y       = lerp(a.acc_y, b.acc_y);
    res.acc_z       = lerp(a.acc_z, b.acc_z);
    res.mag_x       = lerp(a.mag_x, b.mag_x);
    res.mag_y       = lerp(a.mag_y, b.mag_y);
    res.mag_z       = lerp(a.mag_z, b.mag_z);
    res.lin_acc_x   = lerp(a.lin_acc_x, b.lin_acc_x);
    res.lin_acc_y   = lerp(a.lin_acc_y, b.lin_acc_y);
    res.lin_acc_z   = lerp(a.lin_acc_z, b.lin_acc_z);
    res.sync_edge   = b.sync_edge;
    res.battery_pct = b.battery_pct;
    res.rssi_pct    = b.rssi_pct;
    return res;
}

float ImuSampleBuffer::syncEdgeAtLocked(uint64_t query_timestamp_us) const {
    if (sync_history_.empty()) {
        return current_state_.sync_edge;
    }
    auto it = std::upper_bound(
        sync_history_.begin(), sync_history_.end(), query_timestamp_us,
        [](uint64_t ts, const SyncEvent& ev) { return ts < ev.imu_timestamp_us; });
    if (it == sync_history_.begin()) {
        return 0.0f;
    }
    return std::prev(it)->edge;
}

bool ImuSampleBuffer::sampleAt(uint64_t query_timestamp_us,
                               ResampleMode mode,
                               std::vector<float>& out_channels) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_data_ || history_.empty()) {
        return false;
    }

    ImuStateSnapshot snap{};
    if (query_timestamp_us <= history_.front().timestamp_us) {
        snap = history_.front();
    } else if (query_timestamp_us >= history_.back().timestamp_us) {
        snap = history_.back();
    } else {
        auto upper = std::lower_bound(
            history_.begin(), history_.end(), query_timestamp_us,
            [](const ImuStateSnapshot& s, uint64_t ts) { return s.timestamp_us < ts; });

        if (upper == history_.begin()) {
            snap = *upper;
        } else {
            const auto lower = std::prev(upper);
            if (mode == ResampleMode::ZeroOrderHold || upper->timestamp_us == lower->timestamp_us) {
                snap = *lower;
            } else {
                const double span = static_cast<double>(upper->timestamp_us - lower->timestamp_us);
                const double alpha = static_cast<double>(query_timestamp_us - lower->timestamp_us) / span;
                snap = interpolateSnapshots(*lower, *upper, alpha);
            }
        }
    }

    // Apply exact step-based sync edge & latest diagnostics
    snap.sync_edge   = syncEdgeAtLocked(query_timestamp_us);
    snap.battery_pct = current_state_.battery_pct;
    snap.rssi_pct    = current_state_.rssi_pct;

    appendSnapshotToVector(snap, out_channels);
    return true;
}

bool ImuSampleBuffer::sampleUniformGrid(uint64_t start_timestamp_us,
                                        double sample_rate_hz,
                                        size_t num_frames,
                                        ResampleMode mode,
                                        std::vector<float>& out_interleaved) const {
    if (sample_rate_hz <= 0.0 || num_frames == 0) {
        return false;
    }
    const double step_us = 1'000'000.0 / sample_rate_hz;
    for (size_t i = 0; i < num_frames; ++i) {
        const uint64_t ts = start_timestamp_us + static_cast<uint64_t>(std::llround(static_cast<double>(i) * step_us));
        if (!sampleAt(ts, mode, out_interleaved)) {
            return false;
        }
    }
    return true;
}

std::vector<SyncEvent> ImuSampleBuffer::drainSyncEvents() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<SyncEvent> events;
    events.swap(sync_events_);
    return events;
}

} // namespace qdevice_ximu3
