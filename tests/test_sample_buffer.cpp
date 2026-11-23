#include "qdevice_ximu3/ChannelLayout.hpp"
#include "qdevice_ximu3/ImuSampleBuffer.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace qdevice_ximu3;

namespace {

bool approx(float a, float b, float eps = 1e-4f) {
    return std::fabs(a - b) <= eps;
}

void testChannelLayout() {
    const auto biomech = buildChannelDescriptors("Pelvis", ChannelGroup::DefaultBiomechanics);
    // Quaternion(4) + Inertial(6) + Sync(1) = 11
    assert(biomech.size() == 11);
    assert(biomech[0].name == "Pelvis Quat W");
    assert(biomech[4].name == "Pelvis Gyro X");
    assert(biomech[10].name == "Pelvis Sync Edge");

    const auto all = buildChannelDescriptors("", ChannelGroup::All);
    // 4 + 3 + 6 + 3 + 3 + 1 + 2 = 22
    assert(all.size() == 22);
    assert(all[0].name == "Quat W");
    assert(all.back().name == "Rssi Pct");
}

void testZeroOrderHoldAndLinearResampling() {
    ImuSampleBuffer buf(ChannelGroup::Quaternion | ChannelGroup::Inertial);
    assert(buf.channelCount() == 10);

    // Push at t=1000 us and t=3500 us (dt=2500 us, i.e. 400 Hz)
    buf.pushQuaternion(1000, 1.0f, 0.0f, 0.0f, 0.0f);
    buf.pushInertial(1000, 0.0f, 10.0f, 20.0f, 0.0f, 0.0f, 1.0f);

    // Opposite-sign equivalent quaternion at t=3500 us to test shortest-path NLERP
    buf.pushQuaternion(3500, -1.0f, 0.0f, 0.0f, 0.0f);
    buf.pushInertial(3500, 100.0f, 30.0f, 40.0f, 1.0f, 2.0f, 3.0f);

    std::vector<float> out_zoh;
    assert(buf.sampleAt(2250, ResampleMode::ZeroOrderHold, out_zoh));
    assert(out_zoh.size() == 10);
    assert(approx(out_zoh[4], 0.0f)); // gyro_x held from t=1000

    std::vector<float> out_lin;
    assert(buf.sampleAt(2250, ResampleMode::Linear, out_lin));
    assert(out_lin.size() == 10);
    // Midpoint quaternion must remain unit-length and not flip through zero
    const float q_norm = std::sqrt(
        out_lin[0] * out_lin[0] + out_lin[1] * out_lin[1] +
        out_lin[2] * out_lin[2] + out_lin[3] * out_lin[3]);
    assert(approx(q_norm, 1.0f));
    assert(approx(out_lin[4], 50.0f)); // midpoint gyro_x
    assert(approx(out_lin[7], 0.5f));  // midpoint acc_x
}

void testUniformGridAndSyncEvents() {
    ImuSampleBuffer buf(ChannelGroup::Inertial | ChannelGroup::Sync);
    assert(buf.channelCount() == 7);

    buf.pushInertial(10000, 1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f);
    buf.pushSync(12500, 1.0f);
    buf.pushInertial(15000, 2.0f, 4.0f, 6.0f, 0.2f, 0.4f, 0.6f);

    std::vector<float> grid;
    // 3 frames @ 400 Hz (2500 us step): t=10000, 12500, 15000
    assert(buf.sampleUniformGrid(10000, 400.0, 3, ResampleMode::Linear, grid));
    assert(grid.size() == 21);
    assert(approx(grid[0], 1.0f));   // t=10000 gyro_x
    assert(approx(grid[7], 1.5f));   // t=12500 gyro_x (pure linear interp, undisturbed by pushSync)
    assert(approx(grid[13], 1.0f));  // t=12500 sync_edge
    assert(approx(grid[14], 2.0f));  // t=15000 gyro_x

    const auto events = buf.drainSyncEvents();
    assert(events.size() == 1);
    assert(events[0].imu_timestamp_us == 12500);
    assert(approx(events[0].edge, 1.0f));
    assert(buf.drainSyncEvents().empty());
}

} // namespace

int main() {
    testChannelLayout();
    testZeroOrderHoldAndLinearResampling();
    testUniformGridAndSyncEvents();
    std::cout << "All qdevice_ximu3 unit tests passed.\n";
    return 0;
}
