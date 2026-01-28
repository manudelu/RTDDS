#pragma once
#include <cstdint>

template <size_t NUM_DOFS>
struct rt_joint_state_msg {
    uint64_t seq;
    uint64_t timestamp_ns;
    double positions[NUM_DOFS];
    double velocities[NUM_DOFS];
    double efforts[NUM_DOFS];
};

using SpotJointStateMsg = rt_joint_state_msg<12>;