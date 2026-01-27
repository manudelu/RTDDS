#pragma once
#include <cstdint>

#define N_JOINTS 12

struct rt_joint_state_msg {
    uint64_t seq;
    uint64_t timestamp_ns;
    double positions[N_JOINTS];
    double velocities[N_JOINTS];
    double efforts[N_JOINTS];
};