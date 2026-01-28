#pragma once

#include <cstddef>

template <size_t MAX_DOFS>
struct RobotModel
{
    static constexpr size_t dofs = MAX_DOFS;

    const char* joint_names[MAX_DOFS];
    double joint_min[MAX_DOFS];
    double joint_max[MAX_DOFS];
    double home_position[MAX_DOFS];
};

// Spot Config 12 Joints
using SpotRobot = RobotModel<12>;
constexpr SpotRobot spot {
    .joint_names = {
        "front_left_hip_x",  "front_left_hip_y",  "front_left_knee",
        "front_right_hip_x", "front_right_hip_y", "front_right_knee",
        "rear_left_hip_x",   "rear_left_hip_y",   "rear_left_knee",
        "rear_right_hip_x",  "rear_right_hip_y",  "rear_right_knee"
    },
    .joint_min = {
        -0.785, -0.899, -2.793,
        -0.785, -0.899, -2.793,
        -0.785, -0.899, -2.793,
        -0.785, -0.899, -2.793
    },
    .joint_max = {
        0.785, 2.295, -0.255,
        0.785, 2.295, -0.248,
        0.785, 2.295, -0.267,
        0.785, 2.295, -0.258
    },
    .home_position = {
        0.0, 0.0, -1.5238505,
        0.0, 0.0, -1.5202315,
        0.0, 0.0, -1.5300265,
        0.0, 0.0, -1.5253125
    }
};