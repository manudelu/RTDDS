#pragma once
#include <string>
#include <vector>
#include <stdexcept>

struct RobotModel
{
    std::vector<std::string> joint_names;
    std::vector<double> joint_min;
    std::vector<double> joint_max;
    std::vector<double> home_position;

    size_t dofs() const { return joint_names.size(); }

    void validate() const
    {
        if (joint_names.size() != joint_min.size() ||
            joint_names.size() != joint_max.size() ||
            joint_names.size() != home_position.size())
            throw std::runtime_error("RobotModel: size mismatch");
    }
};

RobotModel robot {
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

std::vector<double> end_pos { 0.0, 0.4, -0.5, 0.0, 0.5, -0.4, 0.0, 0.4, -0.5, 0.0, 0.5, -0.4 };