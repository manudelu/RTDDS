#pragma once
#include <vector>
#include <string>
#include <memory>
#include <algorithm>
#include <stdexcept>

#include "TrajectoryTypes.hpp"
#include "RobotModel.hpp"

enum class TrajectoryType 
{
    Cubic,
    Quintic
};

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(const RobotModel& robot,
                        const std::vector<double>& goal,
                        double duration,
                        TrajectoryType type)
        : robot_{robot}, T_{duration}
    {
        robot_.validate();

        if (goal.size() != robot_.dofs())
            throw std::runtime_error("Robot DOFs mismatch with N_JOINTS");

        trajectories_.reserve(robot_.dofs());

        for (size_t i {0}; i < robot_.dofs(); ++i)
        {
            double q0 { robot_.home_position[i] };
            double qf { clamp(goal[i], robot_.joint_min[i], robot_.joint_max[i]) };

            switch (type)
            {
            case TrajectoryType::Cubic:
                trajectories_.push_back(std::unique_ptr<Trajectory1D>(new Cubic(q0, qf, 0.0, 0.0, T_)));
                break;

            case TrajectoryType::Quintic:
                trajectories_.push_back(std::unique_ptr<Trajectory1D>(new Quintic(q0, qf, 0.0, 0.0, 0.0, 0.0, T_)));
                break;
            }
        }
    }

    void compute(double t, std::vector<double>& pos, std::vector<double>& vel) const
    {
        double tc { std::min(t, T_) };

        pos.resize(trajectories_.size());
        vel.resize(trajectories_.size());

        for (size_t i {0}; i < trajectories_.size(); ++i)
        {
            pos[i] = trajectories_[i]->position(tc);
            vel[i] = trajectories_[i]->velocity(tc);
        }
    }

    const std::vector<std::string>& joint_names() const
    {
        return robot_.joint_names;
    }

private:
    static double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(hi, v));
    }

    RobotModel robot_;
    double T_;
    std::vector<std::unique_ptr<Trajectory1D>> trajectories_;
};
