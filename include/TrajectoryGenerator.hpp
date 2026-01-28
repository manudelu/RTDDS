#pragma once

#include <cstddef>
#include "RobotModel.hpp"

template <typename Traj, size_t MAX_DOFS>
class TrajectoryGenerator
{
    static_assert(MAX_DOFS > 0, "MAX_DOFS must be positive");
    
public:
    TrajectoryGenerator(const RobotModel<MAX_DOFS>& robot,
                        const double goal[MAX_DOFS],
                        double duration)
        : robot_{robot}, T_{duration}
    {
        for (size_t i {0}; i < MAX_DOFS; ++i) {
            double q0 = robot_.home_position[i];
            double qf = clamp(goal[i], robot_.joint_min[i], robot_.joint_max[i]);

            trajectories_[i] = Traj(q0, qf, T_);
        }
    }

    void compute(double t, double pos[MAX_DOFS], double vel[MAX_DOFS]) const
    {
        const double tc = (t < T_) ? t : T_;

        for (size_t i {0}; i < MAX_DOFS; ++i)
        {
            pos[i] = trajectories_[i].position(tc);
            vel[i] = trajectories_[i].velocity(tc);
        }
    }

private:
    static double clamp(double q, double low, double high)
    {
        return (q < low) ? low : ((q > high) ? high : q);
    }

    const RobotModel<MAX_DOFS>& robot_;
    double T_;
    Traj trajectories_[MAX_DOFS];
};
