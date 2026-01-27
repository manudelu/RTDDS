#pragma once

struct Trajectory1D
{
    virtual ~Trajectory1D() = default;
    virtual double position(double t) const = 0;
    virtual double velocity(double t) const = 0;
};

struct Cubic final : public Trajectory1D
{
    double a0_, a1_, a2_, a3_;

    Cubic(double q0, double qf, double qd0, double qdf, double T)
            : a0_{q0}, a1_{qd0}
    {
        a2_ = (3*(qf - q0)/(T*T)) - (2*qd0/T) - (qdf/T);
        a3_ = (-2*(qf - q0)/(T*T*T)) + ((qd0 + qdf)/(T*T));
    }

    double position(double t) const override { 
        return a0_ + a1_*t + a2_*t*t + a3_*t*t*t; 
    }
    double velocity(double t) const override { 
        return a1_ + 2*a2_*t + 3*a3_*t*t; 
    }
};

struct Quintic final : public Trajectory1D
{
    double a0_, a1_, a2_, a3_, a4_, a5_;

    Quintic(double q0, double qf, double qd0, double qdf, double qdd0, double qddf, double T)
            : a0_{q0}, a1_{qd0}, a2_{qdd0/2.0}
    {
        a3_ = (20*qf - 20*q0 - (8*qdf + 12*qd0)*T - (3*qdd0 - qddf)*T*T) / (2*T*T*T);
        a4_ = (30*q0 - 30*qf + (14*qdf + 16*qd0)*T + (3*qdd0 - 2*qddf)*T*T) / (2*T*T*T*T);
        a5_ = (12*qf - 12*q0 - (6*qdf + 6*qd0)*T - (qdd0 - qddf)*T*T) / (2*T*T*T*T*T);
    }

    double position(double t) const override { 
        return a0_ + a1_*t + a2_*t*t + a3_*t*t*t + a4_*t*t*t*t + a5_*t*t*t*t*t; 
    }
    double velocity(double t) const override { 
        return a1_ + 2*a2_*t + 3*a3_*t*t + 4*a4_*t*t*t + 5*a5_*t*t*t*t; 
    }
};
