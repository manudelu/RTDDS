#pragma once

struct Cubic
{
    double a0_, a1_, a2_, a3_;

    Cubic() = default;

    Cubic(double q0, double qf, double T, double qd0 = 0.0, double qdf = 0.0)
            : a0_{q0}, a1_{qd0}
    {
        a2_ = (3*(qf - q0)/(T*T)) - (2*qd0/T) - (qdf/T);
        a3_ = (-2*(qf - q0)/(T*T*T)) + ((qd0 + qdf)/(T*T));
    }

    double position(double t) const { 
        return a0_ + a1_*t + a2_*t*t + a3_*t*t*t; 
    }
    double velocity(double t) const { 
        return a1_ + 2*a2_*t + 3*a3_*t*t; 
    }
};

struct Quintic
{
    double a0_, a1_, a2_, a3_, a4_, a5_;

    Quintic() = default;

    Quintic(double q0, double qf, double T , double qd0 = 0.0, double qdf = 0.0, double qdd0 = 0.0, double qddf = 0.0)
            : a0_{q0}, a1_{qd0}, a2_{qdd0/2.0}
    {
        a3_ = (20*qf - 20*q0 - (8*qdf + 12*qd0)*T - (3*qdd0 - qddf)*T*T) / (2*T*T*T);
        a4_ = (30*q0 - 30*qf + (14*qdf + 16*qd0)*T + (3*qdd0 - 2*qddf)*T*T) / (2*T*T*T*T);
        a5_ = (12*qf - 12*q0 - (6*qdf + 6*qd0)*T - (qdd0 - qddf)*T*T) / (2*T*T*T*T*T);
    }

    double position(double t) const { 
        return a0_ + a1_*t + a2_*t*t + a3_*t*t*t + a4_*t*t*t*t + a5_*t*t*t*t*t; 
    }
    double velocity(double t) const { 
        return a1_ + 2*a2_*t + 3*a3_*t*t + 4*a4_*t*t*t + 5*a5_*t*t*t*t; 
    }
};
