#ifndef F_OMNI_KINEMATICS_HPP__
#define F_OMNI_KINEMATICS_HPP__

#include <vector>
#include <memory>

struct Omega{
    double rpm1, rpm2, rpm3, rpm4; // in [rad/s] --> by default
};

struct BodyVel{
    double avg_u, avg_v, avg_r;
};

class FOmniKinematics{
private:
    double a_, l_, w_, t_;
    std::vector<double> body_vel_lim_;
    Omega omega_;
    BodyVel avg_body_vel_;

public:
    FOmniKinematics();
    FOmniKinematics(double d, double L, double W, double t);
    ~FOmniKinematics();

    Omega getRPM(double u, double v, double r);
    BodyVel getVelocities(Omega omega);
};



#endif