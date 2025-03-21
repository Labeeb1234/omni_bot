#include <vector>
#include <memory>
#include <limits>
#include <utility>
#include <cmath>

#include "omni_bot_navigation/omni_bot_kinematics.hpp"


/*
    wheel configuration

        (x)
    (y)__|

        w1
    w2      w4
        w3    

*/

FOmniKinematics::FOmniKinematics() {}
FOmniKinematics::FOmniKinematics(double d, double L, double W, double t): a_(d/2), l_(L/2), w_(W/2), t_(t){}
FOmniKinematics::~FOmniKinematics() {}

Omega FOmniKinematics::getRPM(double u, double v, double r){
    omega_.rpm1 = 60*((v/a_+ r*(l_+t_)/a_)/(2*M_PI));
    omega_.rpm2 = 60*((-u/a_ + r*(w_+t_)/a_)/(2*M_PI));
    omega_.rpm3 = 60*((-v/a_ + r*(l_+t_)/a_)/(2*M_PI));
    omega_.rpm4 = 60*((u/a_ + r*(w_+t_)/a_)/(2*M_PI));
    return omega_; // in rpm
}

BodyVel FOmniKinematics::getVelocities(Omega omega){    
    avg_body_vel_.avg_u = (2*M_PI*a_)*((-omega.rpm2 + omega.rpm4)/2);
    avg_body_vel_.avg_v = (2*M_PI*a_)*((omega.rpm1 - omega.rpm3)/2);
    avg_body_vel_.avg_r = ((2*M_PI*a_)/(pow((l_+t_),2) + pow((w_+t_),2)))*((((l_+t_)*omega.rpm1)+((w_+t_)*omega.rpm2)+(l_+t_)*(omega.rpm3)+((w_+t_)*omega.rpm4))/2);
    return avg_body_vel_; // zeta (local/body frame velocities)
}

void FOmniKinematics::convert_to_rads(Omega& omega){
    omega.rpm1 = omega.rpm1*2*M_PI/60;
    omega.rpm2 = omega.rpm2*2*M_PI/60;
    omega.rpm3 = omega.rpm3*2*M_PI/60;
    omega.rpm4 = omega.rpm4*2*M_PI/60;
}



