#ifndef ROBOT_H
#define ROBOT_H

#include <math.h>
#include <cmath.h>
#include <eigen3/Eigen/Core>

namespace robot {

namespace dim {
    const double robot_diameter = 0.23;
    const double robot_height = 0.28;
    const double wheel_radius = 0.049;
    const double wheel_distance = 0.21;
}

namespace prop {
    const double ticks_per_rev = 360.0;
    const double max_velocity = 5.0;
    const double max_rot_velocity = M_PI/4.0;
}

namespace ir {
    const int id_front_left = 0;
    const int id_front_right = 1;
    const int id_rear_left = 2;
    const int id_rear_right = 3;
    
    //in m
    const double offset_front_left = dim::robot_diameter/2.0 - 0.023;
    const double offset_front_right = dim::robot_diameter/2.0 - 0.024;
    const double offset_rear_left = dim::robot_diameter/2.0 - 0.034;
    const double offset_rear_right = dim::robot_diameter/2.0 - 0.032;
    
    static double distance(int id, int voltage) {
        double dist= 0;
        switch(id) {
            case id_front_left:
            {
               double FL_p1=-0.8745;
               double FL_p2= 4.209;
               double FL_p3=-6.922;
               double FL_p4= 5.957;
               double FL_p5=-7.192;
               double FL_p6= 12.31;
               dist=FL_p1*pow(voltage,5) + FL_p2*pow(voltage,4) + FL_p3*pow(voltage,3) + FL_p4*pow(voltage,2) + FL_p5*voltage + FL_p6;
               
                return dist;
            }
            case id_front_right:
            {
               double FR_p1=-0.7935;
               double FR_p2=3.638;
               double FR_p3=-5.976;
               double FR_p4=5.676;
               double FR_p5=-7.334;
               double FR_p6=12.45;
               dist=FR_p1*pow(voltage,5) + FR_p2*pow(voltage,4) + FR_p3*pow(voltage,3) + FR_p4*pow(voltage,2) + FR_p5*voltage + FR_p6;
                return dist;
            }
            case id_rear_left:
            {    
                double BL_a=8753;
                double BL_b=-1.245;
                double BL_c=2.145;
                dist=BL_a*pow(x,BL_b)+BL_c;
                return dist;
            }
            case id_rear_right:
            {
               double BR_p1=-0.7276;
               double BR_p2=2.757;
               double BR_p3=-3.964;
               double BR_p4=4.782;
               double BR_p5=-7.936;
               double BR_p6=12.86;
               dist=BR_p1*pow(voltage,5) + BR_p2*pow(voltage,4) + BR_p3*pow(voltage,3) + BR_p4*pow(voltage,2) + BR_p5*voltage + BR_p6;
                
                return dist;
            }
            default:
            {
                return std::numeric_limits<double>::quiet_NaN();
            }
        }
    }
}

/**
  * Calculates the number of ticks per wheel to achieve the desired rotation.
  * @param rot Desired rotation in degrees
  * @return Vector2d, where the first coefficient denotes the delta ticks for
  *         the left wheel. The values are rounded down.
  */
static Vector2i Delta_ticks_for_rotation(double rot) {

    //boil rotation down to +-180 deg
    for(;rot >  180.0; rot -= 360.0);
    for(;rot < -180.0; rot += 360.0);

    double turn_circumference = M_PI*dim::robot_diameter;
    double s = turn_circumference * (rot/360.0);

    double s_per_rev = 2.0*M_PI*dim::wheel_radius;
    int delta_ticks_right = (int) ( prop::ticks_per_rev * (s / s_per_rev));
    int delta_ticks_left  = -delta_ticks_right;

    Vector2i delta_ticks(delta_ticks_left, delta_ticks_right);
    return delta_ticks;
}

}

#endif // ROBOT_H
