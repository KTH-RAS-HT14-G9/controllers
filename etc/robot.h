#ifndef ROBOT_H
#define ROBOT_H

#include <math.h>

namespace robot {

namespace dim {
    double robot_diameter = 0.23;
    double robot_height = 0.28;
    double wheel_radius = 0.049;
    double wheel_distance = 0.21;
}

namespace prop {
    double ticks_per_rev = 360.0;
    double max_velocity = 5.0;
    double max_rot_velocity = M_PI/4.0;
}

}

#endif // ROBOT_H
