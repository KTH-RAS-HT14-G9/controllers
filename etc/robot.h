#ifndef ROBOT_H
#define ROBOT_H

#include <math.h>

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

}

#endif // ROBOT_H
