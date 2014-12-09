#ifndef WALL_FOLLOWING_CONTROLLER_H
#define WALL_FOLLOWING_CONTROLLER_H

#include <kinematic_controllers/controller_base.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <geometry_msgs/Vector3.h>
#include <sstream>
#include <math.h>
#include <common/robot.h>
#include <common/parameter.h>
#include <std_msgs/Bool.h>
#include <ir_converter/Distance.h>

class WallFollowingController : public ControllerBase
{
public:
    WallFollowingController(ros::NodeHandle& handle, double update_frequency);

    ~WallFollowingController();

    virtual geometry_msgs::TwistConstPtr update();

    virtual void hard_reset();

private:
    //------------------------------------------------------------------------------
    // Method declarations
    void callback_sensors(const ir_converter::DistanceConstPtr& distances);
    void callback_activate(const std_msgs::BoolConstPtr& val);
    bool leftWallClose();
    bool rightWallClose();

    //------------------------------------------------------------------------------
    // Member
    double _fl_side,_fr_side,_bl_side,_br_side;
	bool _active;
    geometry_msgs::TwistPtr _twist;

    //------------------------------------------------------------------------------
    // Parameter
    Parameter<double> _kp_single;
    Parameter<double> _kp_double;
    Parameter<double> _wall_th;

    //------------------------------------------------------------------------------
    // Subscribers and publisher
    ros::Subscriber _sub_sens;
    ros::Subscriber _sub_act;
};


#endif // WALL_FOLLOWING_CONTROLLER_H
