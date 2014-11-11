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

class WallFollowingController : public ControllerBase
{
public:
    WallFollowingController(ros::NodeHandle& handle, double update_frequency);

    ~WallFollowingController();

    virtual geometry_msgs::TwistConstPtr update();

private:
    //------------------------------------------------------------------------------
    // Method declarations
    void callback_sensors(const ras_arduino_msgs::ADConverter lecture);
    void callback_activate(const std_msgs::BoolConstPtr& val);
    bool bothWallsClose();
    bool leftWallClose();
    bool rightWallClose();
    //------------------------------------------------------------------------------
    // Member
    int _ch1,_ch2,_ch3,_ch4;
    double fl_side,fr_side,bl_side,br_side;
	bool _active;
    geometry_msgs::TwistPtr _twist;

    //------------------------------------------------------------------------------
    // Parameter
    Parameter<double> _kp;
    Parameter<double> wall_th;

    //------------------------------------------------------------------------------
    // Subscribers and publisher
    ros::Subscriber _sub_sens;
    ros::Subscriber _sub_act;
};


#endif // WALL_FOLLOWING_CONTROLLER_H
