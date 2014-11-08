#ifndef FORWARD_CONTROLLER_H
#define FORWARD_CONTROLLER_H

#include <kinematic_controllers/controller_base.h>
#include <common/parameter.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

class ForwardController : public ControllerBase
{
public:
    ForwardController(ros::NodeHandle& handle, double update_frequency);

    ~ForwardController();

    virtual geometry_msgs::TwistConstPtr update();

private:
    //------------------------------------------------------------------------------
    // Method declarations
    void callback_forward_velocity(const std_msgs::Float64ConstPtr& vel);
    void callback_activate(const std_msgs::BoolConstPtr& val);

    //------------------------------------------------------------------------------
    // Member
    double _velocity;
    bool _active;
    geometry_msgs::TwistPtr _twist;

    //------------------------------------------------------------------------------
    // Parameter
    Parameter<double> _kp;

    //------------------------------------------------------------------------------
    // Subscribers and publisher
    ros::Subscriber _sub_vel;
    ros::Subscriber _sub_act;
};


#endif // FORWARD_CONTROLLER_H
