#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class ControllerBase
{
public:
    ControllerBase(ros::NodeHandle& handle, double update_frequency);
    virtual ~ControllerBase();

    virtual geometry_msgs::TwistConstPtr update();

protected:
    ros::NodeHandle _handle;
    double _update_frequency;
};

#endif
