#ifndef TURN_CONTROLLER_THETA_H
#define TURN_CONTROLLER_THETA_H

#include <kinematic_controllers/controller_base.h>
#include <common/parameter.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>

class TurnControllerTheta : public ControllerBase
{
public:
    TurnControllerTheta(ros::NodeHandle& handle, double update_frequency);

    virtual ~TurnControllerTheta();

    virtual geometry_msgs::TwistConstPtr update();

private:

    //------------------------------------------------------------------------------
    // Method declarations
    void callback_turn_angle(const std_msgs::Float64ConstPtr& deg);
    void callback_odom(const nav_msgs::OdometryConstPtr& odom);

    void send_done_message(bool flag);
    double control_angular_velocity();

    //------------------------------------------------------------------------------
    // Member
    double _angle_to_rotate;
    double _theta_last;
    double _theta;
    double _theta_target;

    double _w, _w_last;
    geometry_msgs::TwistPtr _twist;

    //------------------------------------------------------------------------------
    // Parameter
    Parameter<double> _kp;
    Parameter<double> _kd;
    Parameter<double> _convergence_threshold_w;
    Parameter<double> _convergence_threshold_theta;
    Parameter<double> _limit_w;

    //------------------------------------------------------------------------------
    // Subscribers and publisher
    ros::Subscriber _sub_angle;
    ros::Subscriber _sub_odom;

    ros::Publisher _pub_done;
};

#endif // TURN_CONTROLLER_THETA_H
