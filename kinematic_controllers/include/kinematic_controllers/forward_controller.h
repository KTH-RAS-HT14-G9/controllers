#ifndef FORWARD_CONTROLLER_H
#define FORWARD_CONTROLLER_H

#include <kinematic_controllers/controller_base.h>
#include <common/parameter.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <vision_msgs/Planes.h>
#include <common/marker_delegate.h>


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
    void callback_planes(const vision_msgs::PlanesConstPtr& planes);
    void callback_odometry(const nav_msgs::OdometryConstPtr& odometry);

    //------------------------------------------------------------------------------
    // Member
    bool _active;
    bool _send_msg_flag;
    geometry_msgs::TwistPtr _twist;

    ros::Time _time_since_last_plane;
    double _front_wall_x, _front_wall_y;
    double _dist_to_wall;
    bool _continue_to_wall;

    common::MarkerDelegate _markers;
    int _ray_marker, _label_marker;

    //------------------------------------------------------------------------------
    // Parameter
    Parameter<double> _velocity;
    Parameter<double> _kp_a, _kp_b, _kp_b_wall;
    Parameter<double> _stop_thresh;
    Parameter<double> _wall_time_thresh;
    Parameter<double> _wall_dist_thresh;
    Parameter<double> _wall_target_dist;

    //------------------------------------------------------------------------------
    // Subscribers and publisher
    ros::Subscriber _sub_vel;
    ros::Subscriber _sub_act;
    ros::Subscriber _sub_planes;
    ros::Subscriber _sub_odom;
    ros::Publisher  _pub_stop;
    ros::Publisher  _pub_viz;
};


#endif // FORWARD_CONTROLLER_H
