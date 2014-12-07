#ifndef GOTO_CONTROLLER_H
#define GOTO_CONTROLLER_H

#include <kinematic_controllers/controller_base.h>
#include <common/parameter.h>
#include <common/lowpass_filter.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <navigation_msgs/Node.h>
#include <Eigen/Core>
#include <ir_converter/Distance.h>


class GotoController : public ControllerBase
{
public:
    GotoController(ros::NodeHandle& handle, double update_frequency);

    ~GotoController();

    virtual geometry_msgs::TwistConstPtr update();

private:
    //------------------------------------------------------------------------------
    // Method declarations
    void callback_target_node(const navigation_msgs::NodeConstPtr& node);
    void callback_odometry(const nav_msgs::OdometryConstPtr& odometry);
    void callback_turn_done(const std_msgs::BoolConstPtr& done);
    void callback_ir(const ir_converter::DistanceConstPtr& distances);

    void reset();

    double angle_between(Eigen::Vector2d& from, Eigen::Vector2d& to);
    void turn(double angle_rad);

    void execute_first_phase();
    void execute_second_phase();
    void execute_third_phase();
    void execute_fourth_phase();

    //------------------------------------------------------------------------------
    // Member
    geometry_msgs::TwistPtr _twist;
    navigation_msgs::Node _target_node;
    int _phase;
    double _dist_to_target;
    double _last_dist_to_target;
    double _fwd_vel;

    double _odom_x, _odom_y, _odom_theta;

    enum { IDLE = 0};
    enum { FIRST_TURN = 1};
    enum { SECOND_THETA_CORRECTION = 2};
    enum { THIRD_FWD = 3};
    enum { FOURTH_THETA_CORRECTION = 4};
    enum { TARGET_REACHED = 5};
    enum { TARGET_UNREACHABLE = 6};

    //turning phases
    bool _wait_for_turn_done;
    bool _turn_done;
    double _angle_to_obj;

    //fwd phase
    common::LowPassFilter _dist_convergence;
    bool _wall_following_active;

    //obstacle stuff
    bool _obstacle_ahead;
    bool _break;

    //------------------------------------------------------------------------------
    // Parameter
    Parameter<double> _kp;
    Parameter<double> _min_dist_to_succeed;
    Parameter<double> _velocity;

    //------------------------------------------------------------------------------
    // Subscribers and publisher
    ros::Subscriber _sub_node;
    ros::Subscriber _sub_turn_done;
    ros::Subscriber _sub_odom;
    ros::Subscriber _sub_ir;

    ros::Publisher _pub_turn_angle;
    ros::Publisher _pub_activate_wall_follow;
    ros::Publisher _pub_success;
};


#endif // GOTO_CONTROLLER_H

