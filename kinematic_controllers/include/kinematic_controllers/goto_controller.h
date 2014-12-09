#ifndef GOTO_CONTROLLER_H
#define GOTO_CONTROLLER_H

#include <kinematic_controllers/controller_base.h>
#include <common/parameter.h>
#include <common/lowpass_filter.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <navigation_msgs/Node.h>
#include <navigation_msgs/Path.h>
#include <navigation_msgs/Raycast.h>
#include <Eigen/Core>
#include <ir_converter/Distance.h>


class GotoController : public ControllerBase
{
public:
    GotoController(ros::NodeHandle& handle, double update_frequency);

    ~GotoController();

    virtual geometry_msgs::TwistConstPtr update();

    virtual void hard_reset();

private:
    //------------------------------------------------------------------------------
    // Method declarations
    void callback_target_node(const navigation_msgs::NodeConstPtr& node);
    void callback_path(const navigation_msgs::PathConstPtr& path);

    void callback_odometry(const nav_msgs::OdometryConstPtr& odometry);
    void callback_turn_done(const std_msgs::BoolConstPtr& done);
    void callback_ir(const ir_converter::DistanceConstPtr& distances);

    void reset();

    void simplify_path(const navigation_msgs::PathConstPtr& path, navigation_msgs::Path& result);
    bool straight_obstacle_free(double x0, double y0, double x1, double y1);
    bool request_raycast(double x, double y, double dir_x, double dir_y, double max_length, double& dist);
    int greedy_removal(const std::vector<navigation_msgs::Node>& nodes, int start);

    double angle_between(Eigen::Vector2d& from, Eigen::Vector2d& to);
    void turn(double angle_rad);

    void execute_first_phase();
    void execute_second_phase();
    void execute_third_phase();
    void execute_fourth_phase();

    //------------------------------------------------------------------------------
    // Member
    geometry_msgs::TwistPtr _twist;
    navigation_msgs::Path _path;
    int _next_node;
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
    enum { HARD_STOP = 7};

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
    ros::Subscriber _sub_node, _sub_path;
    ros::Subscriber _sub_turn_done;
    ros::Subscriber _sub_odom;
    ros::Subscriber _sub_ir;

    ros::Publisher _pub_turn_angle;
    ros::Publisher _pub_activate_wall_follow;
    ros::Publisher _pub_success;

    //------------------------------------------------------------------------------
    // Service Clients
    ros::ServiceClient _srv_raycast;
};


#endif // GOTO_CONTROLLER_H

