#include <kinematic_controllers/goto_controller.h>
#include <std_msgs/Bool.h>
#include <common/robot.h>
#include <common/util.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <pid.h>

#define DEG2RAD(x) ((x)*M_PI/180.0)
#define RAD2DEG(x) ((x)*180.0/M_PI)


GotoController::GotoController(ros::NodeHandle &handle, double update_frequency)
    :ControllerBase(handle, update_frequency)
    ,_kp("/controller/goto/foward_kp", 0.01)
    ,_min_dist_to_succeed("/controller/goto/min_dist_to_succeed", robot::dim::wheel_distance/2.0)
    ,_velocity("/controller/forward/velocity",0.2)
    ,_phase(IDLE)
    ,_twist(new geometry_msgs::Twist)
    ,_dist_to_target(0)
    ,_dist_convergence(0.75)
{
    _sub_node = _handle.subscribe("/controller/goto/target_node",  10, &GotoController::callback_target_node, this);
    _sub_odom = _handle.subscribe("/pose/odometry/", 10, &GotoController::callback_odometry, this);
    _sub_turn_done = _handle.subscribe("/controller/turn/done", 10, &GotoController::callback_turn_done, this);
    _sub_ir = _handle.subscribe("/perception/ir/distance", 0.25, &GotoController::callback_ir, this);

    _pub_turn_angle = _handle.advertise<std_msgs::Float64>("/controller/turn/angle",10);
    _pub_activate_wall_follow = _handle.advertise<std_msgs::Bool>("/controller/wall_follow/active",10);
    _pub_success = _handle.advertise<std_msgs::Bool>("/controller/goto", 10);

    reset();
}

GotoController::~GotoController()
{}

void GotoController::callback_target_node(const navigation_msgs::NodeConstPtr& node) {
    _target_node = *node;
    reset();
}

double euclidean_distance(double x0, double y0, double x1, double y1)
{
    return sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
}

void GotoController::callback_odometry(const nav_msgs::OdometryConstPtr &odometry)
{
    if (_phase != THIRD_FWD) return;

    _odom_x = odometry->pose.pose.position.x;
    _odom_y = odometry->pose.pose.position.y;

    tf::Quaternion q(odometry->pose.pose.orientation.x, odometry->pose.pose.orientation.y, odometry->pose.pose.orientation.z, odometry->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double dummy;
    m.getRPY(dummy,dummy, _odom_theta);

    _last_dist_to_target = _dist_to_target;
    _dist_to_target = euclidean_distance(_target_node.x, _target_node.y, _odom_x, _odom_y);
}

void GotoController::callback_turn_done(const std_msgs::BoolConstPtr &done)
{
    _turn_done = done->data;
}

void GotoController::callback_ir(const ir_converter::DistanceConstPtr &distances)
{
    if (distances->l_front < 0.25 || distances->r_front < 0.25) {
        _break_due_to_obstacle = true;
        ROS_ERROR("Emergency break due to obstacle");
    }
}

void GotoController::reset() {
    _break_due_to_obstacle = false;

    _fwd_vel = 0;

    _dist_to_target = 0;
    _last_dist_to_target = 0;

    _wait_for_turn_done = false;
    _turn_done = false;
    _phase = IDLE;

    _wall_following_active = false;

    std_msgs::Bool msg;
    msg.data = false;
    _pub_activate_wall_follow.publish(msg);

    _twist->angular.z = 0;
    _twist->linear.x = 0;

    _dist_convergence.init_to(1.0);
}

double GotoController::angle_between(Eigen::Vector2d& from, Eigen::Vector2d& to)
{
    double angle = acos(from.dot(to));
    if (angle < -1.0) angle = -1.0;
    else if (angle > 1.0) angle = 1.0;

    double crossZ = from(0) * to(1) - from(1) * to(0);

    if (crossZ < 0) angle *= -1.0;
    return angle;
}

void GotoController::turn(double angle_rad) {
    std_msgs::Float64 msg;
    msg.data = RAD2DEG(angle_rad);
    _pub_turn_angle.publish(msg);
}

void GotoController::execute_first_phase()
{
    if (_wait_for_turn_done && _turn_done) {
        _phase++;
        _wait_for_turn_done = false;
        _turn_done = false;

        ROS_INFO("Commencing phase %d",_phase);
    }
    else {
        Eigen::Vector2d forward(cos(_odom_theta), sin(_odom_theta));
        Eigen::Vector2d to_obj(_target_node.x - _odom_x, _target_node.y - _odom_y);
        to_obj.normalize();

        _angle_to_obj = angle_between(forward, to_obj);

        ROS_INFO("Angle to turn: %.3lf",_angle_to_obj);

        _wait_for_turn_done = true;
        _turn_done = false;

        turn(_angle_to_obj);

        if (_angle_to_obj <= M_PI_4 && _angle_to_obj >= -M_PI_4) {
            _phase++;
        }
        else {

            if (_angle_to_obj > M_PI_4)
                _angle_to_obj -= 90.0;
            else
                _angle_to_obj += 90.0;
        }
    }
}

void GotoController::execute_second_phase()
{
    _phase++;
    ROS_INFO("Commencing phase %d",_phase);

//    if (_wait_for_turn_done && _turn_done) {
//        _phase++;
//        _wait_for_turn_done = false;
//        _turn_done = false;
//    }
//    else {
//        _wait_for_turn_done = true;
//        _turn_done = false;

//        turn(_angle_to_obj);
//    }
}

void GotoController::execute_third_phase()
{
    double dist_diff = _dist_convergence.filter(std::abs(_last_dist_to_target - _dist_to_target));

    if (dist_diff < 0.01)
    {
        _phase++;
        _fwd_vel = 0;

        std_msgs::Bool msg;
        msg.data = false;
        _pub_activate_wall_follow.publish(msg);
        _wall_following_active = false;

        if (_break_due_to_obstacle)
            _phase = TARGET_UNREACHABLE;

        _break_due_to_obstacle = false;

        ROS_INFO("Commencing phase %d",_phase);
    }
    else
    {
        if (!_wall_following_active)
        {
            std_msgs::Bool msg;
            msg.data = true;
            _pub_activate_wall_follow.publish(msg);
            _wall_following_active = true;
        }

        if (!_break_due_to_obstacle)
            _fwd_vel += pd::P_control(_kp(), _dist_to_target , 0);
        else {
            double kp_break;
            ros::param::getCached("/controller/forward/kp/break",kp_break);
            _fwd_vel += pd::P_control(kp_break, _fwd_vel, 0);
        }
    }
}

void GotoController::execute_fourth_phase()
{
    if (_wait_for_turn_done && _turn_done) {
        _wait_for_turn_done = false;
        _turn_done = false;


        if (_dist_to_target < _min_dist_to_succeed())
        {
            _phase++;
        }
        else
        {
            _phase = TARGET_UNREACHABLE;
        }

        ROS_INFO("Commencing phase %d",_phase);
    }
    else {
        _wait_for_turn_done = true;
        _turn_done = false;

        ROS_INFO("Angle to turn: %.3lf",-_angle_to_obj);

        turn(-_angle_to_obj);
    }
}

geometry_msgs::TwistConstPtr GotoController::update()
{
    if (_phase > IDLE) {


        switch(_phase) {
        case FIRST_TURN:
        {
            execute_first_phase();
        }
        case SECOND_THETA_CORRECTION:
        {
            execute_second_phase();
            break;
        }
        case THIRD_FWD:
        {
            execute_third_phase();
            break;
        }
        case FOURTH_THETA_CORRECTION:
        {
            execute_fourth_phase();
            break;
        }
        case TARGET_REACHED:
        {
            ROS_INFO("Target reached");
            //send msg
            std_msgs::Bool msg;
            msg.data = true;
            _pub_success.publish(msg);
            reset();
            break;
        }
        case TARGET_UNREACHABLE:
        {
            ROS_ERROR("Target unreachable. Best distance reachable: %.3lf",_dist_to_target);
            std_msgs::Bool msg;
            msg.data = false;
            _pub_success.publish(msg);
            reset();
            break;
        }
        default:
        {
            reset();
            break;
        }
        }

    }
    else {
    }

    _twist->linear.x = common::Clamp<double>(_fwd_vel,-_velocity(), _velocity());
    return _twist;
}
