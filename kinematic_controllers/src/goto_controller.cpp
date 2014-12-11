#include <kinematic_controllers/goto_controller.h>
#include <std_msgs/Bool.h>
#include <common/robot.h>
#include <common/util.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <pid.h>
#include <navigation_msgs/NextNodeOfInterest.h>

#define DEG2RAD(x) ((x)*M_PI/180.0)
#define RAD2DEG(x) ((x)*180.0/M_PI)


GotoController::GotoController(ros::NodeHandle &handle, double update_frequency)
    :ControllerBase(handle, update_frequency)
    ,_kp("/controller/goto/foward_kp", -0.6)
    ,_min_dist_to_succeed("/controller/goto/min_dist_to_succeed", robot::dim::wheel_distance/2.0)
    ,_velocity("/controller/forward/velocity",0.2)
    ,_phase(IDLE)
    ,_twist(new geometry_msgs::Twist)
    ,_dist_to_target(0)
    ,_dist_convergence(0.95)
{
    _sub_node = _handle.subscribe("/controller/goto/target_node", 10, &GotoController::callback_target_node, this);
    _sub_path = _handle.subscribe("/controller/goto/follow_path", 10, &GotoController::callback_path, this);
    _sub_straight = _handle.subscribe("/controller/goto/straight", 10, &GotoController::callback_straight_distance, this);
    _sub_shake = _handle.subscribe("/controller/goto/shake", 10, &GotoController::callback_shake, this);


    _sub_odom = _handle.subscribe("/pose/odometry/", 10, &GotoController::callback_odometry, this);
    _sub_turn_done = _handle.subscribe("/controller/turn/done", 10, &GotoController::callback_turn_done, this);
    _sub_ir = _handle.subscribe("/perception/ir/distance", 0.25, &GotoController::callback_ir, this);

    _pub_turn_angle = _handle.advertise<std_msgs::Float64>("/controller/turn/angle",10);
    _pub_activate_wall_follow = _handle.advertise<std_msgs::Bool>("/controller/wall_follow/active",10);
    _pub_success = _handle.advertise<std_msgs::Bool>("/controller/goto/success", 10);

    _srv_raycast = _handle.serviceClient<navigation_msgs::Raycast>("/mapping/raycast");

    reset();
}

GotoController::~GotoController()
{}

bool GotoController::is_active()
{
    return _phase != IDLE;
}

int GotoController::greedy_removal(const std::vector<navigation_msgs::Node>& nodes, int start)
{
    int end = start;
    double x0 = nodes[start].x;
    double y0 = nodes[start].y;

    for (int i = start+1; i < nodes.size(); ++i)
    {
        double x1 = nodes[i].x;
        double y1 = nodes[i].y;

        if(!straight_obstacle_free(x0,y0, x1,y1))
            return end;

        end++;
    }
}

void GotoController::simplify_path(const navigation_msgs::PathConstPtr &path, navigation_msgs::Path &result_path)
{
    const std::vector<navigation_msgs::Node>& nodes = path->path;
    std::vector<navigation_msgs::Node>& result = result_path.path;

    result.clear();
    result.push_back(nodes[0]);

    for (int i = 0; i < nodes.size()-1  ;)
    {
        int next = greedy_removal(nodes, i);

        if (i == next && i < nodes.size()) {
            ROS_WARN("Unexpected case: Cannot reach next node %d from %d. Will go to %d anyway", i+1,i, i+1);
            next = i+1;
        }

        result.push_back(nodes[next]);

        i = next;
    }
}


bool GotoController::straight_obstacle_free(double x0, double y0, double x1, double y1)
{
    Eigen::Vector2d dir(x1-x0, y1-y0);
    Eigen::Vector2d origin(x0,y0);

    double distance = (dir-origin).norm();
    dir.normalize();

    Eigen::Vector2d dir_ortho(-dir(1),dir(0));

    //do three raycasts. one in the center, and two at the outer position of the robot
    double dummy;
    if (!request_raycast(origin(0),origin(1),dir(0),dir(1),distance,dummy))
        return false;

    Eigen::Vector2d left_origin = origin + dir_ortho*robot::dim::robot_diameter/2.0;
    Eigen::Vector2d right_origin = origin - dir_ortho*robot::dim::robot_diameter/2.0;

    if (!request_raycast(left_origin(0),left_origin(1),dir(0),dir(1),distance,dummy))
        return false;
    if (!request_raycast(right_origin(0),right_origin(1),dir(0),dir(1),distance,dummy))
        return false;

    return true;
}

bool GotoController::request_raycast(double x, double y, double dir_x, double dir_y, double max_length, double &dist)
{
    navigation_msgs::RaycastRequest request;
    navigation_msgs::RaycastResponse response;

    request.frame_id = "map";
    request.origin_x = x;
    request.origin_y = y;
    request.dir_x = dir_x;
    request.dir_y = dir_y;
    request.max_length = max_length;

    response.hit = false;
    _srv_raycast.call(request, response);

    if (response.hit) {
        dist = response.hit_dist;
        return true;
    }

    return false;
}

double euclidean_distance(double x0, double y0, double x1, double y1)
{
    return sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
}

void GotoController::callback_target_node(const navigation_msgs::NodeConstPtr& node) {

    reset();

    _path.path.push_back(*node);

    _phase = FIRST_TURN;
    _next_node = 0;
}

void GotoController::callback_path(const navigation_msgs::PathConstPtr &path)
{
    reset();

    if (path->path.empty()) {
        _phase = TARGET_REACHED;
        return;
    }

    simplify_path(path, _path);

    std::cout << "Simplified path: ";
    for(int i = 0; i < _path.path.size(); ++i)
    {
        std::cout << _path.path[i].id_this << " ";
    }
    std::cout << std::endl;

    _phase = FIRST_TURN;
    _next_node = 0;
}

void GotoController::callback_straight_distance(const std_msgs::Float64ConstPtr& dist)
{
    double cur_x = _odom_x;
    double cur_y = _odom_y;
    double cur_theta = _odom_theta;

    double dir_x = cos(cur_theta);
    double dir_y = sin(cur_theta);

    double target_x = cur_x + dir_x*dist->data;
    double target_y = cur_y + dir_y*dist->data;

    navigation_msgs::Node target;
    target.x = target_x;
    target.y = target_y;

    _path.path.push_back(target);

    _phase = MOVE_STRAIGHT;
    _angle_to_target = 0;
    _next_node = 0;

    _straight_direction = dist->data >= 0 ? 1.0 : -1.0;
}

void GotoController::callback_shake(const std_msgs::Float64ConstPtr& time)
{
 double shake_time=time->data;
 _shake_times=shake_time/_update_frequency;

}


void GotoController::callback_odometry(const nav_msgs::OdometryConstPtr &odometry)
{
    _odom_x = odometry->pose.pose.position.x;
    _odom_y = odometry->pose.pose.position.y;

    tf::Quaternion q(odometry->pose.pose.orientation.x, odometry->pose.pose.orientation.y, odometry->pose.pose.orientation.z, odometry->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double dummy;
    m.getRPY(dummy,dummy, _odom_theta);
}

void GotoController::update_distance_to_target()
{
    _last_dist_to_target = _dist_to_target;
    navigation_msgs::Node& next_node = _path.path[_next_node];
    _dist_to_target = euclidean_distance(next_node.x, next_node.y, _odom_x, _odom_y);
}

void GotoController::callback_turn_done(const std_msgs::BoolConstPtr &done)
{
    if (_phase == IDLE) return;
    _turn_done = done->data;
}

void GotoController::callback_ir(const ir_converter::DistanceConstPtr &distances)
{
    if (_phase == IDLE) return;

    if (distances->l_front < 0.25 || distances->r_front < 0.25) {
        _obstacle_ahead = true;
    }
    else
        _obstacle_ahead = false;
}

void GotoController::hard_reset()
{
    if (_phase == IDLE) return;

    _phase = HARD_STOP;
}

void GotoController::reset() {
    _obstacle_ahead = false;
    _break = false;

    _fwd_vel = 0;

    _path.path.clear();
    _next_node = 0;
    _dist_to_target = std::numeric_limits<double>::infinity();
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
}

double GotoController::angle_between(Eigen::Vector2d& from, Eigen::Vector2d& to)
{
    double dot = from.dot(to);
    if (dot < -1.0) dot = -1.0;
    else if (dot > 1.0) dot = 1.0;
    double angle = acos(dot);

    double crossZ = from(0) * to(1) - from(1) * to(0);

    if (crossZ < 0) angle *= -1.0;
    return angle;
}

void GotoController::turn(double angle_rad) {
    std_msgs::Float64 msg;
    msg.data = RAD2DEG(angle_rad);
    ROS_ERROR("Angle to turn: %.3lf",msg.data);
    _pub_turn_angle.publish(msg);
}

void GotoController::execute_first_phase()
{
    if (_wait_for_turn_done) {
        if (_turn_done) {
            _phase++;
            _wait_for_turn_done = false;
            _turn_done = false;

            ROS_INFO("Commencing phase %d",_phase);
        }
    }
    else {
        navigation_msgs::Node& next_node = _path.path[_next_node];

        Eigen::Vector2d forward(cos(_odom_theta), sin(_odom_theta));
        Eigen::Vector2d to_obj(next_node.x - _odom_x, next_node.y - _odom_y);
        to_obj.normalize();

        _angle_to_target = angle_between(forward, to_obj);

        _wait_for_turn_done = true;
        _turn_done = false;

        if (std::abs(RAD2DEG(_angle_to_target)) > 2.0 )
            turn(_angle_to_target);
        else
            _turn_done = true;

        if (_angle_to_target > M_PI_4)
        {
            if (_angle_to_target < M_PI_2)
                _angle_to_target -= M_PI_2;
            else
                _angle_to_target -= M_PI;
        }
        else if (_angle_to_target < -M_PI_4)
        {
            if (_angle_to_target > -M_PI_2)
                _angle_to_target += M_PI_2;
            else
                _angle_to_target += M_PI;
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

//    ROS_INFO("Current vel: %.3lf, Actual distance: %.3lf, Last distance: %.3lf, Dist diff %.3lf", _fwd_vel, _dist_to_target, _last_dist_to_target, dist_diff);

    if (dist_diff < 0.0005)
    {
        _phase++;
        _fwd_vel = 0;

        std_msgs::Bool msg;
        msg.data = false;
        _pub_activate_wall_follow.publish(msg);
        _wall_following_active = false;

       // if (_break)
       //    _phase = TARGET_UNREACHABLE;

        _break = false;

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

        if (_obstacle_ahead) _break = true;

        if (!_break)
            _fwd_vel = pd::P_control(_kp(), _dist_to_target , 0);
        else {
//            ROS_ERROR("Emergency break due to obstacle");
            double kp_break;
            ros::param::getCached("/controller/forward/kp/break",kp_break);
            _fwd_vel += pd::P_control(kp_break, _fwd_vel, 0);
        }
    }
}

void GotoController::execute_fourth_phase()
{
    if (_wait_for_turn_done) {
        if (_turn_done) {
            _wait_for_turn_done = false;
            _turn_done = false;


            if (_dist_to_target < _min_dist_to_succeed())
                _phase++;
            else {
                ROS_ERROR("Failed fourth phase");
                _phase = TARGET_UNREACHABLE;
            }

            ROS_INFO("Commencing phase %d",_phase);
        }
    }
    else {
        _wait_for_turn_done = true;
        _turn_done = false;

        if (std::abs(RAD2DEG(_angle_to_target)) > 2.0)
            turn(-_angle_to_target);
        else
        {
            _turn_done = true;
        }
    }
}



void GotoController::execute_move_straight()
{
    double dist_diff = _dist_convergence.filter(std::abs(_last_dist_to_target - _dist_to_target));

    if (dist_diff < 0.0005)
    {
        _fwd_vel = 0;
        _phase = TARGET_REACHED;
    }
    else
    {
        _fwd_vel = pd::P_control(_kp(), _straight_direction*_dist_to_target , 0);
    }
}

void GotoController::execute_shake()
{

    if (_shake_flag){
    _twist->linear.x=0.1;
    _shake_flag=0;
    }
    else
    {
     _twist->linear.x=-0.1;
     _shake_flag=1;
    }
    _phase=SHAKE;
}



geometry_msgs::TwistConstPtr GotoController::update()
{
    if (_phase > IDLE) {

        update_distance_to_target();

        if (std::isnan(_dist_to_target)) {
            ROS_ERROR("Distance is infinity. Waiting for odometry callback.");
            return _twist;
        }

        if (_dist_to_target < _min_dist_to_succeed()) {
            //ROS_ERROR("Node already reached");
            _phase = TARGET_REACHED;
        }

        switch(_phase) {
        case FIRST_TURN:
        {
            execute_first_phase();
            break;
        }
        case SECOND_THETA_CORRECTION:
        {
            execute_second_phase();

            if (_phase == THIRD_FWD)
                _dist_convergence.init_to(_dist_to_target);

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
            _next_node++;
            if (_next_node < _path.path.size()) {
                ROS_INFO("Node %d reached. Continue to node %d", _path.path[_next_node-1].id_this, _path.path[_next_node].id_this);
                _phase = FIRST_TURN;
            }
            else {
                ROS_INFO("Target reached");
                //send msg
                std_msgs::Bool msg;
                msg.data = true;
                _pub_success.publish(msg);
                reset();
            }
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
        case HARD_STOP:
        {
            ROS_ERROR("[GotoController::update] Hard stop due to crash.");
            std_msgs::Bool msg;
            msg.data = false;
            _pub_success.publish(msg);
            reset();
            break;
        }
        case MOVE_STRAIGHT:
        {
            execute_move_straight();
            break;
        }
        case SHAKE:
        {
            if (_shake_times!=0)
            {
            execute_shake();
            _shake_times=_shake_times-1;
            }
            break;
        }
        default:
        {
            reset();
            break;
        }
        }

    }

    _twist->linear.x = common::Clamp<double>(_fwd_vel,-_velocity(), _velocity());
    return _twist;
}

