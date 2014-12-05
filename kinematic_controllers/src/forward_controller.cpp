#include <kinematic_controllers/forward_controller.h>
#include <std_msgs/Bool.h>
#include <../etc/pid.h>
#include <tf/transform_listener.h>
#include <common/robot.h>

ForwardController::ForwardController(ros::NodeHandle &handle, double update_frequency)
    :ControllerBase(handle, update_frequency)
    ,_kp_a("/controller/forward/kp/accel", 0.05)
    ,_kp_b("/controller/forward/kp/break", 0.15)
    ,_kp_b_wall("/controller/forward/kp/wall_break", 0.05)
    ,_stop_thresh("/controller/forward/stop_thresh", 0.01)
    ,_velocity("/controller/forward/velocity",0.2)
    ,_wall_time_thresh("/controller/forward/wall_time_thresh",2.0)
    ,_wall_dist_thresh("/controller/forward/wall_dist_thresh",0.40)
    ,_wall_target_dist("/controller/forward/wall_target_dist",0.05)
    ,_active(false)
    ,_twist(new geometry_msgs::Twist)
    ,_send_msg_flag(false)
    ,_time_since_last_plane(0)
    ,_continue_to_wall(false)
    ,_markers("map","dist_to_front_wall")
    ,_label_marker(-1)
    ,_ray_marker(-1)
{
    _sub_act = _handle.subscribe("/controller/forward/active",   10, &ForwardController::callback_activate, this);
    _sub_planes = _handle.subscribe("/vision/obstacles/planes",  10, &ForwardController::callback_planes, this);
    _sub_odom = _handle.subscribe("/pose/odometry/", 10, &ForwardController::callback_odometry, this);
    _pub_stop = _handle.advertise<std_msgs::Bool>("/controller/forward/stopped",10);
    _pub_viz = _handle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
}

ForwardController::~ForwardController()
{}

void ForwardController::callback_activate(const std_msgs::BoolConstPtr& val) {
    _active = val->data;
    if (_active==true) _send_msg_flag = true;
    else {
        _continue_to_wall = false;
        if ((ros::Time::now().toSec() - _time_since_last_plane.toSec()) < _wall_time_thresh())
        {
            if (_dist_to_wall < _wall_dist_thresh())
                _continue_to_wall = true;
        }
    }
}

void ForwardController::callback_planes(const vision_msgs::PlanesConstPtr &planes)
{
    static tf::TransformListener tf_listener;

    double max_dot = 0;
    int ortho_plane = 0;

    for(int i = 0; i < planes->planes.size(); ++i)
    {
        const vision_msgs::Plane& plane = planes->planes[i];
        if (plane.is_ground_plane) continue;

        Eigen::Vector3f plane_normal(plane.plane_coefficients[0],
                                     plane.plane_coefficients[1],
                                     plane.plane_coefficients[2]);
        Eigen::Vector3f forward(1,0,0);

        double dotprod = std::abs(forward.dot(plane_normal));
        if (dotprod > max_dot) {
            max_dot = dotprod;
            ortho_plane = i;
        }
    }

    if (max_dot > 0.9) {
        const vision_msgs::Plane& front_plane = planes->planes[ortho_plane];

        try {
            geometry_msgs::PointStamped p_in, p_out;
            p_in.header.frame_id = "robot";
            p_in.point.x = front_plane.bounding_box[0];
            p_in.point.y = 0;//front_plane.plane_coefficients[1];
            p_in.point.z = 0;

            tf_listener.transformPoint("map",p_in, p_out);
            _front_wall_x = p_out.point.x;
            _front_wall_y = p_out.point.y;

            _time_since_last_plane = ros::Time::now();

            //ROS_ERROR("Plane in front at: %.3lf, %.3lf, x= %.3lf", _front_wall_x, _front_wall_y, p_in.point.x);
        } catch(...) {
        }
    }
}

double euclidean_distance(double x0, double y0, double x1, double y1)
{
    return sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
}

void ForwardController::callback_odometry(const nav_msgs::OdometryConstPtr &odometry)
{
    _dist_to_wall = euclidean_distance(_front_wall_x, _front_wall_y, odometry->pose.pose.position.x, odometry->pose.pose.position.y);
    //ROS_ERROR("Distance between: (%.3lf,%.3lf), (%.3lf,%.3lf) = %.3lf",_front_wall_x, _front_wall_y, odometry->pose.pose.position.x, odometry->pose.pose.position.y,_dist_to_wall);

//    _ray_marker = _markers.add_line(_front_wall_x, _front_wall_y, odometry->pose.pose.position.x, odometry->pose.pose.position.y, 0.1, 0.01, 0, 0, 255, _ray_marker);

//    std::string label = static_cast<std::ostringstream*>( &(std::ostringstream() << _dist_to_wall) )->str();
//    float x = (_front_wall_x + odometry->pose.pose.position.x)/2.0f;
//    float y = (_front_wall_y + odometry->pose.pose.position.y)/2.0f;
//    _label_marker = _markers.add_text(x, y, 0.15, label, 0, 0, 255, _label_marker);

//    _pub_viz.publish(_markers.get());
}

geometry_msgs::TwistConstPtr ForwardController::update()
{
    if (_active)
        _twist->linear.x += pd::P_control(_kp_a(), _twist->linear.x, _velocity());
    else {

        if (_continue_to_wall) {

            double target_dist = robot::dim::robot_diameter/2.0 + _wall_target_dist();
            _twist->linear.x += pd::P_control(_kp_b_wall(), _dist_to_wall, target_dist);

            //ROS_ERROR("Distance to wall: %.3lf",_dist_to_wall);

            if (std::abs(_dist_to_wall - target_dist) < _stop_thresh() && _send_msg_flag == true) {
                _twist->linear.x = 0;
                //send msg
                std_msgs::Bool msg;
                msg.data = true;
                _pub_stop.publish(msg);
                _send_msg_flag = false;

                _continue_to_wall = false;
            }

        }
        else {

            _twist->linear.x += pd::P_control(_kp_b(), _twist->linear.x, 0);

            if (std::abs(_twist->linear.x) < _stop_thresh() && _send_msg_flag == true) {
                _twist->linear.x = 0;
                //send msg
                std_msgs::Bool msg;
                msg.data = true;
                _pub_stop.publish(msg);
                _send_msg_flag = false;
            }

        }
    }

    return _twist;
}
