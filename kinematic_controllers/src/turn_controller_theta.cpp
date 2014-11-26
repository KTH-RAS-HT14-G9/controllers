#include "kinematic_controllers/turn_controller_theta.h"
#include <common/robot.h>
#include <common/util.h>
#include <pid.h>
#include <tf/tf.h>

#define SIGN(x) ( (x) < 0 ? -1.0 : ((x) > 0 ? 1.0 : 0.0) )

double wrap_radians(double r)
{
	r += (r > M_PI) ? -M_PI*2.0 : (r < -M_PI) ? M_PI*2.0 : 0;
	return r;
}

//------------------------------------------------------------------------------
// Callbacks

void TurnControllerTheta::callback_turn_angle(const std_msgs::Float64ConstPtr& deg)
{
    if (isnan(deg->data)) {
        ROS_ERROR("Received nan value.");
        return;
    }

    _angle_to_rotate = deg->data;

	for(;_angle_to_rotate >  180.0; _angle_to_rotate -= 360.0);
    for(;_angle_to_rotate < -180.0; _angle_to_rotate += 360.0);

    double rad_to_rotate = _angle_to_rotate * M_PI / 180.0;

    _theta_target = wrap_radians(_theta + rad_to_rotate);
}

void TurnControllerTheta::callback_odom(const nav_msgs::OdometryConstPtr& odom)
{
//    tf::Quaternion q(odom->pose.pose.orientation.x,
//                     odom->pose.pose.orientation.y,
//                     odom->pose.pose.orientation.z,
//                     odom->pose.pose.orientation.w);

    _theta_last = _theta;
    _theta = tf::getYaw(odom->pose.pose.orientation);
}

//------------------------------------------------------------------------------
// Methods

TurnControllerTheta::TurnControllerTheta(ros::NodeHandle &handle, double update_frequency)
    :ControllerBase(handle, update_frequency)
    ,_kp("/controller/turn/kp", 0.5)
    ,_kd("/controller/turn/kd", 0.0)
    ,_convergence_threshold_w("/controller/turn/conv_thresh_w", 0.003)
    ,_convergence_threshold_theta("/controller/turn/conv_thresh_theta", 1.0*M_PI/180.0)
    ,_limit_w("/controller/turn/limit_w", 0.6)
    ,_twist(new geometry_msgs::Twist)
    ,_theta(M_PI/2.0)
{
    _angle_to_rotate = 0;

    _sub_angle = _handle.subscribe("/controller/turn/angle", 10, &TurnControllerTheta::callback_turn_angle, this);
    _sub_odom = _handle.subscribe("/pose/odometry", 10, &TurnControllerTheta::callback_odom, this);

    _pub_done = _handle.advertise<std_msgs::Bool>("/controller/turn/done", 10);
}

TurnControllerTheta::~TurnControllerTheta() {
}

void TurnControllerTheta::send_done_message(bool flag) {
    std_msgs::Bool done;
    done.data = flag;
    _pub_done.publish(done);
}

double TurnControllerTheta::control_angular_velocity()
{
	double state = wrap_radians(_theta_target - _theta);
    double state_last = wrap_radians(_theta_target - _theta_last);
    double w = -pd::PD_control(_kp(),_kd(),state,0.0,state_last,0,1.0/_update_frequency);

    return w;
}

geometry_msgs::TwistConstPtr TurnControllerTheta::update()
{
    if (_angle_to_rotate != 0)
    {
        _w = control_angular_velocity();

        double theta_diff = wrap_radians(_theta_target - _theta);

        ROS_INFO("Target: %.2lf, Theta: %.2lf, Diff: %lf", _theta_target, _theta, theta_diff);
        ROS_INFO("Velocity: \t %lf\n\n", _w);

        //stop rotating when the angular velocity is stabelized
        if (std::abs(theta_diff) < _convergence_threshold_theta() &&
            std::abs(_w) < _convergence_threshold_w() )
        {
            _angle_to_rotate = 0;
            _w = _w_last = 0;

            send_done_message(true);

            ROS_INFO("Target rotation reached.\n\n");
        }
        else {
            _w_last = _w;
        }

        double sup = _limit_w();
        double inf = -sup;
        double u = common::Clamp<double>(_w,inf,sup); //restrict to maximum velocity
        _twist->angular.z = u;
    }
    else
    {
        _twist->angular.z = 0;
    }

    return _twist;
}
