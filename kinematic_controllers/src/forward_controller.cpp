#include <kinematic_controllers/forward_controller.h>
#include <std_msgs/Bool.h>
#include <../etc/pid.h>

ForwardController::ForwardController(ros::NodeHandle &handle, double update_frequency)
    :ControllerBase(handle, update_frequency)
    ,_kp_a("/controller/forward/kp/accel", 0.05)
    ,_kp_b("/controller/forward/kp/break", 0.15)
    ,_stop_thresh("/controller/forward/stop_thresh", 0.01)
    ,_active(false)
    ,_velocity(0.2)
    ,_twist(new geometry_msgs::Twist)
    ,_send_msg_flag(false)
{
    _sub_vel = _handle.subscribe("/controller/forward/velocity", 10, &ForwardController::callback_forward_velocity, this);
    _sub_act = _handle.subscribe("/controller/forward/active",   10, &ForwardController::callback_activate, this);
    _pub_stop = _handle.advertise<std_msgs::Bool>("/controller/forward/stopped",10);
}

ForwardController::~ForwardController()
{}

void ForwardController::callback_forward_velocity(const std_msgs::Float64ConstPtr& vel) {
    _velocity = vel->data;
}

void ForwardController::callback_activate(const std_msgs::BoolConstPtr& val) {
    _active = val->data;
    if (_active==true) _send_msg_flag = true;
}

geometry_msgs::TwistConstPtr ForwardController::update()
{
    if (_active)
        _twist->linear.x += pd::P_control(_kp_a(), _twist->linear.x, _velocity);
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

    return _twist;
}

/*
//------------------------------------------------------------------------------
// Constants

const double PUBLISH_FREQUENCY = 10.0;
const double DEFAULT_VELOCITY = 0.5;

//------------------------------------------------------------------------------
// Member

double _velocity = DEFAULT_VELOCITY;
bool _active;

Parameter<double> _kp("/controller/forward/kp", 0.15);

//------------------------------------------------------------------------------
// Callbacks

void callback_forward_velocity(const std_msgs::Float64ConstPtr& vel) {
    _velocity = vel->data;
}

void callback_activate(const std_msgs::BoolConstPtr& val) {
    _active = val->data;
}

//------------------------------------------------------------------------------
// Entry point

int main(int argc, char **argv)
{
    ros::init(argc, argv, "forward_controller");

    ros::NodeHandle n;
    ros::Subscriber sub_vel = n.subscribe("/controller/forward/velocity", 1, callback_forward_velocity);
    ros::Subscriber sub_act = n.subscribe("/controller/forward/active",   1, callback_activate);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/controller/forward/twist", 1);

    ros::Rate loop_rate(PUBLISH_FREQUENCY);

    geometry_msgs::Twist twist;

    while (n.ok()) {

        if (_active)
            twist.linear.x += pd::P_control(_kp(), twist.linear.x, _velocity);
        else
            twist.linear.x += pd::P_control(_kp(), twist.linear.x, 0);

        pub.publish(twist);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
*/
