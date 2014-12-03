#include <kinematic_controllers/forward_controller.h>
#include <std_msgs/Bool.h>
#include <../etc/pid.h>

ForwardController::ForwardController(ros::NodeHandle &handle, double update_frequency)
    :ControllerBase(handle, update_frequency)
    ,_kp_a("/controller/forward/kp/accel", 0.05)
    ,_kp_b("/controller/forward/kp/break", 0.15)
    ,_stop_thresh("/controller/forward/stop_thresh", 0.01)
    ,_velocity("/controller/forward/velocity",0.2)
    ,_active(false)
    ,_twist(new geometry_msgs::Twist)
    ,_send_msg_flag(false)
{
    _sub_act = _handle.subscribe("/controller/forward/active",   10, &ForwardController::callback_activate, this);
    _pub_stop = _handle.advertise<std_msgs::Bool>("/controller/forward/stopped",10);
}

ForwardController::~ForwardController()
{}

void ForwardController::callback_activate(const std_msgs::BoolConstPtr& val) {
    _active = val->data;
    if (_active==true) _send_msg_flag = true;
}

geometry_msgs::TwistConstPtr ForwardController::update()
{
    if (_active)
        _twist->linear.x += pd::P_control(_kp_a(), _twist->linear.x, _velocity());
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
