#include "kinematic_controllers/turn_controller.h"
#include <common/robot.h>
#include <pid.h>

#define SIGN(x) ( (x) < 0 ? -1.0 : ((x) > 0 ? 1.0 : 0.0) )


//------------------------------------------------------------------------------
// Callbacks

void TurnController::callback_turn_angle(const std_msgs::Float64ConstPtr& deg)
{
    if (isnan(deg->data)) {
        ROS_ERROR("Received nan value.");
        return;
    }

    _angle_to_rotate = -deg->data;

    Vector2i delta_ticks = robot::Delta_ticks_for_rotation(_angle_to_rotate);
    _target = _encoders + delta_ticks;
}

void TurnController::callback_encoders(const ras_arduino_msgs::EncodersConstPtr& encoders)
{
    _encoders_last = _encoders;

    _encoders(0) = encoders->encoder1;
    _encoders(1) = encoders->encoder2;
}

//------------------------------------------------------------------------------
// Methods

TurnController::TurnController(ros::NodeHandle &handle, double update_frequency)
    :ControllerBase(handle, update_frequency)
    ,_kp("/controller/turn/kp", 0.0001)
    ,_kd("/controller/turn/kd", 0.0005)
    ,_convergence_threshold_w("/controller/turn/conv_thresh", 0.001)
    ,_encoder_threshold("/controller/turn/encoder_thresh", 5)
    ,_initial_w("/controller/turn/initial_w", 0.5)
    ,_limit_w("/controller/turn/limit_w", 0.5)
    ,_twist(new geometry_msgs::Twist)
{
    _angle_to_rotate = 0;

    _sub_angle = _handle.subscribe("/controller/turn/angle", 10, &TurnController::callback_turn_angle, this);
    _sub_enc = _handle.subscribe("/arduino/encoders", 10, &TurnController::callback_encoders, this);

    _pub_done = _handle.advertise<std_msgs::Bool>("/controller/turn/done", 10);
}

TurnController::~TurnController() {
}

void TurnController::send_done_message(bool flag) {
    std_msgs::Bool done;
    done.data = flag;
    _pub_done.publish(done);
}

double TurnController::control_angular_velocity()
{
    int32_t state = _target(0) - _encoders(0);
    int32_t state_last = _target(0) - _encoders_last(0);
    double w = -pd::PD_control(_kp(),_kd(),(double)state,0.0,(double)state_last,0,1.0/_update_frequency);
    state_last = state;

    return w;
}

geometry_msgs::TwistConstPtr TurnController::update()
{
    if (_angle_to_rotate != 0)
    {
        _w += control_angular_velocity();
        double sup = _limit_w();
        double inf = -sup;
        _w = std::min(sup, std::max(inf, _w)); //restrict to maximum velocity

        int encoderDifference = _target(0) - _encoders(0);
        //double acceleration = (w-w_last)/dt;

        ROS_INFO("Encoder differences: (%d, %d)\n", encoderDifference, _target(1)-_encoders(1));
        ROS_INFO("Velocity: %lf\n\n", _w);

        //stop rotating when the angular velocity is stabelized
        if (std::abs(encoderDifference) < _encoder_threshold() &&
            std::abs(_w) < 0.3
            //std::abs(acceleration) < _convergence_threshold_w
                )
        {
            _angle_to_rotate = 0;
            _w = _w_last = 0;

            send_done_message(true);

            ROS_INFO("Target rotation reached.\n\n");
        }
        else {
            _w_last = _w;
        }

        _twist->angular.z = _w+SIGN(_w)*_initial_w();
    }
    else
    {
        _twist->angular.z = 0;
    }

    return _twist;
}
