#include "turn_controller.h"
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/Encoders.h>
#include <pid.h>
#include <robot.h>

#define SIGN(x) ((x) <= 0 ? -1.0 : (x) > 0 ? 1.0 : 0.0)

//------------------------------------------------------------------------------
// Constants

const double PUBLISH_FREQUENCY = 10.0;

//------------------------------------------------------------------------------
// Member variables

char _active_cmd = 0;
double _angle_to_rotate = 0;
Vector2i _encoders_last;
Vector2i _encoders;
Vector2i _target;

double _kp = 0.0008; std::string _kp_key = "/controller/turn/kp";
double _kd = 0.0015; std::string _kd_key = "/controller/turn/kd";
double _convergence_threshold_w = 0.001; std::string _convergence_threshold_w_key = "/controller/turn/conv_thresh";
int _encoder_threshold = 10; std::string _encoder_threshold_key = "/controller/turn/encoder_thresh";
double _initial_w = 1.5; std::string _initial_w_key = "/controller/turn/initial_w";

//------------------------------------------------------------------------------
// Callbacks

void callback_turn_angle(const std_msgs::Float64ConstPtr& deg)
{
    if (isnan(deg->data)) {
        ROS_ERROR("Received nan value.");
        return;
    }

    _angle_to_rotate = deg->data;

    Vector2i delta_ticks = robot::Delta_ticks_for_rotation(_angle_to_rotate);
    _target = _encoders + delta_ticks;
}

void callback_encoders(const ras_arduino_msgs::EncodersConstPtr& encoders)
{
    _encoders_last = _encoders;

    _encoders(0) = encoders->encoder1;
    _encoders(1) = encoders->encoder2;
}

//------------------------------------------------------------------------------
// Methods

void update_params() {
    ros::param::getCached(_convergence_threshold_w_key, _convergence_threshold_w);
    ros::param::getCached(_kp_key, _kp);
    ros::param::getCached(_kd_key, _kd);
    ros::param::getCached(_encoder_threshold_key, _encoder_threshold);
    ros::param::getCached(_initial_w_key, _initial_w);
}

void send_done_message(bool flag, ros::Publisher& publisher) {
    std_msgs::Bool done;
    done.data = flag;
    publisher.publish(done);
}

double control_angular_velocity()
{
    int32_t state = _target(0) - _encoders(0);
    int32_t state_last = _target(0) - _encoders_last(0);
    double w = -pd::PD_control(_kp,_kd,(double)state,0.0,(double)state_last,0,1.0/PUBLISH_FREQUENCY);
    w = w + SIGN(w)*_initial_w;
    state_last = state;

    return w;
}

//------------------------------------------------------------------------------
// Entry point

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turn_controller");

    ros::NodeHandle n;
    ros::Subscriber sub_angle = n.subscribe("/controller/turn/angle", 1, callback_turn_angle);
    ros::Subscriber sub_enc = n.subscribe("/arduino/encoders", 1, callback_encoders);
    ros::Publisher pub_twist = n.advertise<geometry_msgs::Twist>("/controller/turn/twist", 1);

    //ros::Subscriber sub_enc = n.subscribe("/kobuki/encoders", 1, callback_encoders);
    //ros::Publisher pub_twist = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
    ros::Publisher pub_done = n.advertise<std_msgs::Bool>("/controller/turn/done", 1);

    n.setParam(_kp_key, _kp);
    n.setParam(_kd_key, _kd);
    n.setParam(_convergence_threshold_w_key, _convergence_threshold_w);
    n.setParam(_encoder_threshold_key, _encoder_threshold);
    n.setParam(_initial_w_key, _initial_w);

    ros::Rate loop_rate(PUBLISH_FREQUENCY);

    geometry_msgs::Twist twist;

    double w_last = 0;
    double w = 0;
    double dt = 1.0/PUBLISH_FREQUENCY;

    while (n.ok()) {

        if (_angle_to_rotate != 0)
        {
            update_params();

            w += control_angular_velocity();
            //w = std::min(w, 1.0); //restrict to maximum velocity

            int encoderDifference = _target(0) - _encoders(0);
            //double acceleration = (w-w_last)/dt;

            ROS_INFO("Encoder differences: (%d, %d)\n", encoderDifference, _target(1)-_encoders(1));
            ROS_INFO("Velocity: %lf\n\n", w);

            //stop rotating when the angular velocity is stabelized
            if (std::abs(encoderDifference) < _encoder_threshold &&
                w < 0.05
                //std::abs(acceleration) < _convergence_threshold_w
                    )
            {
                _angle_to_rotate = 0;
                w = w_last = 0;

                send_done_message(true, pub_done);

                ROS_INFO("Target rotation reached.\n\n");
            }
            else {
                w_last = w;
            }

            twist.angular.z = w;
            pub_twist.publish(twist);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
