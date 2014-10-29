#include "turn_controller.h"
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/Encoders.h>
#include <pid.h>
#include <robot.h>

//------------------------------------------------------------------------------
// Constants

const double PUBLISH_FREQUENCY = 10.0;

//------------------------------------------------------------------------------
// Member variables

char _active_cmd = 0;
double _angle_to_rotate = 0;
Vector2i _encoders;
Vector2i _target;

double _kp = 1.0; std::string _kp_key = "/controller/turn/kp";
double _convergence_threshold = 0.1; std::string _convergence_threshold_key = "/controller/turn/conv_thresh";

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
    _encoders(0) = encoders->encoder1;
    _encoders(1) = encoders->encoder2;
}

//------------------------------------------------------------------------------
// Methods

void update_params() {
    ros::param::getCached(_convergence_threshold_key, _convergence_threshold);
    ros::param::getCached(_kp_key, _kp);
}

void send_done_message(bool flag, ros::Publisher& publisher) {
    std_msgs::Bool done;
    done.data = flag;
    publisher.publish(done);
}

double update_angular_velocity(double w)
{
    int32_t state = _target(0) - _encoders(0);
    return w+pd::P_control(_kp, (double)state, 0.0);
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
    ros::Publisher pub_done = n.advertise<std_msgs::Bool>("/controller/turn/done", 1);

    ros::Rate loop_rate(PUBLISH_FREQUENCY);

    geometry_msgs::Twist twist;

    double w_last = 0;
    double w = 0;
    double dt = 1.0/PUBLISH_FREQUENCY;

    while (n.ok()) {

        if (_angle_to_rotate != 0)
        {
            update_params();

            w = update_angular_velocity(w);

            //stop rotating when the angular velocity is stabilized
            if (std::abs(w - w_last)/dt < _convergence_threshold)
            {
                _angle_to_rotate = 0;
                w = w_last = 0;

                send_done_message(true, pub_done);
            }
            else {
                w_last = w;
            }

            twist.angular.y = w;
            pub_twist.publish(twist);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
