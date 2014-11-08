#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <ros/ros.h>
#include <ras_arduino_msgs/PWM.h>
#include <ras_arduino_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <stdlib.h>
#include "../etc/pid.h"
#include <common/robot.h>
#include <common/parameter.h>

class MotorController {

public:
    MotorController();
    ~MotorController();
    void twistCallback(const geometry_msgs::Twist::ConstPtr &twist);
    void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr& encoder_data);
    void updatePWM();
    void publishPWM() const;
    bool ok() const;
    void update_pid_params();
private:
    double estimated_angular_velocity(int encoder_delta) const;
    void updateLeftPWM();
    void updateRightPWM();
    double left_target_angular_velocity() const;
    double right_target_angular_velocity() const;
    int get_left_const();
    int get_right_const();

    Parameter<double> left_p, left_i, left_d;
    Parameter<double> right_p, right_i, right_d;
    Parameter<int> left_const, right_const;

    double linear_velocity, angular_velocity;
    int left_encoder_delta, right_encoder_delta;
    int left_pwm, right_pwm;

    ros::NodeHandle handle;
    ros::Subscriber twist_subscriber;
    ros::Subscriber encoder_subscriber;
    ros::Publisher pwm_publisher;

    pid_1d * left_controller;
    pid_1d * right_controller;

};

#endif // MOTOR_CONTROLLER_H
