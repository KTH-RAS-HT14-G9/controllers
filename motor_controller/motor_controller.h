#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <ros/ros.h>
#include <ras_arduino_msgs/PWM.h>
#include <ras_arduino_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <stdlib.h>
#include <pid.h>
#include <common/robot.h>
#include <common/parameter.h>
#include <common/util.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include "PWMSpiker.h"

class MotorController {

public:
    MotorController();
    ~MotorController();
    void twistCallback(const geometry_msgs::Twist::ConstPtr &twist);
    void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr& encoder_data);
    void resetPIDCallback(const std_msgs::Bool::ConstPtr& data);
    void crashCallback(const std_msgs::TimeConstPtr& time);

    void updatePWM();
    void publishPWM();
    inline bool ok() const {handle.ok();}
    void update_pid_params();
private:
    void timer_callback(const ros::TimerEvent& event);

    double estimated_angular_velocity(int encoder_delta) const;
    void updateLeftPWM();
    void updateRightPWM();
    double left_target_angular_velocity() const;
    double right_target_angular_velocity() const;
    int get_left_const();
    int get_right_const();

    Parameter<double> left_p, left_i, left_d;
    Parameter<double> right_p, right_i, right_d;
    Parameter<int> _power_pwm_left, _power_pwm_right;
    Parameter<int> _sustain_pwm_left, _sustain_pwm_right;

    double linear_velocity, angular_velocity;
    int left_encoder_delta, right_encoder_delta;
    double left_pwm, right_pwm;

    bool _mute;

    ros::NodeHandle handle;
    ros::Subscriber twist_subscriber;
    ros::Subscriber encoder_subscriber;
    ros::Subscriber reset_pid_subscriber;
    ros::Subscriber crash_subscriber;

    ros::Publisher pwm_publisher;

    pid_1d * left_controller;
    pid_1d * right_controller;

    PWMSpiker _spiker_left;
    PWMSpiker _spiker_right;

};

#endif // MOTOR_CONTROLLER_H
