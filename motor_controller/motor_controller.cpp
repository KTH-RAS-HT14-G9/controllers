#include "motor_controller.h"
#include <common/util.h>

//------------------------------------------------------------------------------
// Constructor - Deconstructor

MotorController::MotorController()
    :linear_velocity(0.0), angular_velocity(0.0)
    ,left_encoder_delta(0), right_encoder_delta(0)
    ,left_pwm(0), right_pwm(0)
    ,left_p("/pid/left_p", 0.75)
    ,left_i("/pid/left_i", 0.0)
    ,left_d("/pid/left_d", 0.1)
    ,right_p("/pid/right_p", 0.5)
    ,right_i("/pid/right_i", 0.0)
    ,right_d("/pid/right_d", 0.1)
    ,_power_pwm_left("/pid/power_pwm_left", 65)
    ,_power_pwm_right("/pid/power_pwm_right", 64)
    ,_sustain_pwm_left("/pid/sustain_pwm_left", 48)
    ,_sustain_pwm_right("/pid/sustain_pwm_right", 47)
{
    handle = ros::NodeHandle("");

    left_controller = new pid_1d(left_p(), left_i(), left_d());
    right_controller = new pid_1d(right_p(), right_i(), right_d());

    update_pid_params();
    twist_subscriber = handle.subscribe("/motor_controller/twist", 1, &MotorController::twistCallback, this);
    encoder_subscriber = handle.subscribe("/arduino/encoders", 10, &MotorController::encoderCallback, this);
    reset_pid_subscriber = handle.subscribe("/controller/motor/reset", 1, &MotorController::resetPIDCallback, this);
    pwm_publisher = handle.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 10);
}

MotorController::~MotorController() {
    delete left_controller;
    delete right_controller;
}

//------------------------------------------------------------------------------
// Callbacks

void MotorController::twistCallback(const geometry_msgs::Twist::ConstPtr &twist)
{
    linear_velocity = twist->linear.x;
    angular_velocity = twist->angular.z;
}

void MotorController::encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr& encoder_data)
{
    left_encoder_delta = encoder_data->delta_encoder1;
    right_encoder_delta = encoder_data->delta_encoder2;
}

void MotorController::resetPIDCallback(const std_msgs::Bool::ConstPtr& data)
{
    left_controller->reset();
    right_controller->reset();
}

//------------------------------------------------------------------------------
// Methods

void MotorController::updatePWM()
{
    updateLeftPWM();
    updateRightPWM();
}

void MotorController::publishPWM()
{
    double estimated_left = estimated_angular_velocity(left_encoder_delta);
    double estimated_right = estimated_angular_velocity(right_encoder_delta);
    double target_left = left_target_angular_velocity();
    double target_right = right_target_angular_velocity();

	ROS_INFO("pwms pure \t l: %d, r: %d", (int)left_pwm, (int)right_pwm);
	ROS_INFO("pwm const \t l: %d, r: %d", get_left_const(), get_right_const());

    ras_arduino_msgs::PWM pwm;
    pwm.PWM1 = common::Clamp<int>((int)left_pwm + get_left_const(), -255, 255);;
    pwm.PWM2 = common::Clamp<int>((int)right_pwm + get_right_const(), -255, 255);;

    ROS_INFO("current angvel \t l: %f, r: %f", estimated_left, estimated_right);
    ROS_INFO("target angvel \t l:%f, r:%f", target_left, target_right);
    ROS_INFO("publishing \t l: %d, r: %d", pwm.PWM1, pwm.PWM2);
	ROS_INFO("------------------------------------------------\n");

    pwm_publisher.publish(pwm);
}

bool MotorController::ok() const
{
    return handle.ok();
}

void MotorController::update_pid_params()
{
    left_controller->set_kp_1d(left_p());
    left_controller->set_ki_1d(left_i());
    left_controller->set_kd_1d(left_d());
    right_controller->set_kp_1d(right_p());
    right_controller->set_ki_1d(right_i());
    right_controller->set_kd_1d(right_d());

    _spiker_left.set(_power_pwm_left(), _sustain_pwm_left());
    _spiker_right.set(_power_pwm_right(), _sustain_pwm_right());
}

double MotorController::estimated_angular_velocity(int encoder_delta) const
{
    return -((double) encoder_delta)*2.0*M_PI*robot::prop::encoder_publish_frequency/robot::prop::ticks_per_rev;
}

void MotorController::updateLeftPWM()
{
    double estimated = estimated_angular_velocity(left_encoder_delta);
    double target = left_target_angular_velocity();

	if (target == 0) {
		left_pwm = 0;
		left_controller->reset();
	}

    left_pwm += left_controller->control_1d(estimated, target, 1.0/robot::prop::encoder_publish_frequency);
}

void MotorController::updateRightPWM()
{
    double estimated = estimated_angular_velocity(right_encoder_delta);
    double target = right_target_angular_velocity();

	if (target == 0) {
		right_pwm = 0;
		right_controller->reset();
	}

    right_pwm += right_controller->control_1d(estimated, target, 1.0/robot::prop::encoder_publish_frequency);
}

double MotorController::left_target_angular_velocity() const
{
    return (linear_velocity - (robot::dim::wheel_distance/2.0)*angular_velocity)/robot::dim::wheel_radius;
}

double MotorController::right_target_angular_velocity() const
{
    return (linear_velocity + (robot::dim::wheel_distance/2.0)*angular_velocity)/robot::dim::wheel_radius;
}

int MotorController::get_left_const() {

    return _spiker_left.apply(estimated_angular_velocity(left_encoder_delta), left_target_angular_velocity());
}

int MotorController::get_right_const() {

    return _spiker_right.apply(estimated_angular_velocity(right_encoder_delta), right_target_angular_velocity());

}

//------------------------------------------------------------------------------
// Entry point

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_controller");
    MotorController mc;
    ros::Rate loop_rate(robot::prop::encoder_publish_frequency);

    while(mc.ok())
    {
        mc.update_pid_params();
        mc.updatePWM();
        mc.publishPWM();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
