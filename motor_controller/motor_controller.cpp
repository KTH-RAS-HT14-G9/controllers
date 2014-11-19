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
    ,left_d("/pid/left_d", 0.0)
    ,right_p("/pid/right_p", 0.5)
    ,right_i("/pid/right_i", 0.0)
    ,right_d("/pid/right_d", 0.0)
    ,left_const("/pid/left_const", 55)
    ,right_const("/pid/right_const", 55)
    ,_lower_pwm_thresh("/pid/lower_thresh", 0)
    ,_upper_pwm_thresh("/pid/upper_thresh", 20)
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

    int low = _lower_pwm_thresh();
    int upp = _upper_pwm_thresh();
    _hyst_left_pos.set(low, upp, 0, left_const());
    _hyst_right_pos.set(low, upp, 0, right_const());
    _hyst_left_neg.set(-upp, -low, -left_const(), 0);
    _hyst_right_neg.set(-upp, -low, -right_const(), 0);
}

double MotorController::estimated_angular_velocity(int encoder_delta) const
{
    return -((double) encoder_delta)*2.0*M_PI*robot::prop::encoder_publish_frequency/robot::prop::ticks_per_rev;
}

void MotorController::updateLeftPWM()
{
    double estimated = estimated_angular_velocity(left_encoder_delta);
    double target = left_target_angular_velocity();
    left_pwm += left_controller->control_1d(estimated, target, 1.0/robot::prop::encoder_publish_frequency);
}

void MotorController::updateRightPWM()
{
    double estimated = estimated_angular_velocity(right_encoder_delta);
    double target = right_target_angular_velocity();
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
    if (left_pwm > 0) return _hyst_left_pos.apply(left_pwm);
    if (left_pwm < 0) return _hyst_left_neg.apply(left_pwm);
    return 0;
}

int MotorController::get_right_const() {
    if (right_pwm > 0) return _hyst_right_pos.apply(right_pwm);
    if (right_pwm < 0) return _hyst_right_neg.apply(right_pwm);
    return 0;
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
