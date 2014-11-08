#include "motor_controller.h"



MotorController::MotorController() :
    linear_velocity(0.0), angular_velocity(0.0),
    left_encoder_delta(0), right_encoder_delta(0),
    left_pwm(0), right_pwm(0),
    left_p("/pid/left_p", 12.5),
    left_i("/pid/left_i", 1.0),
    left_d("/pid/left_d", 0.5),
    right_p("/pid/right_p", 11.0),
    right_i("/pid/right_i", 1.0),
    right_d("/pid/right_d", 0.82),
    left_const("/pid/left_const", 47),
    right_const("/pid/right_const", 42)
{
    handle = ros::NodeHandle("");

    left_controller = new pid_1d(left_p(), left_i(), left_d());
    right_controller = new pid_1d(right_p(), right_i(), right_d());

    update_pid_params();
    twist_subscriber = handle.subscribe("/motor_controller/twist", 1000, &MotorController::twistCallback, this);
    encoder_subscriber = handle.subscribe("/arduino/encoders", 1000, &MotorController::encoderCallback, this);
    pwm_publisher = handle.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1000);
}

MotorController::~MotorController() {
    delete left_controller;
    delete right_controller;
}

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

void MotorController::updatePWM()
{
    updateLeftPWM();
    updateRightPWM();
}

void MotorController::publishPWM() const
{
    double estimated_left = estimated_angular_velocity(left_encoder_delta);
    double estimated_right = estimated_angular_velocity(right_encoder_delta);
    double target_left = left_target_angular_velocity();
    double target_right = right_target_angular_velocity();

    ROS_INFO("current angvel l: %f, r: %f", estimated_left, estimated_right);
    ROS_INFO("target angvel l:%f, r:%f", target_left, target_right);
    ROS_INFO("publishing l: %d, r: %d", left_pwm, right_pwm);

    ras_arduino_msgs::PWM pwm;
    pwm.PWM1 = left_pwm;
    pwm.PWM2 = right_pwm;
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
}

double MotorController::estimated_angular_velocity(int encoder_delta) const
{
    return -((double) encoder_delta)*2.0*M_PI*robot::prop::encoder_publish_frequency/robot::prop::ticks_per_rev;
}

void MotorController::updateLeftPWM()
{
    double estimated = estimated_angular_velocity(left_encoder_delta);
    double target = left_target_angular_velocity();
    left_pwm = get_left_const() + (int) left_controller->control_1d(estimated, target, 1.0/robot::prop::encoder_publish_frequency);
    left_pwm = left_pwm > 255 ? 255: left_pwm;
    left_pwm = left_pwm < -255 ? -255: left_pwm;
}

void MotorController::updateRightPWM()
{
    double estimated = estimated_angular_velocity(right_encoder_delta);
    double target = right_target_angular_velocity();
    right_pwm = get_right_const() + (int) right_controller->control_1d(estimated, target, 1.0/robot::prop::encoder_publish_frequency);
    right_pwm = right_pwm > 255 ? 255: right_pwm;
    right_pwm = right_pwm < -255 ? -255:right_pwm;
}

double MotorController::left_target_angular_velocity() const
{
    return (linear_velocity - robot::dim::wheel_distance/2.0*angular_velocity)/robot::dim::wheel_radius;
}

double MotorController::right_target_angular_velocity() const
{
    return (linear_velocity + robot::dim::wheel_distance/2.0*angular_velocity)/robot::dim::wheel_radius;
}

int MotorController::get_left_const() {
    if(linear_velocity > 0 || angular_velocity < 0)
        return left_const();
    if(linear_velocity < 0 || angular_velocity > 0)
        return -left_const();
    return 0;
}

int MotorController::get_right_const() {
    if(linear_velocity > 0 || angular_velocity > 0)
        return right_const();
    if(linear_velocity < 0 || angular_velocity < 0)
        return -right_const();
    return 0;

}

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
