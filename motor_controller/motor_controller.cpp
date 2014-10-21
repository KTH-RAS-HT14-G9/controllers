 #include <ros/ros.h>
#include <ras_arduino_msgs/PWM.h>
#include <ras_arduino_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <stdlib.h> 
#include "../etc/pid.h"

class MotorController 
{

public:

	static const double control_frequency = 10.0;
  	static const double ticks_per_rev = 360.0;
  	static const double wheel_radius = 0.049; 
  	static const double wheel_distance = 0.21; 

	MotorController() : linear_velocity(0.0), angular_velocity(0.0), 
						left_encoder_delta(0), right_encoder_delta(0),
						left_pwm(0), right_pwm(0),
						left_kp(0), left_ki(0), left_kd(0),
						right_kp(0), right_ki(0), right_kd(0),
						left_controller(left_kp, left_ki, left_kd),
						right_controller(right_kp, right_ki, right_kd)
	{
		handle = ros::NodeHandle("");
		initialise_pid_params();
		update_pid_params();

		//*left_controller = pid_1d(left_kp, left_ki, left_kd);
		//*right_controller = pid_1d(right_kp, right_ki, right_kd);

		twist_subscriber = handle.subscribe("/motor_controller/twist", 1000, &MotorController::twistCallback, this);
		encoder_subscriber = handle.subscribe("/arduino/encoders", 1000, &MotorController::encoderCallback, this);
		pwm_publisher = handle.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1000);
	}

	void twistCallback(const geometry_msgs::Twist::ConstPtr &twist) 
	{
		linear_velocity = twist->linear.x;
		angular_velocity = twist->angular.z;
	}

	void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr& encoder_data) 
	{
		left_encoder_delta = encoder_data->delta_encoder1;
		right_encoder_delta = encoder_data->delta_encoder2;
	}

	void updatePWM() 
	{
		updateLeftPWM();
		updateRightPWM();
	}

	void publishPWM() const
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

	bool ok() const
	{
		return handle.ok();
	}

	void update_pid_params() 
	{
		ros::param::get(p_l_key, left_kp);
    	ros::param::get(p_r_key, right_kp);
    	ros::param::get(d_l_key, left_kd);
    	ros::param::get(d_r_key, right_kd);
    	ros::param::get(i_l_key, left_ki);
    	ros::param::get(i_r_key, right_ki);
    	left_controller.set_kp_1d(left_kp);
     	left_controller.set_ki_1d(left_ki);
    	left_controller.set_kd_1d(left_kd);
    	right_controller.set_kp_1d(right_kp);
    	right_controller.set_ki_1d(right_ki);
    	right_controller.set_kd_1d(right_kd);

    	ros::param::get(left_const_key, left_const);
    	ros::param::get(right_const_key, right_const);
	}

private:

	double left_kp; std::string p_l_key;
	double right_kp; std::string p_r_key;
	double left_kd;  std::string d_l_key;
	double right_kd; std::string d_r_key;
	double left_ki; std::string i_l_key;
	double right_ki; std::string i_r_key;
	int left_const; std::string left_const_key;
	int right_const; std::string right_const_key;

  	double linear_velocity;
  	double angular_velocity;
  	int left_encoder_delta;
  	int right_encoder_delta;
  	int left_pwm;
  	int right_pwm;

	ros::NodeHandle handle;
	ros::Subscriber twist_subscriber;
	ros::Subscriber encoder_subscriber;
	ros::Publisher pwm_publisher;
	
	pid_1d left_controller;
  	pid_1d right_controller;

	double estimated_angular_velocity(int encoder_delta) const
	{
		  return -((double) encoder_delta)*2.0*M_PI*control_frequency/ticks_per_rev;
	}

	void updateLeftPWM() 
	{
		double estimated = estimated_angular_velocity(left_encoder_delta);
		double target = left_target_angular_velocity();
		left_pwm = get_left_constant() + (int) left_controller.control_1d(estimated, target, 1.0/control_frequency);
		left_pwm = left_pwm > 255 ? 255: left_pwm;
	}

	void updateRightPWM()
	{
		double estimated = estimated_angular_velocity(right_encoder_delta);
		double target = right_target_angular_velocity();
		right_pwm = get_right_constant() + (int) right_controller.control_1d(estimated, target, 1.0/control_frequency);
		right_pwm = right_pwm > 255 ? 255: right_pwm;
	}

	double left_target_angular_velocity() const
	{
		return (linear_velocity - wheel_distance/2.0*angular_velocity)/wheel_radius;
	}

	double right_target_angular_velocity() const 
	{
		return (linear_velocity + wheel_distance/2.0*angular_velocity)/wheel_radius;
	}


	int get_left_constant() {
		return get_constant(left_pwm, left_const, estimated_angular_velocity(left_encoder_delta), left_target_angular_velocity());
	}

	int get_right_constant() {
		return get_constant(right_pwm, right_const, estimated_angular_velocity(right_encoder_delta), right_target_angular_velocity());
	}

	int get_constant(int pwm, int max_constant, int current_ang_vel, int target_ang_vel) {
		/*if(pwm < max_constant && current_ang_vel*wheel_radius <= 0.2 && linear_velocity > 0 )
			return max_constant - pwm;
		return 0;*/
		return 0;
		/*if(abs(current_ang_vel) >= abs(target_ang_vel) || current_ang_vel > target_ang_vel)
			return 0;
		return max_constant*(target_ang_vel - current_ang_vel)/target_ang_vel;*/
	}

	void initialise_pid_params() 
	{
		left_kp = 5.0; p_l_key = "/pid/p_left";
		left_ki = 2.0; i_l_key = "/pid/i_left";
		left_kd = 0.5; d_l_key = "/pid/d_left";
		
		right_kp = 5.0; p_r_key = "/pid/p_right";
		right_ki = 2.0; i_r_key = "/pid/i_right";
		right_kd = 0.5; d_r_key = "/pid/d_right";
		left_const = 70; left_const_key = "/pid/left_const";
		right_const = 60; right_const_key = "/pid/right_const";
		
		// first time only
/*
		handle.setParam(p_l_key,left_kp);
   		handle.setParam(p_r_key,right_kp);
    	handle.setParam(d_l_key,left_kd);
    	handle.setParam(d_r_key,right_kd);
    	handle.setParam(i_l_key,left_ki);
    	handle.setParam(i_r_key,right_ki);
    	handle.setParam(left_const_key, left_const);
    	handle.setParam(right_const_key, right_const);*/
	}

};

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "motor_controller");
	
	MotorController mc;
	ros::Rate loop_rate(mc.control_frequency);

	while(mc.ok()) 
	{	
		mc.update_pid_params();
		mc.updatePWM();
		mc.publishPWM();
		ros::spinOnce();
		loop_rate.sleep();
	}
}
