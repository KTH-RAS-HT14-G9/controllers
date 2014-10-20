#include <ros/ros.h>
#include <ras_arduino_msgs/PWM.h>
#include <ras_arduino_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "../etc/pid.h"

class MotorController 
{

public:

	static const double control_frequency = 10.0;
  	static const double ticks_per_rev = 360.0;
  	static const double wheel_radius = 0.049; 
  	static const double wheel_distance = 0.21; 

  	double left_kp; std::string p_l_key;
	double right_kp; std::string p_r_key;

	double left_kd;  std::string d_l_key;
	double right_kd; std::string d_r_key;

	double left_ki; std::string i_l_key;
	double right_ki; std::string i_r_key;

	pid_1d left_controller;
  	pid_1d right_controller;

	MotorController() : linear_velocity(0.0), angular_velocity(0.0), 
						left_encoder_delta(0), right_encoder_delta(0),
						left_pwm(0), right_pwm(0),
						left_kp(3.4), right_kp(3.0),
						left_kd(0.8), right_kd(0.8),
						left_ki(0.06), right_ki(0.05),
						left_controller(left_kp, left_ki, left_kd),
  						right_controller(right_kd, right_ki, right_kd)

	{
		p_l_key = "/pid/p_left";
		p_r_key = "/pid/p_right";
		d_l_key = "/pid/d_left";
		d_r_key = "/pid/d_right";
		i_l_key = "/pid/i_left";
		i_r_key = "/pid/i_right";
		
		handle = ros::NodeHandle("");

		handle.setParam(p_l_key,left_kp);
   		handle.setParam(p_r_key,right_kp);

    	handle.setParam(d_l_key,left_kd);
    	handle.setParam(d_r_key,right_kd);

    	handle.setParam(i_l_key,left_ki);
    	handle.setParam(i_r_key,right_ki);

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
		ras_arduino_msgs::PWM pwm;
		pwm.PWM1 = left_pwm;
		pwm.PWM2 = right_pwm;
   		pwm_publisher.publish(pwm);
	}

	bool ok() const
	{
		return handle.ok();
	}

private:

  	/*static const double left_kp = 1.0;
  	static const double left_ki = 0.0;
  	static const double left_kd = 0.0;
  	static const double right_kp = 1.0;
  	static const double right_ki = 0.0;
  	static const double right_kd = 0.0;*/

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
	

	double estimated_angular_velocity(int encoder_delta) 
	{
		  return ((double) encoder_delta)*2.0*M_PI*control_frequency/ticks_per_rev;
	}

	void updateLeftPWM() 
	{
		double estimated = estimated_angular_velocity(left_encoder_delta);
		double target = target_angular_velocity();
		left_pwm += (int) left_controller.control_1d(estimated, target, 1.0/control_frequency);
	}

	void updateRightPWM()
	{
		double estimated = estimated_angular_velocity(right_encoder_delta);
		double target = target_angular_velocity();
		right_pwm += (int) right_controller.control_1d(estimated, target, 1.0/control_frequency);
	}

	double target_angular_velocity()
	{
		return (linear_velocity - wheel_distance/2.0*angular_velocity)/wheel_radius;
	}

};

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "motor_controller");
	
	MotorController mc;
	ros::Rate loop_rate(mc.control_frequency);

	while(mc.ok()) {
		ros::param::get(mc.p_l_key, mc.left_kp);
    	ros::param::get(mc.p_r_key, mc.right_kp);
    	ros::param::get(mc.d_l_key, mc.left_kd);
    	ros::param::get(mc.d_r_key, mc.right_kd);
    	ros::param::get(mc.i_l_key, mc.left_ki);
    	ros::param::get(mc.i_r_key, mc.right_ki);

    	mc.left_controller.set_kp_1d(mc.left_kp);
    	mc.left_controller.set_ki_1d(mc.left_ki);
    	mc.left_controller.set_kd_1d(mc.left_kd);

    	mc.right_controller.set_kp_1d(mc.right_kp);
    	mc.right_controller.set_ki_1d(mc.right_ki);
    	mc.right_controller.set_kd_1d(mc.right_kd);

		mc.updatePWM();
		mc.publishPWM();
		ros::spinOnce();
		loop_rate.sleep();
	}


}
