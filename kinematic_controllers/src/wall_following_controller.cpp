#include <ros/ros.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <sstream>
#include <math.h>
#include <pid.h>
#include <common/robot.h>
#include <common/parameter.h>
#include <std_msgs/Bool.h>

int _ch1,_ch2,_ch3,_ch4;
double _d1,_d2,_d3,_d4;
bool _active;

Parameter<double> _kp("/controller/wall_follow/kp", 3);
Parameter<double> _kp2("/controller/wall_follow/kp2", 10);

/**
 * Function that will be executed when the values from the distance
 * sensors have been read
 */
void sensorCallback(const ras_arduino_msgs::ADConverter lecture)
{
  _ch1 = lecture.ch1;
  _ch2 = lecture.ch2;
  _ch3 = lecture.ch3;
  _ch4 = lecture.ch4;
  _d1 = robot::ir::distance(1,_ch1);
  _d2 = robot::ir::distance(2,_ch2);
  _d3 = robot::ir::distance(3,_ch3);
  _d4 = robot::ir::distance(4,_ch4);
  ROS_INFO("Distance fl(1): %f\n", _d1);
  ROS_INFO("Distance fr(2): %f\n", _d2);
  ROS_INFO("Distance bl(3): %f\n", _d3);
  ROS_INFO("Distance br(4): %f\n", _d4);
}

void callback_activate(const std_msgs::BoolConstPtr& val) {
    _active = val->data;
}


int main(int argc, char **argv)
{
  /**
   * Init the node
   */
  ros::init(argc, argv, "line_cartesian_controller");

  ros::NodeHandle n;

  /**
   * Publish to the topic "/controller/align/twist"
   */
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
  /*
   * Subscribe to the topic "/arduino/adc"
   */
  ros::Subscriber sub = n.subscribe("/arduino/adc",1000,sensorCallback);
  ros::Subscriber sub_act = n.subscribe("/controller/wall_follower/active", 1, callback_activate);

  ros::Rate loop_rate(10);

  /**
   * Message to send
   */
  geometry_msgs::Twist twist;
  _ch1 = 0;
  _ch2 = 0;
  _ch3 = 0;
  _ch4 = 0;
  _d1 = 0;
  _d2 = 0;
  _d3 = 0;
  _d4 = 0;

  twist.linear.x = 0;  //linear velocity
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;

  double state;
  double target;

  while (ros::ok())
  {
	if (_active)
	{
		if(_d1<0.25 && _d2<0.25)
		{
			if (_d1 < 0.1) //front left side < 0.1
			{
			  //twist.angular.z = _kp2*(0.1-_d1);
			  state = _d1;
			  target = 0.1;
			  twist.angular.z = pd::P_control(_kp2(),state,target);
			} else if (_ch2 < 0.1) //front right side < 0.1
			{
			  //twist.angular.z = -_kp2*(0.1-_d2);
			  state = _d2;
			  target = 0.1;
			  twist.angular.z = -pd::P_control(_kp2(),state,target);
			} else
			{
			  //twist.angular.z = _kp*((_d1+_d4)-(_d2+_d3));
			  state = _d2+_d3;
			  target = _d1+_d4;
			  twist.angular.z = pd::P_control(_kp(),state,target);
			}
		} else if (_d1<0.25 && _d2>=0.25)
		{
			state = _d3;
			target = _d1;
			twist.angular.z = pd::P_control(_kp(),state,target);
		} else if (_d1>=0.25 && _d2<0.25)
		{
			state = _d4;
			target = _d2;
			twist.angular.z = -pd::P_control(_kp(),state,target);
		} else 
		{
			twist.angular.z = 0;
		}
	} else
	{
		twist.angular.z = 0;
	}
    

    pub.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
	
  }

  return 0;
}
