#include <ros/ros.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <sstream>
#include <math.h>
#include <pid.h>
#include <common/robot.h>
#include <common/parameter.h>

int _ch1,_ch2,_ch3,_ch4;
double _d1,_d2,_d3,_d4;

Parameter<double> _kp("/controller/wall_follow/kp", -3);

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
    if (_d1 < 0.1) //front left side < 0.1
    {
      //twist.angular.z = _kp*(_d1-_d3);
      state = _d3;
      target = _d1;
      twist.angular.z = pd::P_control(_kp(),state,target);
    } else if (_ch2 < 0.1) //front right side < 0.1
    {
      //twist.angular.z = -_kp*(_d2-_d4);
      state = _d4;
      target = _d2;
      twist.angular.z = -pd::P_control(_kp(),state,target);
    } else
    {
      //twist.angular.z = _kp*((_d1+_d4)-(_d2+_d3));
      state = _d2+_d3;
      target = _d1+_d4;
      twist.angular.z = pd::P_control(_kp(),state,target);
    }
    

    pub.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
	
  }

  return 0;
}
