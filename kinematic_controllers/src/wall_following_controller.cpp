#include <kinematic_controllers/wall_following_controller.h>
#include <pid.h>


WallFollowingController::WallFollowingController(ros::NodeHandle &handle, 
                                                double update_frequency)
    :ControllerBase(handle, update_frequency)
    ,_kp("/controller/wall_follow/kp", 3)
    ,_kp2("/controller/wall_follow/kp2", 30)
	,_no_wall_th("/controller/wall_follow/_no_wall_th",0.30)
    ,_active(false)
    ,_ch1(0)
    ,_ch2(0)
    ,_ch3(0)
    ,_ch4(0)
    ,_d1(0)
    ,_d2(0)
    ,_d3(0)
    ,_d4(0)
    ,_twist(new geometry_msgs::Twist)
{
    _sub_sens = _handle.subscribe("/arduino/adc", 1000, 
                  &WallFollowingController::callback_sensors, this);
    _sub_act = _handle.subscribe("/controller/wall_follow/active", 1, 
                  &WallFollowingController::callback_activate, this);
}

WallFollowingController::~WallFollowingController()
{}

void WallFollowingController::callback_sensors(const ras_arduino_msgs::ADConverter lecture) {
    _ch1 = lecture.ch1;
    _ch2 = lecture.ch2;
    _ch3 = lecture.ch3;
    _ch4 = lecture.ch4;
    _d1 = robot::ir::distance(1,_ch1); //+ robot::ir::offset_front_left;
    _d2 = robot::ir::distance(2,_ch2); //+ robot::ir::offset_front_right;
    _d3 = robot::ir::distance(3,_ch3); //+ robot::ir::offset_rear_left;
    _d4 = robot::ir::distance(4,_ch4); //+ robot::ir::offset_rear_right;
	/*ROS_INFO("Distance fl(1): %f\n", _d1);
  	ROS_INFO("Distance fr(2): %f\n", _d2);
  	ROS_INFO("Distance bl(3): %f\n", _d3);
  	ROS_INFO("Distance br(4): %f\n", _d4);*/
}

void WallFollowingController::callback_activate(const std_msgs::BoolConstPtr& val) {
    _active = val->data;
}

geometry_msgs::TwistConstPtr WallFollowingController::update()
{
    if (_active)
    {
        if(_d1<_no_wall_th() && _d2<_no_wall_th()) //d1<0.30 && d2<0.30
        {
            if (_d1 < 0.1) //front left side < 0.1
            {
                _twist->angular.z = pd::P_control(_kp2(),_d3,_d1);
            } else if (_d2 < 0.1) //front right side < 0.1
            {
                _twist->angular.z = -pd::P_control(_kp2(),_d4,_d2);
            } else
            {
                _twist->angular.z = pd::P_control(_kp(),_d2+_d3,_d1+_d4);
            }
        } else if (_d1<_no_wall_th() && _d2>=_no_wall_th()) //d1<0.30 && d2>=0.30
        {
			if (_d1 < 0.1) //front left side < 0.1
            {
                _twist->angular.z = pd::P_control(_kp2(),_d3,_d1);
			} else
			{
            	_twist->angular.z = pd::P_control(_kp(),_d3,_d1);
			}
        } else if (_d1>=_no_wall_th() && _d2<_no_wall_th()) //d1>=0.30 && d2<0.30
        {
			if (_d2 < 0.1) //front right side < 0.1
            {
                _twist->angular.z = -pd::P_control(_kp2(),_d4,_d2);
            } else
			{
            	_twist->angular.z = -pd::P_control(_kp(),_d4,_d2);
			}
        } else //_d1>=30 && _d2>=30
        {
            _twist->angular.z = 0;
        }
    } else
    {
        _twist->angular.z = 0;
    }

    return _twist;
}

/*
int _ch1,_ch2,_ch3,_ch4;
double _d1,_d2,_d3,_d4;
bool _active;

Parameter<double> _kp("/controller/wall_follow/kp", 3);
Parameter<double> _kp2("/controller/wall_follow/kp2", 10);


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
  //Init the node
  ros::init(argc, argv, "line_cartesian_controller");

  ros::NodeHandle n;

  //Publish to the topic "/controller/align/twist"
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
  
  //Subscribe to the topic "/arduino/adc" 
  ros::Subscriber sub = n.subscribe("/arduino/adc",1000,sensorCallback);
  ros::Subscriber sub_act = n.subscribe("/controller/wall_follower/active", 
                  1, callback_activate);

  ros::Rate loop_rate(10);

  //Message to send
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
*/
