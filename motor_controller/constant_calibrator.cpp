#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <ras_arduino_msgs/PWM.h>

bool _lock_left = false;
bool _lock_right = false;

ras_arduino_msgs::PWM _pwm;

void callback_encoders(const ras_arduino_msgs::EncodersConstPtr& encoders)
{
    if (encoders->delta_encoder1 > 1)
    {
        _lock_left = true;
        ROS_INFO("Locking left. PWM = %d\n",_pwm.PWM1);
        _pwm.PWM1 = 0;
    }
    if (encoders->delta_encoder2 > 1)
    {
        _lock_right = true;
        ROS_INFO("Locking right. PWM = %d\n",_pwm.PWM2);
        _pwm.PWM2 = 0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "constant_calibrator");

    ros::NodeHandle n;
    ros::Publisher pub_pwm = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 10);
    ros::Subscriber sub_enc = n.subscribe("/arduino/encoders",10,callback_encoders);

    const double freq = 100.0;
    ros::Rate rate(freq);

    _pwm.PWM1 = 0;
    _pwm.PWM2 = 0;

    double time_to_increment = 0.5;
    double dt = time_to_increment;

    while(n.ok())
    {
        if (dt >= time_to_increment)
        {
            if (!_lock_left) _pwm.PWM1 += 1;
            if (!_lock_right) _pwm.PWM2 += 1;

            ROS_INFO("Publishing: %d, %d\n", _pwm.PWM1, _pwm.PWM2);

            pub_pwm.publish(_pwm);
            dt = 0;
        }
        dt += 1.0/freq;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
