#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <ras_arduino_msgs/PWM.h>
#include <common/lowpass_filter.h>

int _power_pwm_l, _power_pwm_r;
int _sustain_pwm_l, _sustain_pwm_r;

bool _lock_left = false;
bool _lock_right = false;

int _step = 1;

ras_arduino_msgs::PWM _pwm;

common::LowPassFilter _filter_left(0.5);
common::LowPassFilter _filter_right(0.5);

void step1_auto_locking(double left, double right)
{
    if (std::abs(left) > 5)
    {
        _lock_left = true;
        _power_pwm_l = _pwm.PWM1;
        ROS_INFO("Min PWM to achieve movement left = %d\n",_pwm.PWM1);
    }
    if (std::abs(right) > 5)
    {
        _lock_right = true;
        _power_pwm_r = _pwm.PWM2;
        ROS_INFO("Min PWM to achieve moment right = %d\n",_pwm.PWM2);
    }

    if (_lock_left && _lock_right)
    {
        ROS_INFO("Commencing step 2. Decreasing PWMs until standing");
        _step = 2;

        _lock_left = _lock_right = false;
    }
}

void step2_auto_locking(double left, double right)
{
    if (std::abs(left) < 1e-6)
    {
        _lock_left = true;
        _sustain_pwm_l = _pwm.PWM1;
        ROS_INFO("Max PWM to sustain movement left = %d\n",_pwm.PWM1);
    }
    if (std::abs(right) < 1e-6)
    {
        _lock_right = true;
        _sustain_pwm_r = _pwm.PWM2;
        ROS_INFO("Max PWM to sustain movement right = %d\n",_pwm.PWM2);
    }

    if (_lock_left && _lock_right)
    {
        ROS_INFO("Finished.");
        ROS_INFO("Power pwms: %d, %d", _power_pwm_l, _power_pwm_r);
        ROS_INFO("Sustain pwms: %d, %d", _sustain_pwm_l, _sustain_pwm_r);

        _pwm.PWM1 = _pwm.PWM2 = 0;
    }
}

void callback_encoders(const ras_arduino_msgs::EncodersConstPtr& encoders)
{
    double left = _filter_left.filter(encoders->delta_encoder1);
    double right = _filter_right.filter(encoders->delta_encoder2);

    if (_step == 1)
        step1_auto_locking(left,right);
    else
        step2_auto_locking(left,right);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "constant_calibrator");

    ros::NodeHandle n;
    ros::Publisher pub_pwm = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 10);
    ros::Subscriber sub_enc = n.subscribe("/arduino/encoders",10,callback_encoders);

    const double freq = 100.0;
    ros::Rate rate(freq);

    _pwm.PWM1 = 45;
    _pwm.PWM2 = -40;

    _filter_left.filter(0);
    _filter_right.filter(0);

    double time_to_increment = 0.2;
    double dt = time_to_increment;

    while(n.ok())
    {
        int dpwm = 1;
        if (_step == 2) dpwm = -dpwm;

        if (dt >= time_to_increment)
        {
            if (!_lock_left) _pwm.PWM1 += dpwm;
            if (!_lock_right) _pwm.PWM2 -= dpwm;

            if (!_lock_left || !_lock_right) {
                ROS_INFO("Publishing: %d, %d\n", _pwm.PWM1, _pwm.PWM2);

                pub_pwm.publish(_pwm);
            }
            else {
                break;
            }
            dt = 0;
        }
        dt += 1.0/freq;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
