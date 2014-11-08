#ifndef TURN_CONTROLLER_H
#define TURN_CONTROLLER_H

#include <kinematic_controllers/controller_base.h>
#include <common/parameter.h>
#include <std_msgs/Float64.h>
#include <ras_arduino_msgs/Encoders.h>
#include <std_msgs/Bool.h>
#include <Eigen/Core>

class TurnController : public ControllerBase
{
public:
    TurnController(ros::NodeHandle& handle, double update_frequency);

    virtual ~TurnController();

    virtual geometry_msgs::TwistConstPtr update();

private:

    //------------------------------------------------------------------------------
    // Method declarations
    void callback_turn_angle(const std_msgs::Float64ConstPtr& deg);
    void callback_encoders(const ras_arduino_msgs::EncodersConstPtr& encoders);

    void send_done_message(bool flag);
    double control_angular_velocity();

    //------------------------------------------------------------------------------
    // Member
    double _angle_to_rotate;
    Eigen::Vector2i _encoders_last;
    Eigen::Vector2i _encoders;
    Eigen::Vector2i _target;
    double _w, _w_last;
    geometry_msgs::TwistPtr _twist;

    //------------------------------------------------------------------------------
    // Parameter
    Parameter<double> _kp;
    Parameter<double> _kd;
    Parameter<double> _convergence_threshold_w;
    Parameter<int> _encoder_threshold;
    Parameter<double> _initial_w;
    Parameter<double> _limit_w;

    //------------------------------------------------------------------------------
    // Subscribers and publisher
    ros::Subscriber _sub_angle;
    ros::Subscriber _sub_enc;

    ros::Publisher _pub_done;
};

#endif // TURN_CONTROLLER_H
