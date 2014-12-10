#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Time.h>
#include "kinematic_controllers/controller_base.h"
#include "kinematic_controllers/police.h"
#include "kinematic_controllers/turn_controller.h"
#include "kinematic_controllers/turn_controller_theta.h"
#include "kinematic_controllers/forward_controller.h"
#include "kinematic_controllers/wall_following_controller.h"
#include "kinematic_controllers/goto_controller.h"


//------------------------------------------------------------------------------
// Member

geometry_msgs::Twist _twist;
bool _hard_reset;

//------------------------------------------------------------------------------
// Callbacks
void callback_crash(const std_msgs::TimeConstPtr& time)
{
    ROS_ERROR("Received crash signal. Hard resetting all controllers");
    _hard_reset = true;
}

//------------------------------------------------------------------------------
// Entry point

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_adapter");

    _hard_reset = false;

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/motor_controller/twist", 10);
    ros::Rate loop_rate(robot::prop::encoder_publish_frequency);

    ros::Subscriber sub_crash = nh.subscribe<std_msgs::Time>("/perception/imu/peak", 10, callback_crash);

    ControllerBase* fwd_controller = new ForwardController(nh, robot::prop::encoder_publish_frequency);
    ControllerBase* goto_controller = new GotoController(nh, robot::prop::encoder_publish_frequency);

    ControllerBase* controllers[] = {
        new Police(nh, robot::prop::encoder_publish_frequency),
        fwd_controller,
        new TurnControllerTheta(nh, robot::prop::encoder_publish_frequency),
        new WallFollowingController(nh, robot::prop::encoder_publish_frequency),
        goto_controller
    };
    int nControllers = sizeof(controllers)/sizeof(ControllerBase*);


    while(ros::ok()) {

        _twist.linear.x = 0;
        _twist.linear.y = 0;
        _twist.linear.z = 0;
        _twist.angular.x = 0;
        _twist.angular.y = 0;
        _twist.angular.z = 0;

        for(int i = 0; i < nControllers; ++i) {

            if (_hard_reset) {
                controllers[i]->hard_reset();
            }

            if (controllers[i] == fwd_controller && goto_controller->is_active())
            {/* do not update forward controller */}
            else {
                const geometry_msgs::TwistConstPtr twist = controllers[i]->update();

                //combine the twists blindly
                _twist.angular.z += twist->angular.z;
                _twist.linear.x += twist->linear.x;
            }
        }

        if (_hard_reset) _hard_reset = false;

        pub.publish(_twist);
        ros::spinOnce();
        loop_rate.sleep();

    }

    //delete all controllers
    for(int i = 0; i < nControllers; ++i) {
        delete controllers[i];
    }

    return 0;
}
