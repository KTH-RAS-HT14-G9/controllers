#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "topic_subscriber.h"

//------------------------------------------------------------------------------
// Constants

const double PUBLISH_FREQUENCY = 10.0;

//------------------------------------------------------------------------------
// Member

geometry_msgs::Twist _twist;

//------------------------------------------------------------------------------
// Entry point

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_adapter");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
    ros::Rate loop_rate(PUBLISH_FREQUENCY);

    TopicSubscriber subscribers[] = {
        TopicSubscriber(nh,"/controller/forward/twist",1),
        TopicSubscriber(nh,"/controller/turn/twist",1)
    };
    int nSubscribers = sizeof(subscribers)/sizeof(TopicSubscriber);


    while(ros::ok()) {

        _twist.linear.x = 0;
        _twist.linear.y = 0;
        _twist.linear.z = 0;
        _twist.angular.x = 0;
        _twist.angular.y = 0;
        _twist.angular.z = 0;

        for(int i = 0; i < nSubscribers; ++i) {

            const geometry_msgs::Twist& twist = subscribers[i].get_twist();

            //combine the twists blindly

            _twist.angular.x += twist.angular.x;
            _twist.angular.y += twist.angular.y;
            _twist.angular.z += twist.angular.z;

            _twist.linear.x += twist.linear.x;
            _twist.linear.y += twist.linear.y;
            _twist.linear.z += twist.linear.z;
        }

        pub.publish(_twist);
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
