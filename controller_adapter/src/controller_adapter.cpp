#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


class TopicSubscriber {

public:
    TopicSubscriber(ros::NodeHandle& handle, const std::string& topic, int queue_size=100)
    {
        handle.subscribe(topic, queue_size, &TopicSubscriber::callback, this);
    }

    geometry_msgs::Twist get_twist() { return _twist; }

private:

    geometry_msgs::Twist _twist;

    void callback(const geometry_msgs::TwistConstPtr& data) {
        _twist = *data;
    }
};


geometry_msgs::Twist _twist;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_adapter");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
    ros::Rate loop_rate(10);


    TopicSubscriber subscribers[] = {
        TopicSubscriber(nh,"/controller/twist"),
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
