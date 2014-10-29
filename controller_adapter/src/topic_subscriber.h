#ifndef TOPIC_SUBSCRIBER_H
#define TOPIC_SUBSCRIBER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class TopicSubscriber {

public:
    TopicSubscriber(ros::NodeHandle& handle, const std::string& topic, int queue_size=100)
    {
        _subscriber = handle.subscribe(topic, queue_size, &TopicSubscriber::callback, this);
    }

    geometry_msgs::Twist get_twist() { return _twist; }

private:

    geometry_msgs::Twist _twist;
    ros::Subscriber _subscriber;

    void callback(const geometry_msgs::TwistConstPtr& data) {
        _twist = *data;
    }
};

#endif // TOPIC_SUBSCRIBER_H
