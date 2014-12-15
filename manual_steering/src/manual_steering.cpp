#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manual_steering");

    ros::NodeHandle n;
    ros::Publisher pub_fwd = n.advertise<std_msgs::Bool>("/controller/forward/active",10);
    ros::Publisher pub_turn = n.advertise<std_msgs::Float64>("/controller/turn/angle",10);

    ros::Rate rate(50.0);

    double velocity;
    ros::param::getCached("/controller/forward/velocity", velocity);
    double cur_vel = 0;

    std_msgs::Bool msg_bool;
    std_msgs::Float64 msg_angle;

    char cmd[32];

    while(ros::ok())
    {
        std::cin.getline(cmd, 32);
        if(cmd[0]!='w' && cmd[0]!='a' && cmd[0]!='s' && cmd[0]!='d')
        {
            //ROS_ERROR("unknown command: %c",cmd);
            continue;
        }


        if (cmd[0]=='w') {

            if (cur_vel == 0) {
                cur_vel = 1.0*velocity;
                ros::param::set("/controller/forward/velocity",cur_vel);
                msg_bool.data = true;
                pub_fwd.publish(msg_bool);
            }
            else {
                cur_vel = 0;
                msg_bool.data = false;
                pub_fwd.publish(msg_bool);
            }

        }
        else if (cmd[0]=='s') {
            if (cur_vel == 0) {
                cur_vel = -1.0*velocity;
                ros::param::set("/controller/forward/velocity",cur_vel);
                msg_bool.data = true;
                pub_fwd.publish(msg_bool);
            }
            else {
                cur_vel = 0;
                msg_bool.data = false;
                pub_fwd.publish(msg_bool);
            }
        }


        if (cmd[0]=='a') {
            msg_angle.data = 90.0;
            pub_turn.publish(msg_angle);
        }
        else if (cmd[0]=='d') {
            msg_angle.data = -90.0;
            pub_turn.publish(msg_angle);
        }
    }

    return 0;
}
