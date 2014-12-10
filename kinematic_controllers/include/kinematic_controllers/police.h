#ifndef POLICE_H
#define POLICE_H

#include "controller_base.h"
#include <common/robot.h>
#include <common/parameter.h>
#include <ir_converter/Distance.h>

class Police : public ControllerBase {
public:
    Police(ros::NodeHandle& handle, double freq);

    virtual geometry_msgs::TwistConstPtr update();

protected:

    void callback_ir(const ir_converter::DistanceConstPtr& distances);

    ros::Subscriber _sub_ir;

    Parameter<double> _max_velocity;
    Parameter<double> _min_velocity;
    Parameter<double> _activate_when_closer_than;

    geometry_msgs::TwistPtr _twist;
};

Police::Police(ros::NodeHandle& handle, double freq)
    :ControllerBase(handle, freq)
    ,_max_velocity("/controller/police/max_velocity",0.2)
    ,_min_velocity("/controller/police/min_velocity",0.05)
    ,_activate_when_closer_than("/controller/police/wall_thresh",0.16)
{
    _twist = geometry_msgs::TwistPtr(new geometry_msgs::Twist);
    _twist->angular.z = 0;
    _twist->linear.x = 0;

    _sub_ir = handle.subscribe<ir_converter::Distance>("/perception/ir/distance",10,&Police::callback_ir,this);
}

geometry_msgs::TwistConstPtr Police::update()
{
    return _twist;
}

double scale_velocity(double min_v, double max_v, double factor)
{
    return min_v + (max_v-min_v)*factor;
}

void Police::callback_ir(const ir_converter::DistanceConstPtr &distances)
{
    double min_dist = std::min(distances->bl_side, distances->br_side);
    min_dist = std::min(min_dist, distances->fl_side);
    min_dist = std::min(min_dist, distances->fr_side);

    if (min_dist < _activate_when_closer_than())
    {
        double closest = 0.12;
        double furthest = std::max(0.0, (_activate_when_closer_than() - closest));

        min_dist = std::max(0.0, min_dist-closest);

        double factor = (min_dist / furthest);
        double vel = scale_velocity(_min_velocity(), _max_velocity(), factor);

        ros::param::set("/controller/forward/velocity",vel);
    }
}





#endif // POLICE_H
