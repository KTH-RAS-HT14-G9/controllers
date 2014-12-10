#include <kinematic_controllers/controller_base.h>

ControllerBase::ControllerBase(ros::NodeHandle& handle, double update_frequency)
    :_handle(handle)
    ,_update_frequency(update_frequency)
{
}

ControllerBase::~ControllerBase()
{
}

bool ControllerBase::is_active()
{
    return true;
}

geometry_msgs::TwistConstPtr ControllerBase::update()
{
    return geometry_msgs::TwistPtr(new geometry_msgs::Twist);
}

void ControllerBase::hard_reset()
{
}
