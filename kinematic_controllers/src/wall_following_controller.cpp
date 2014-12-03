#include <kinematic_controllers/wall_following_controller.h>
#include <pid.h>


WallFollowingController::WallFollowingController(ros::NodeHandle &handle,
                                                 double update_frequency)
    :ControllerBase(handle, update_frequency)
    ,_kp_single("/controller/wall_follow/single/kp", 8.0)
    ,_kp_double("/controller/wall_follow/double/kp", 5.0)
    ,_wall_th("/controller/wall_follow/wall_th", 0.40)
    ,_active(false)
    ,_fl_side(0)
    ,_fr_side(0)
    ,_bl_side(0)
    ,_br_side(0)
    ,_twist(new geometry_msgs::Twist)
{
    _sub_sens = _handle.subscribe("/perception/ir/distance", 10,
                                  &WallFollowingController::callback_sensors, this);
    _sub_act = _handle.subscribe("/controller/wall_follow/active", 10,
                                 &WallFollowingController::callback_activate, this);
}

WallFollowingController::~WallFollowingController()
{}

void WallFollowingController::callback_sensors(const ir_converter::DistanceConstPtr& distances) {
    using namespace robot::ir;
    _fl_side = distances->fl_side;
    _fr_side = distances->fr_side;
    _bl_side = distances->bl_side;
    _br_side = distances->br_side;
}

void WallFollowingController::callback_activate(const std_msgs::BoolConstPtr& val) {
    _active = val->data;
}

geometry_msgs::TwistConstPtr WallFollowingController::update()
{
    _twist->angular.z = 0;

    if (_active)
    {
        if(leftWallClose() && rightWallClose())
        {
            _twist->angular.z = pd::P_control(_kp_double(),_bl_side,_br_side) - pd::P_control(_kp_double(),_fl_side,_fr_side);

        } else if (leftWallClose())
        {
            _twist->angular.z = pd::P_control(_kp_single(),_bl_side, _fl_side);
        }
        else if (rightWallClose())
        {
            _twist->angular.z = -pd::P_control(_kp_single(),_br_side,_fr_side);
        }
    }
    return _twist;
}

bool WallFollowingController::leftWallClose()
{
    return _fl_side<_wall_th() && _bl_side<_wall_th();
}

bool WallFollowingController::rightWallClose()
{
    return _fr_side<_wall_th() && _br_side<_wall_th();
}
