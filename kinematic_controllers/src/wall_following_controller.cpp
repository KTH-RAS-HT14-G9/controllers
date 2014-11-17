#include <kinematic_controllers/wall_following_controller.h>
#include <pid.h>


WallFollowingController::WallFollowingController(ros::NodeHandle &handle, 
                                                 double update_frequency)
    :ControllerBase(handle, update_frequency)
    ,_kp_single("/controller/wall_follow/single/kp", 0.4)
    ,_kp_double("/controller/wall_follow/double/kp", 0.4)
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

    /*ROS_INFO("Distance fl(1): %f\n", fl_side);
    ROS_INFO("Distance fr(2): %f\n", fr_side);
    ROS_INFO("Distance bl(3): %f\n", bl_side);
    ROS_INFO("Distance br(4): %f\n", br_side);*/
}

void WallFollowingController::callback_activate(const std_msgs::BoolConstPtr& val) {
    _active = val->data;
}

geometry_msgs::TwistConstPtr WallFollowingController::update()
{
    if (_active)
    {
        if(leftWallClose() && rightWallClose())
        {
            //control sides
//            _twist->angular.z += pd::P_control(_kp(),fl_side,bl_side);
//            _twist->angular.z += pd::P_control(_kp(),fr_side,br_side);

            //control adjacent distances
            _twist->angular.z = pd::P_control(_kp_double(),_bl_side,_br_side) - pd::P_control(_kp_double(),_fl_side,_fr_side);

            //_twist->angular.z = pd::P_control(_kp(),fr_side+bl_side+fr_side+br_side,fl_side+br_side+fl_side+bl_side);

        } else if (leftWallClose())
        {
            _twist->angular.z = pd::P_control(_kp_single(),_bl_side,_fl_side);

        } else if (rightWallClose())
        {
            _twist->angular.z = -pd::P_control(_kp_single(),_br_side,_fr_side);

        } else
        {
            //TODO: consider following a continued wall
            _twist->angular.z = 0;
        }
    } else
    {
        _twist->angular.z = 0;
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
