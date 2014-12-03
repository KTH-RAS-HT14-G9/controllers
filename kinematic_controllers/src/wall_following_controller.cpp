#include <kinematic_controllers/wall_following_controller.h>
#include <pid.h>


WallFollowingController::WallFollowingController(ros::NodeHandle &handle,
                                                 double update_frequency)
    :ControllerBase(handle, update_frequency)
    ,_kp_to_wall("/controller/wall_follow/kp/to_wall", 2.0)
    ,_kp_from_wall("/controller/wall_follow/kp/from_wall", 5.0)
    ,_kp_angular("/controller/wall_follow/kp/angular", 3.0)
    ,_wall_th("/controller/wall_follow/wall_th", 0.40)
    ,_wall_target_dist("/controller/wall_follow/_wall_target_dist", 0.18)
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
            _twist->angular.z = pd::P_control(_kp_angular(),_bl_side,_br_side) - pd::P_control(_kp_angular(),_fl_side,_fr_side);

        } else
        {
            if (leftWallClose())
            {
                double dist = std::min(_bl_side,_fl_side);
                if(dist > _wall_target_dist())
                {
                    _twist->angular.z += pd::P_control(_kp_angular(),_bl_side, _fl_side) - pd::P_control(_kp_to_wall(),dist, _wall_target_dist());
                } else {
                    _twist->angular.z += pd::P_control(_kp_angular(),_bl_side, _fl_side) - pd::P_control(_kp_from_wall(),dist, _wall_target_dist());
                }
            }
            if (rightWallClose())
            {
                double dist = std::min(_br_side,_fr_side);
                if(dist > _wall_target_dist())
                {
                    _twist->angular.z += -pd::P_control(_kp_angular(),_br_side,_fr_side) + pd::P_control(_kp_to_wall(),dist, _wall_target_dist());
                } else {
                    _twist->angular.z += -pd::P_control(_kp_angular(),_br_side,_fr_side) + pd::P_control(_kp_from_wall(),dist, _wall_target_dist());
                }
            }
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
