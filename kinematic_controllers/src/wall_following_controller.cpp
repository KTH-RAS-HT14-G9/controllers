#include <kinematic_controllers/wall_following_controller.h>
#include <pid.h>


WallFollowingController::WallFollowingController(ros::NodeHandle &handle, 
                                                 double update_frequency)
    :ControllerBase(handle, update_frequency)
    ,_kp("/controller/wall_follow/kp", 3.0)
    ,wall_th("/controller/wall_follow/wall_th", 0.40)
    ,_active(false)
    ,_ch1(0)
    ,_ch2(0)
    ,_ch3(0)
    ,_ch4(0)
    ,fl_side(0)
    ,fr_side(0)
    ,bl_side(0)
    ,br_side(0)
    ,_twist(new geometry_msgs::Twist)
{
    _sub_sens = _handle.subscribe("/arduino/adc", 10,
                                  &WallFollowingController::callback_sensors, this);
    _sub_act = _handle.subscribe("/controller/wall_follow/active", 10,
                                 &WallFollowingController::callback_activate, this);
}

WallFollowingController::~WallFollowingController()
{}

void WallFollowingController::callback_sensors(const ras_arduino_msgs::ADConverter lecture) {
    _ch1 = lecture.ch1;
    _ch2 = lecture.ch2;
    _ch3 = lecture.ch3;
    _ch4 = lecture.ch4;
    fl_side = robot::ir::distance(1,_ch1) + robot::ir::offset_front_left;
    fr_side = robot::ir::distance(2,_ch2) + robot::ir::offset_front_right;
    bl_side = robot::ir::distance(3,_ch3) + robot::ir::offset_rear_left;
    br_side = robot::ir::distance(4,_ch4) + robot::ir::offset_rear_right;
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
        if(bothWallsClose())
        {
            //_twist->angular.z = pd::P_control(_kp(),fr_side+bl_side,fl_side+br_side);
            _twist->angular.z = pd::P_control(_kp(),fr_side+bl_side+fr_side+br_side,fl_side+br_side+fl_side+bl_side);

        } else if (leftWallClose())
        {
            _twist->angular.z = pd::P_control(_kp(),bl_side,fl_side);

        } else if (rightWallClose())
        {
            _twist->angular.z = -pd::P_control(_kp(),br_side,fr_side);

        } else
        {
            _twist->angular.z = 0;
        }
    } else
    {
        _twist->angular.z = 0;
    }

    return _twist;
}

bool WallFollowingController::bothWallsClose()
{
    return leftWallClose() && rightWallClose();
}

bool WallFollowingController::leftWallClose()
{
    return fl_side<wall_th() && bl_side<wall_th();
}

bool WallFollowingController::rightWallClose()
{
    return fr_side<wall_th() && br_side<wall_th();
}
