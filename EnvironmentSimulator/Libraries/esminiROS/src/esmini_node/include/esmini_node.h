#include <ros/ros.h>
#include <ros/package.h>
#include "esmini/ObjectState.h"
#include "esmini/ObjectStates.h"
#include "std_msgs/Float64.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "esminiLib.hpp"

#define STEERING_MAX_ANGLE (60 * M_PI / 180)
#define MAX_ACC_DEFAULT    20.0
#define MAX_DEC_DEFAULT    20.0

class ESMiniNode
{
private:
    ros::NodeHandle nh_;
    ros::Timer      scenario_timer_;

    ros::Publisher  object_states_pub_;
    ros::Subscriber accel_sub_;
    ros::Subscriber steering_sub_;

    esmini::ObjectStates object_states_msg_;

    int    ego_id_;
    double ego_speed_;
    double ego_x_, ego_y_, ego_h_;

    double accel_{0.0}, steering_{0.0};

    void*                 ego_handle_;
    SE_SimpleVehicleState ego_state_{0, 0, 0, 0, 0, 0, 0, 0};

public:
    ESMiniNode(int argc, char* argv[]);
    ~ESMiniNode();

private:
    void timerCallback(const ros::TimerEvent& e);
    void accelCallback(const std_msgs::Float64::ConstPtr& msg);
    void steeringCallback(const std_msgs::Float64::ConstPtr& msg);
};