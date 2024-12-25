#include <ros/ros.h>
#include <ros/package.h>
#include "esmini/ObjectState.h"
#include "esmini/ObjectStates.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "esminiLib.hpp"

class ESMiniNode
{
private:
    ros::NodeHandle nh_;
    ros::Timer      scenario_timer_;

    ros::Publisher object_states_pub_;

    esmini::ObjectStates object_states_msg_;

public:
    ESMiniNode(int argc, char* argv[]);
    ~ESMiniNode();

private:
    void timerCallback(const ros::TimerEvent& e);
};