#include <ros/ros.h>
#include <ros/package.h>
#include "esminiLib.hpp"

class ESMiniNode
{
private:
    ros::NodeHandle nh_;
    ros::Timer      scenario_timer_;

public:
    ESMiniNode(int argc, char* argv[]);
    ~ESMiniNode();

private:
    void timerCallback(const ros::TimerEvent& e);
};