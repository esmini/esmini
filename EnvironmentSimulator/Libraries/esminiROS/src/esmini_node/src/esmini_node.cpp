#include <esmini_node.h>

ESMiniNode::ESMiniNode(int argc, char *argv[]) : nh_("~")
{
    std::string osc_file_;
    nh_.param("osc", osc_file_, ros::package::getPath("esmini") + "/../../../../../resources/xosc/cut-in.xosc");
    SE_Init(osc_file_.c_str(), 0, 1, 0, 0);

    scenario_timer_ = nh_.createTimer(ros::Duration(0.01), &ESMiniNode::timerCallback, this);
}

ESMiniNode::~ESMiniNode()
{
}

void ESMiniNode::timerCallback(const ros::TimerEvent &e)
{
    SE_Step();
}