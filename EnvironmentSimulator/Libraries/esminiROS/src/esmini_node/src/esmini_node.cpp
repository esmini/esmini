#include <esmini_node.h>

ESMiniNode::ESMiniNode(int argc, char *argv[]) : nh_("~")
{
    std::string osc_file_;
    nh_.param("osc", osc_file_, ros::package::getPath("esmini") + "/../../../../../resources/xosc/cut-in.xosc");
    SE_Init(osc_file_.c_str(), 0, 1, 0, 0);

    scenario_timer_ = nh_.createTimer(ros::Duration(0.01), &ESMiniNode::timerCallback, this);

    object_states_msg_.header.frame_id = "map";

    object_states_pub_ = nh_.advertise<esmini::ObjectStates>("/esmini/object_states", 1, true);
}

ESMiniNode::~ESMiniNode()
{
}

void ESMiniNode::timerCallback(const ros::TimerEvent &e)
{
    SE_Step();

    object_states_msg_.header.stamp = ros::Time(SE_GetSimulationTime());
    object_states_msg_.states.clear();

    for (int i = 0; i < SE_GetNumberOfObjects(); i++)
    {
        SE_ScenarioObjectState state;

        SE_GetObjectState(SE_GetId(i), &state);

        // Skip for TYPE_NONE
        if(state.objectType == 0) continue;

        esmini::ObjectState obj_state_msg;
        obj_state_msg.id   = state.id;
        obj_state_msg.name = SE_GetObjectName(state.id);

        obj_state_msg.type = state.objectType;

        obj_state_msg.pose.position.x = state.x;
        obj_state_msg.pose.position.y = state.y;
        obj_state_msg.pose.position.z = state.z;

        tf2::Quaternion q;
        q.setRPY(state.r, state.p, state.h);
        obj_state_msg.pose.orientation = tf2::toMsg(q);

        obj_state_msg.speed = state.speed;
        obj_state_msg.length = state.length;
        obj_state_msg.width = state.width;
        obj_state_msg.height = state.height;

        obj_state_msg.wheel_angle = state.wheel_angle;

        object_states_msg_.states.push_back(obj_state_msg);
    }

    object_states_pub_.publish(object_states_msg_);
}