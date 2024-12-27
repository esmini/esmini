#include <esmini_node.h>

ESMiniNode::ESMiniNode(int argc, char *argv[]) : nh_("~")
{
    std::string osc_file_;
    nh_.param("osc", osc_file_, ros::package::getPath("esmini") + "/../../../../../resources/xosc/cut-in.xosc");
    SE_Init(osc_file_.c_str(), 0, 1, 0, 0);

    object_states_msg_.header.frame_id = "map";

    if ((ego_id_ = SE_GetIdByName("Ego")) == -1)
    {
        ROS_ERROR("No ego exists!");
        return;
    }

    SE_ScenarioObjectState ego_state;
    SE_GetObjectState(ego_id_, &ego_state);
    ego_handle_ = SE_SimpleVehicleCreate(ego_state.x, ego_state.y, ego_state.h, ego_state.length, ego_state.speed);

    accel_sub_    = nh_.subscribe<std_msgs::Float64>("/esmini/accel", 1, &ESMiniNode::accelCallback, this, ros::TransportHints().tcpNoDelay());
    steering_sub_ = nh_.subscribe<std_msgs::Float64>("/esmini/steering", 1, &ESMiniNode::steeringCallback, this, ros::TransportHints().tcpNoDelay());
    object_states_pub_ = nh_.advertise<esmini::ObjectStates>("/esmini/object_states", 1, true);

    scenario_timer_ = nh_.createTimer(ros::Duration(0.01), &ESMiniNode::timerCallback, this);
}

ESMiniNode::~ESMiniNode()
{
}

void ESMiniNode::timerCallback(const ros::TimerEvent &e)
{
    SE_ScenarioObjectState ego_state;
    SE_GetObjectState(ego_id_, &ego_state);
    SE_SimpleVehicleSetSpeed(ego_handle_, ego_state.speed);
    SE_SimpleVehicleControlAnalog(ego_handle_, 0.01, accel_, steering_);
    SE_SimpleVehicleGetState(ego_handle_, &ego_state_);
    SE_ReportObjectSpeed(ego_id_, ego_state_.speed + accel_ * SE_GetSimTimeStep());
    SE_ReportObjectPosXYH(ego_id_, 0, ego_state_.x, ego_state_.y, ego_state_.h);
    SE_ReportObjectWheelStatus(0, ego_state_.wheel_rotation, ego_state_.wheel_angle);

    SE_Step();

    object_states_msg_.header.stamp = ros::Time(SE_GetSimulationTime());
    object_states_msg_.states.clear();

    for (int i = 0; i < SE_GetNumberOfObjects(); i++)
    {
        SE_ScenarioObjectState state;

        SE_GetObjectState(SE_GetId(i), &state);

        // Skip for TYPE_NONE
        if (state.objectType == 0)
            continue;

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
        obj_state_msg.speed            = state.speed;
        obj_state_msg.length           = state.length;
        obj_state_msg.width            = state.width;
        obj_state_msg.height           = state.height;

        obj_state_msg.wheel_angle = state.wheel_angle;

        object_states_msg_.states.push_back(obj_state_msg);
    }

    object_states_pub_.publish(object_states_msg_);
}

void ESMiniNode::accelCallback(const std_msgs::Float64::ConstPtr &msg)
{
    if (msg->data > 0)
    {
        accel_ = msg->data / MAX_ACC_DEFAULT;
    }
    else if (msg->data < 0)
    {
        accel_ = msg->data / MAX_DEC_DEFAULT;
    }
    else
    {
        accel_ = 0;
    }
}

void ESMiniNode::steeringCallback(const std_msgs::Float64::ConstPtr &msg)
{

    steering_ = msg->data / STEERING_MAX_ANGLE;
}