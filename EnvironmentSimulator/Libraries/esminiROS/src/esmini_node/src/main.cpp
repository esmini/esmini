#include <esmini_node.hpp>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "esmini_ros_node");

    ESMiniNode esmini_node(argc, argv);

    ros::Rate rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
};
