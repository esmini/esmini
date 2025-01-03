#include "ros/ros.h"
#include <chrono>
#include <thread>
#include "rosgraph_msgs/Clock.h"
#include "esminiLib.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clock_publisher");
    ros::NodeHandle nh("~");

    bool use_sim_time = nh.param("/use_sim_time", false);
    if (!use_sim_time)
        return 0;

    auto init_ts = std::chrono::system_clock::now();

    double                    hz = nh.param("hz", 1.);
    std::chrono::milliseconds period(static_cast<long int>(1000. / hz));
    auto                      cb        = [](const rosgraph_msgs::Clock::ConstPtr &msg) {};
    ros::Subscriber           clock_sub = nh.subscribe<rosgraph_msgs::Clock, rosgraph_msgs::Clock::ConstPtr>("/clock", 1, cb);
    ros::Publisher            clock_pub;
    rosgraph_msgs::Clock      t;

    while (ros::ok())
    {
        if (clock_sub.getNumPublishers() == 0)
            clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
        else if (clock_sub.getNumPublishers() >= 2)
            clock_pub = {};

        if (clock_pub)
        {
            auto curr_ts  = std::chrono::system_clock::now();
            auto duration = curr_ts - init_ts;
            t.clock.fromNSec(duration.count());
            clock_pub.publish(t);
        }
        std::this_thread::sleep_for(period);
    }

    return 0;
}
