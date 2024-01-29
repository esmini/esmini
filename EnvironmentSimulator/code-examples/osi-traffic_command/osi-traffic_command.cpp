#include "esminiLib.hpp"

#include "osi_common.pb.h"
#include "osi_object.pb.h"
#include "osi_groundtruth.pb.h"
#include "osi_trafficcommand.pb.h"
#include "osi_version.pb.h"

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;
    const osi3::TrafficCommand* tc;

    SE_EnableOSIFile(0);  // 0 or "" will result in default filename, ground_truth.osi

    if (SE_Init("../resources/xosc/lane_change_simple.xosc", 0, 1, 0, 0) != 0)
    {
        printf("SE_Init failed\n");
        return -1;
    }

    // for (int i = 0; i < 100; i++)
    while (SE_GetQuitFlag() == 0)
    {
        SE_StepDT(0.1f);

        SE_UpdateOSITrafficCommand();

        // Fetch OSI struct (via pointer, no copying of data)
        tc = reinterpret_cast<const osi3::TrafficCommand*>(SE_GetOSITrafficCommandRaw());

        for (int j = 0; j < tc->action().size(); j++)
        {
            if (tc->action(j).has_lane_change_action())
            {
                printf("Lane change action (id %u) started: objId %d deltaLane %d shapeType %d duration %.2f distance %.2f\n",
                       static_cast<unsigned int>(tc->action(j).lane_change_action().action_header().action_id().value()),
                       static_cast<unsigned int>(tc->traffic_participant_id().value()),
                       tc->action(j).lane_change_action().relative_target_lane(),
                       static_cast<int>(tc->action(j).lane_change_action().dynamics_shape()),
                       tc->action(j).lane_change_action().duration(),
                       tc->action(j).lane_change_action().distance());
            }

            if (tc->action(j).has_speed_action())
            {
                printf("Speed action (id %u) started: objId %d targetSpeed %.2f shape %d duration %.2f distance %.2f\n",
                       static_cast<unsigned int>(tc->action(j).speed_action().action_header().action_id().value()),
                       static_cast<unsigned int>(tc->traffic_participant_id().value()),
                       tc->action(j).speed_action().absolute_target_speed(),
                       static_cast<int>(tc->action(j).speed_action().dynamics_shape()),
                       tc->action(j).speed_action().duration(),
                       tc->action(j).speed_action().distance());
            }

            if (tc->action(j).has_teleport_action())
            {
                printf("Teleport action (id %u) started: objId %d x %.2f y %.2f z %.2f h %.2f p %.2f r %.2f\n",
                       static_cast<unsigned int>(tc->action(j).teleport_action().action_header().action_id().value()),
                       static_cast<unsigned int>(tc->traffic_participant_id().value()),
                       tc->action(j).teleport_action().position().x(),
                       tc->action(j).teleport_action().position().y(),
                       tc->action(j).teleport_action().position().z(),
                       tc->action(j).teleport_action().orientation().yaw(),
                       tc->action(j).teleport_action().orientation().pitch(),
                       tc->action(j).teleport_action().orientation().roll());
            }
        }
    }

    SE_Close();

    return 0;
}
