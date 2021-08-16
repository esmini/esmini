#include "esminiLib.hpp"

#include "osi_common.pb.h"
#include "osi_object.pb.h"
#include "osi_groundtruth.pb.h"
#include "osi_version.pb.h"

int main(int argc, char* argv[])
{
	SE_Init("../resources/xosc/cut-in_simple.xosc", 0, 1, 0, 0);

	osi3::GroundTruth* gt;

	// Initial update of complete Ground Truth, including static things
	SE_UpdateOSIGroundTruth();
	// You could now retrieve the initial state of all objects before stepping the scenario

	for (int i = 0; i < 1500; i++)
	{
		SE_StepDT(0.01f);

		// Further updates will only affect dynamic OSI stuff
		SE_UpdateOSIGroundTruth();

		// Fetch OSI struct
		gt = (osi3::GroundTruth*)SE_GetOSIGroundTruthRaw();

		// Print timestamp
		printf("Frame %d timestamp: %.2f\n", i, gt->mutable_timestamp()->seconds() +
			1E-9 * gt->mutable_timestamp()->nanos());

		// Lane boundaries
		printf("lane boundaries: %d\n", gt->lane_boundary_size());
		for (int j = 0; j < gt->lane_boundary_size(); j++)
		{
			printf("  lane boundary %d, nr of boundary points: %d\n", j, gt->lane_boundary(j).boundary_line_size());

#if 0  // change to 1 in order to print all boundary points
			for (int k = 0; k < gt->lane_boundary(j).boundary_line_size(); k++)
			{
				printf("    (%.2f, %.2f)\n",
					gt->lane_boundary(j).boundary_line(k).position().x(),
					gt->lane_boundary(j).boundary_line(k).position().y());
			}
#endif
		}

		// Road markings, e.g. zebra lines
		printf("road markings: %d\n", gt->road_marking_size());

		// Moving objects
		printf("moving objects: %d\n", gt->moving_object_size());

#if 1  // change to 1 in order to print some moving object state info
		// Print object id, position, orientation and velocity
		for (int j = 0; j < gt->mutable_moving_object()->size(); j++)
		{
			printf("  obj id %lld pos (%.2f, %.2f, %.2f) orientation (%.2f, %.2f, %.2f) vel (%.2f, %.2f, %.2f) acc (%.2f, %.2f, %.2f)\n",
				gt->mutable_moving_object(j)->mutable_id()->value(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_position()->x(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_position()->y(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_position()->z(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_orientation()->yaw(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_orientation()->pitch(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_orientation()->roll(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_velocity()->x(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_velocity()->y(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_velocity()->z(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_acceleration()->x(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_acceleration()->y(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_acceleration()->z()
			);
		}
#endif
	}
	return 0;
}
