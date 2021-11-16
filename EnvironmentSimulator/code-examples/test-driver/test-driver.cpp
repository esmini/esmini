#include "stdio.h"
#include "math.h"
#include <string>
#include "esminiLib.hpp"

#define TARGET_SPEED 50.0
#define CURVE_WEIGHT 30.0
#define THROTTLE_WEIGHT 0.01
#define DURATION 30

void paramDeclCB(void* user_arg)
{
	bool ghostMode = *((bool*)user_arg);

	SE_LogMessage((std::string("Running with ghostMode = ").append(ghostMode == true ? "true" : "false")).c_str());
	SE_SetParameterBool("GhostMode", ghostMode);
}

int main(int argc, char* argv[])
{
	void* vehicleHandle = 0;
	SE_SimpleVehicleState vehicleState = { 0, 0, 0, 0, 0, 0 };
	SE_ScenarioObjectState objectState;
	SE_RoadInfo roadInfo;
	bool ghostMode[2] = { false, true };

	for (int i = 0; i < 2; i++)
	{
		float simTime = 0;
		float dt = 0;

		SE_RegisterParameterDeclarationCallback(paramDeclCB, &ghostMode[i]);

		if (SE_Init("../EnvironmentSimulator/code-examples/test-driver/test-driver.xosc", 0, 1, 0, 0) != 0)
		{
			SE_LogMessage("Failed to initialize the scenario, quit\n");
			return -1;
		}

		// Lock object to the original lane
		// If setting to false, the object road position will snap to closest lane
		SE_SetLockOnLane(0, true);

		// Initialize the vehicle model, fetch initial state from the scenario
		SE_GetObjectState(0, &objectState);
		vehicleHandle = SE_SimpleVehicleCreate(objectState.x, objectState.y, objectState.h, 4.0, 0.0);

		// show some road features, including road sensor
		SE_ViewerShowFeature(4 + 8, true);  // NODE_MASK_TRAIL_DOTS (1 << 2) & NODE_MASK_ODR_FEATURES (1 << 3),

		// Run for specified duration or until 'Esc' button is pressed
		while (SE_GetSimulationTime() < DURATION && SE_GetQuitFlag() != 1)
		{
			// Get simulation delta time since last call (first will be 0)
			dt = SE_GetSimTimeStep();

			// Get road information at a point some speed dependent distance ahead
			double targetSpeed;
			if (ghostMode[i] == true)
			{
				// ghost version
				float ghost_speed;
				SE_GetRoadInfoAlongGhostTrail(0, 5 + 0.75f * vehicleState.speed, &roadInfo, &ghost_speed);
				targetSpeed = ghost_speed;

			}
			else
			{
				// Look ahead along the road, to establish target info for the driver model
				SE_GetRoadInfoAtDistance(0, 5 + 0.75f * vehicleState.speed, &roadInfo, 0, true);

				// Slow down when curve ahead - CURVE_WEIGHT is the tuning parameter
				targetSpeed = TARGET_SPEED / (1 + CURVE_WEIGHT * fabs(roadInfo.angle));
			}

			// Steer towards where the point
			double steerAngle = roadInfo.angle;

			// Accelerate or decelerate towards target speed - THROTTLE_WEIGHT tunes magnitude
			double throttle = THROTTLE_WEIGHT * (targetSpeed - vehicleState.speed);

			// Step vehicle model with driver input, but wait until time > 0
			if (SE_GetSimulationTime() > 0)
			{
				SE_SimpleVehicleControlAnalog(vehicleHandle, dt, throttle, steerAngle);
			}

			// Fetch updated state and report to scenario engine
			SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);

			// Report updated vehicle position and heading. z, pitch and roll will be aligned to the road
			SE_ReportObjectPosXYH(0, simTime, vehicleState.x, vehicleState.y, vehicleState.h, vehicleState.speed);

			// Finally, update scenario using same time step as for vehicle model
			SE_StepDT(dt);
		}
	}
	return 0;
}