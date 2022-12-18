#include "stdio.h"
#include "math.h"
#include <string>
#include "esminiLib.hpp"



void paramDeclCB(void* user_arg)
{
	bool ghostMode = *(static_cast<bool*>(user_arg));

	SE_LogMessage((std::string("Running with ghostMode = ").append(ghostMode == true ? "true" : "false")).c_str());
	SE_SetParameterBool("GhostMode", ghostMode);
}

int main(int argc, char* argv[])
{

	(void)argc;
	(void)argv;

	const double defaultTargetSpeed = 50.0;
	const double curveWeight = 30.0;
	const double throttleWeight = 0.1;
	const float duration = 35.0f;
	bool ghostMode[3] = { false, true, true };

	void* vehicleHandle = 0;
	SE_SimpleVehicleState vehicleState = { 0, 0, 0, 0, 0, 0, 0, 0};
	SE_ScenarioObjectState objectState;
	SE_RoadInfo roadInfo;

	for (int i = 0; i < 3; i++)
	{
		float dt = 0.0f;

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
		SE_SimpleVehicleSteeringRate(vehicleHandle, 6.0f);

		// show some road features, including road sensor
		SE_ViewerShowFeature(4 + 8, true);  // NODE_MASK_TRAIL_DOTS (1 << 2) & NODE_MASK_ODR_FEATURES (1 << 3),

		// Run for specified duration or until 'Esc' button is pressed
		while (SE_GetSimulationTime() < duration && SE_GetQuitFlag() != 1)
		{
			// Get simulation delta time since last call (first will be minimum timestep)
			dt = SE_GetSimTimeStep();

			// Get road information at a point some speed dependent distance ahead
			double targetSpeed;
			if (ghostMode[i] == true)
			{
				// ghost version
				float ghost_speed = 0.0f;

				if (i % 2 == 0)  // alternate between time based and position based look ahead modes
				{
					// Time based look ahead
					SE_GetRoadInfoGhostTrailTime(0, SE_GetSimulationTime() + 0.25f, &roadInfo, &ghost_speed);
				}
				else
				{
					// Position based Time based look ahead
					SE_GetRoadInfoAlongGhostTrail(0, 5 + 0.75f * vehicleState.speed, &roadInfo, &ghost_speed);
				}
				targetSpeed = ghost_speed;
			}
			else
			{
				// Look ahead along the road, to establish target info for the driver model
				SE_GetRoadInfoAtDistance(0, 5 + 0.75f * vehicleState.speed, &roadInfo, 0, true);

				// Slow down when curve ahead - CURVE_WEIGHT is the tuning parameter
				targetSpeed = defaultTargetSpeed / (1 + curveWeight * static_cast<double>(fabs(roadInfo.angle)));
			}

			// Steer towards where the point
			double steerAngle = roadInfo.angle;

			// Accelerate or decelerate towards target speed - THROTTLE_WEIGHT tunes magnitude
			double throttle = throttleWeight * (targetSpeed - static_cast<double>(vehicleState.speed));

			// Step vehicle model with driver input, but wait until time > 0
			if (SE_GetSimulationTime() > 0 && !SE_GetPauseFlag())
			{
				SE_SimpleVehicleControlAnalog(vehicleHandle, dt, throttle, steerAngle);
			}

			// Fetch updated state and report to scenario engine
			SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);

			// Report updated vehicle position and heading. z, pitch and roll will be aligned to the road
			SE_ReportObjectPosXYH(0, 0, vehicleState.x, vehicleState.y, vehicleState.h);

			// The following values are not necessary to report.
			// If not reported, esmini will calculate based on motion over time
			// but for accuracy it's recommendeded to report if available.

			// wheel status (revolution and steering angles)
			SE_ReportObjectWheelStatus(0, vehicleState.wheel_rotation, vehicleState.wheel_angle);

			// speed (along vehicle longitudinal (x) axis)
			SE_ReportObjectSpeed(0, vehicleState.speed);

			// Finally, update scenario using same time step as for vehicle model
			SE_StepDT(dt);
		}
		SE_Close();
	}
	return 0;
}