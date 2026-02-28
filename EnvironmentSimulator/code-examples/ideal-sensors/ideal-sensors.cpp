#include "stdio.h"
#include "esminiLib.hpp"

#define MAX_HITS 10

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    SE_SetOption("sensors");  // enable visualization of sensor view frustums

    SE_Init("../resources/xosc/cut-in.xosc", 0, 1, 0, 0);

    int sensor_id = -1;
    sensor_id     = SE_AddObjectSensor(0, 2.0, 1.0, 0.5, 1.57, 1.0, 50.0, 1.57, MAX_HITS);
    printf("Added sensor %d to object %s\n", sensor_id, SE_GetObjectName(SE_GetId(0)));

    sensor_id = SE_AddObjectSensor(0, -1.0, 0.0, 0.5, 3.14, 0.5, 20.0, 1.57, MAX_HITS);
    printf("Added sensor %d to object %s\n", sensor_id, SE_GetObjectName(SE_GetId(0)));

    for (int i = 0; i < 2000 && !(SE_GetQuitFlag() == 1); i++)
    {
        SE_Step();

        int objList[MAX_HITS];
        for (int j = 0; j < SE_GetNumberOfObjectSensors(); j++)  // iterate over added sensors
        {
            int nHits = SE_FetchSensorObjectList(j, objList);
            for (int k = 0; k < nHits; k++)
            {
                printf("Sensor[%d] detected obj: %d\n", j, objList[k]);

                // Get some info of that detected object
                SE_ScenarioObjectState state;

                SE_GetObjectState(objList[k], &state);
                printf("object[%d] pos: (%.2f, %.2f) heading: %.2f\n", objList[k], state.x, state.y, state.h);
            }
        }
    }

    SE_Close();

    return 0;
}