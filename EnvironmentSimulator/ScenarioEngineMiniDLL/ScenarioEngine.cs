using UnityEngine;
using System.Collections.Generic;
using System.Runtime.InteropServices;

// Note: Hardcoded objects, only supporting scenarios with two objects with id 0 and 1

// A state struct may look something like this - not used yet
public struct ScenarioObjectState
{
    public int id;
    public string name;
    public double timestamp;
    public double x;
    public double y;
    public double z;
    public double h;
    public double p;
    public double r;
    public double speed;
};

public class ScenarioEngine : MonoBehaviour
{
    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_Init")]
    public static extern int SE_Init(string oscFilename);

    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_Close")]
    public static extern void SE_Close();

    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_Step")]
    public static extern void SE_Step(float dt);

    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_ReportObjectPos")]
    public static extern int SE_ReportObjectPos(int id, string name, float timestamp, float x, float y, float z, float h, float p, float r, double speed);

    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_ReportObjectRoadPos")]
    public static extern int SE_ReportObjectRoadPos(int id, string name, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed);

    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_GetNumberOfObjects")]
    public static extern int SE_GetNumberOfObjects();

    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_GetObjectState")]
    public static extern ScenarioObjectState SE_GetObjectState(int index);

    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_GetObjectX")]
    public static extern float SE_GetObjectX(int index);

    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_GetObjectY")]
    public static extern float SE_GetObjectY(int index);

    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_GetObjectZ")]
    public static extern float SE_GetObjectZ(int index);

    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_GetObjectH")]
    public static extern float SE_GetObjectH(int index);

    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_GetObjectP")]
    public static extern float SE_GetObjectP(int index);

    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_GetObjectR")]
    public static extern float SE_GetObjectR(int index);

    public string scenarioFile;
    public Camera scenarioCamera;
    private GameObject ego;
    private List<GameObject> cars = new List<GameObject>();
    private List<string> objectNames = new List<string>
        {
            "car1",
            "car4",
            "car2",
            "car3",
            "car5",
            "car6",
            "bus1",
            "bus2",
        };

    private void Start()
    {
        int returnvalue = SE_Init(scenarioFile);

        if (returnvalue != 0)
        {
            print("failed to load " + scenarioFile);
            return;
        }

        // Instantiate objects
        for (int i = 0; i < SE_GetNumberOfObjects(); i++)
        {
            print("Adding " + objectNames[i % objectNames.Count]);
            cars.Add((GameObject)Instantiate(Resources.Load(objectNames[i % objectNames.Count])));
        }

        if (SE_GetNumberOfObjects() > 0)
        {
            // Let camera follow first object - assumed to be the Ego vehicle
            scenarioCamera.transform.parent = cars[0].transform;
        }
    }


    private void Update()
    {
        float x, y, z, x_rot, y_rot, z_rot;
        int i = 0;

        SE_Step(Time.deltaTime);

        // Check nr of objects
        foreach (GameObject car in cars)
        {
            // Adapt to Unity coordinate system
            x = -SE_GetObjectY(i);
            y = SE_GetObjectZ(i);
            z = SE_GetObjectX(i);
            car.transform.position = new Vector3(x, y, z);

            y_rot = -SE_GetObjectH(i) * 180.0f / Mathf.PI;
            x_rot = -SE_GetObjectP(i) * 180.0f / Mathf.PI;
            z_rot = SE_GetObjectR(i) * 180.0f / Mathf.PI;
            Quaternion rotation = Quaternion.Euler(x_rot, y_rot, z_rot);
            car.transform.rotation = rotation;

            i++;
        }
    }
}
