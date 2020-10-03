
#define USE_STATE_REF 

using UnityEngine;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System;
using System.IO;
using UnityEngine.UI;


// Note: Hardcoded objects, only supporting scenarios with two objects with id 0 and 1

// A state struct may look something like this - not used yet

[StructLayout(LayoutKind.Sequential)]
public struct ScenarioObjectState
{
    public int id;          // Automatically generated unique object id 
    public int model_id;    // Id to control what 3D model to represent the vehicle - see carModelsFiles_[] in scenarioenginedll.cpp
    public int control;     // 0: DefaultController 1: External. Further values see esmini/EnvironmentSimulator/Controllers/Controller::Type enum
    public float timestamp;
    public float x;
    public float y;
    public float z;
    public float h;
    public float p;
    public float r;
    public int roadId;
    public float t;
    public int laneId;
    public float laneOffset;
    public float s;
    public float speed;
    public float centerOffsetX;
    public float centerOffsetY;
    public float centerOffsetZ;
    public float width;
    public float length;
    public float height;
};

[StructLayout(LayoutKind.Sequential)]
public struct RoadInfo
{
    public float global_pos_x;     // steering target position, in global coordinate system
    public float global_pos_y;     // steering target position, in global coordinate system
    public float global_pos_z;     // steering target position, in global coordinate system
    public float local_pos_x;      // steering target position, relative vehicle (pivot position object) coordinate system
    public float local_pos_y;      // steering target position, relative vehicle (pivot position object) coordinate system
    public float local_pos_z;      // steering target position, relative vehicle (pivot position object) coordinate system
    public float angle;            // heading angle to steering target from and relatove to vehicle (pivot position)
    public float road_heading;     // road heading at steering target point
    public float road_pitch;       // road pitch (inclination) at steering target point
    public float road_roll;        // road roll (camber) at steering target point
    public float trail_heading;	   // trail heading (only when used for trail lookups, else equals road_heading)
    public float curvature;        // road curvature at steering target point
    public float speed_limit;	   // speed limit given by OpenDRIVE type entry
};

[StructLayout(LayoutKind.Sequential)]
public struct SE_LaneInfo
{
    public int far_left_lb_id;
    public int left_lb_id;
    public int right_lb_id;
    public int far_right_lb_id;
};

public class ScenarioEngine : MonoBehaviour
{
    [DllImport("esminiLib", EntryPoint = "SE_Init")]
    /// <summary>Initialize the scenario engine</summary>
    /// <param name="oscFilename">Path to the OpenSCEANRIO file</param>
    /// <param name="control">Ego control 0=by OSC 1=Internal 2=External 3=Hybrid</param>
    /// <param name="use_viewer">0=no viewer, 1=use viewer</param>
    /// <param name="record">Create recording for later playback 0=no recording 1=recording</param>
    /// <param name="headstart_time">For hybrid control mode launch ghost vehicle with this headstart time</param>
    /// <returns>0 on success, -1 on failure for any reason</returns>
    public static extern int SE_Init(string oscFilename, int disable_ctrls = 0, int use_viewer = 0, int threading = 0, int record = 0);

    [DllImport("esminiLib", EntryPoint = "SE_StepDT")]
    public static extern int SE_StepDT(float dt);

    [DllImport("esminiLib", EntryPoint = "SE_Step")]
    public static extern int SE_Step();

    [DllImport("esminiLib", EntryPoint = "SE_Close")]
    public static extern void SE_Close();

    [DllImport("esminiLib", EntryPoint = "SE_GetSimulationTime")]
    public static extern float SE_GetSimulationTime();

    [DllImport("esminiLib", EntryPoint = "SE_ReportObjectPos")]
    public static extern int SE_ReportObjectPos(int id, float timestamp, float x, float y, float z, float h, float p, float r, float speed);

    [DllImport("esminiLib", EntryPoint = "SE_ReportObjectRoadPos")]
    public static extern int SE_ReportObjectRoadPos(int id, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed);

    [DllImport("esminiLib", EntryPoint = "SE_GetNumberOfObjects")]
    public static extern int SE_GetNumberOfObjects();

    [DllImport("esminiLib", EntryPoint = "SE_GetObjectState")]
    public static extern int SE_GetObjectState(int index, ref ScenarioObjectState state);

    [DllImport("esminiLib", EntryPoint = "SE_GetObjectGhostState")]
    public static extern int SE_GetObjectGhostState(int index, ref ScenarioObjectState state);

    [DllImport("esminiLib", EntryPoint = "SE_ObjectHasGhost")]
    public static extern int SE_ObjectHasGhost(int index);

    /// <summary>Create an ideal object sensor and attach to specified vehicle</summary>
    /// <param name="object_id">Handle to the object to which the sensor should be attached</param>
    /// <param name="x">Position x coordinate of the sensor in vehicle local coordinates</param>
    /// <param name="y">Position y coordinate of the sensor in vehicle local coordinates</param>
    /// <param name="z">Position z coordinate of the sensor in vehicle local coordinates</param>
    /// <param name="h">Position heading of the sensor in vehicle local coordinates</param>
    /// <param name="fovH">Horizontal field of view, in degrees</param>
    /// <param name="rangeNear">Near value of the sensor depth range</param>
    /// <param name="rangeFar">Far value of the sensor depth range</param>
    /// <param name="maxObj"> Maximum number of objects theat the sensor can track</param>
    /// <returns>0 on success, -1 on failure for any reason</returns>
    [DllImport("esminiLib", EntryPoint = "SE_AddObjectSensor")]
    public static extern int SE_AddObjectSensor(int object_id, float x, float y, float z, float h, float rangeNear, float rangeFar, float fovH, int maxObj);

    /// <summary>Fetch list of identified objects from a sensor</summary>
    /// <param name="object_id">Handle to the object to which the sensor should is attached</param>
    /// <param name="list">Array of object indices</param>
    /// <returns> Number of identified objects, i.e.length of list. -1 on failure</returns>
    [DllImport("esminiLib", EntryPoint = "SE_FetchSensorObjectList")]
    public static extern int SE_FetchSensorObjectList(int object_id, int[] list);


    /// <summary>Get information suitable for driver modeling of a point at a specified distance from object along the road ahead</summary>
    /// <param name="object_id">Handle to the position object from which to measure</param>
    /// <param name="lookahead_distance">The distance, along the road, to the point</param>
    /// <param name="data">Struct including all result values, see typedef for details</param>
    /// <param name="along_road_center">Measure along the reference lane, i.e. at center of the road. Should be false for normal use cases</param>
    /// <returns>0 on success, -1 on failure for any reason</returns>
    [DllImport("esminiLib", EntryPoint = "SE_GetRoadInfoAtDistance")]
    public static extern int SE_GetRoadInfoAtDistance(int object_id, float lookahead_distance, ref RoadInfo data, int along_road_center);

    /// <summary>Get information suitable for driver modeling of a ghost vehicle driving ahead of the ego vehicle</summary>
    /// <param name="object_id">Handle to the position object from which to measure</param>
    /// <param name="lookahead_distance">The distance, along the road, to the point</param>
    /// <param name="data">Struct including all result values, see typedef for details</param>
    /// <param name="speed_ghost">Speed of the ghost vehicle at specifed point</param>
    /// <returns>0 on success, -1 on failure for any reason</returns>
    [DllImport("esminiLib", EntryPoint = "SE_GetRoadInfoAlongGhostTrail")]
    public static extern int SE_GetRoadInfoAlongGhostTrail(int object_id, float lookahead_distance, ref RoadInfo data, ref float speed_ghost);

    private string scenarioFolder;
    public Button btn_cutin, btn_cutin_ego, btn_ltapod, btn_ltapod_ego, btn_basic_ego_internal, btn_basic_ego_external, btn_basic_ego_hybrid, btn_two_ghosts;
    public Toggle toggle_osg;
    public float tension = 10.0f;
    public bool osg_viewer = false;
    private Rigidbody egoBody;
    private float simTime;
    private int car_in_focus_ = 0;

    private Camera _cam;
    private GameObject camTarget;
    private GameObject envModel;
    private bool scenarioLoaded = false;
    private float speed = 0.0f;
    private bool interactive_ = false;
    private int[] objList;
    ScenarioObjectState state;
    private List<GameObject> cars = new List<GameObject>();
    private List<string> objectNames = new List<string>
        {
            "car1",
            "car2",
            "car3",
            "car4",
            "car5",
            "car6",
            "bus1",
            "S90_dummy",
            "bus2",
        };

    private void Start()
    {
        Time.fixedDeltaTime = 0.017f;

        scenarioFolder = Application.streamingAssetsPath + "/xosc/";

        GameObject original = GameObject.FindWithTag("MainCamera");
        _cam = (Camera)Camera.Instantiate(original.GetComponent<Camera>());
        Destroy(original);

        camTarget = new GameObject("camTarget");

        Button btn1 = btn_cutin.GetComponent<Button>();
        Button btn1b = btn_cutin_ego.GetComponent<Button>();
        Button btn2 = btn_ltapod.GetComponent<Button>();
        Button btn3 = btn_ltapod_ego.GetComponent<Button>();
        Button btn4 = btn_basic_ego_internal.GetComponent<Button>();
        Button btn5 = btn_basic_ego_external.GetComponent<Button>();
        Toggle t_osg = toggle_osg.GetComponent<Toggle>();

        //Calls the TaskOnClick/TaskWithParameters method when you click the Button
        btn1.onClick.AddListener(delegate {
            InitScenario(scenarioFolder + Path.DirectorySeparatorChar + "cut-in.xosc", "e6mini", 1, t_osg.isOn, false);
        });
        btn1b.onClick.AddListener(delegate {
            InitScenario(scenarioFolder + Path.DirectorySeparatorChar + "cut-in.xosc", "e6mini", 0, t_osg.isOn, true);
        });
        btn2.onClick.AddListener(delegate {
            InitScenario(scenarioFolder + Path.DirectorySeparatorChar + "ltap-od.xosc", "fabriksg", 1, t_osg.isOn, false);
        });
        btn3.onClick.AddListener(delegate {
            InitScenario(scenarioFolder + Path.DirectorySeparatorChar + "ltap-od.xosc", "fabriksg", 0, t_osg.isOn, true);
        });
        btn4.onClick.AddListener(delegate {
            InitScenario(scenarioFolder + Path.DirectorySeparatorChar + "basic_ghost.xosc", "e6mini", 0, t_osg.isOn, false);
        });
        btn5.onClick.AddListener(delegate {
            InitScenario(scenarioFolder + Path.DirectorySeparatorChar + "basic_ghost.xosc", "e6mini", 1, t_osg.isOn, false);
        });
    }

    void OnApplicationQuit()
    {
        SE_Close();
    }

    private Vector3 RH2Unity(Vector3 rightHandVector)
    {
        return new Vector3(-rightHandVector.y, rightHandVector.z, rightHandVector.x);
    }

    private Vector3 Unity2RH(Vector3 unityVector)
    {
        return new Vector3(unityVector.z, -unityVector.x, unityVector.y);
    }

    private Vector3 RHHPR2UnityXYZ(Vector3 rightHandRotation)
    {
        return new Vector3(180.0f * rightHandRotation.y / Mathf.PI, -180.0f * rightHandRotation.x / Mathf.PI, -180.0f * rightHandRotation.z / Mathf.PI);
    }

    private Vector3 UnityXYZ2RHHPR(Vector3 UnityXYZ)
    {
        return new Vector3(-Mathf.PI * UnityXYZ.y / 180.0f, Mathf.PI * UnityXYZ.x / 180.0f, -Mathf.PI * UnityXYZ.z / 180.0f);
    }

    private void InitScenario(string scenarioFile, string modelName, int disable_controller, bool osg_viewer, bool interactive)
    {
        print("Init scenario " + scenarioFile);
        scenarioLoaded = false;
        speed = 0;
        interactive_ = interactive;
        simTime = 0;

        // Detach camera from any previous parent, then init its transform
        camTarget.transform.parent = null;
        _cam.transform.parent = null;

        // First release any previously loaded objects
        foreach (GameObject car in cars)
        {
            Destroy(car);
        }
        cars.Clear();

        if (envModel != null)
        {
            Destroy(envModel);
        }

        // Then load the requested scenario
        if (SE_Init(scenarioFile, disable_controller, osg_viewer ? 1 : 0, 0, 0) != 0)
        {
            print("failed to load " + scenarioFile);
            return;
        }

        // Add sensors
        if (SE_AddObjectSensor(0, 4, 0, 0.5f, 0.0f, 5.0f, 50.0f, (float)(50 * Math.PI / 180.0), 10) != 0)
        {
            print("failed to add sensor");
            return;
        }
        objList = new int[10];
        state = new ScenarioObjectState();

        // Load Ego
        if (interactive_)
        {
            cars.Add((GameObject)Instantiate(Resources.Load("S90")));
            egoBody = cars[0].GetComponent<Rigidbody>();
        }
        else
        {
            cars.Add((GameObject)Instantiate(Resources.Load("S90_dummy")));
        }
        print("Adding Ego");


        if (SE_GetNumberOfObjects() > 0)
        {
            AttachCameraToObj(0);
        }

        // Load environment 3D model
        envModel = (GameObject)Instantiate(Resources.Load(modelName));

        // Fetch the initial object positions
        UpdateObjectPositions(true);

        if (SE_GetNumberOfObjects() > 0)
        {
            // Initialize camera position to the exact target point
            _cam.transform.position = camTarget.transform.position;
        }

        scenarioLoaded = true;
    }

    private void AttachCameraToObj(int index)
    {
        car_in_focus_ = index;
        camTarget.transform.parent = cars[car_in_focus_].transform;
        camTarget.transform.position = new Vector3(0.0f, 2.5f, -6.0f);
        camTarget.transform.rotation = Quaternion.Euler(15, 0, 0);
        _cam.transform.position = camTarget.transform.position;
        print("Camera follows car " + index);
    }

    private void UpdateObjectPositions(bool fetchEgo)
    {
        if (interactive_ && !fetchEgo)
        {
            Transform c = cars[0].transform;

            // Report ego position
            SE_ReportObjectPos(0, simTime, c.position.z, -c.position.x, c.position.y, 
                -c.eulerAngles.y * Mathf.PI / 180.0f, c.eulerAngles.x * Mathf.PI / 180.0f, -c.eulerAngles.z * Mathf.PI / 180.0f, 
                egoBody.velocity.magnitude);
        }

        // Check nr of objects
        for (int i = 0; i < SE_GetNumberOfObjects(); i++ )
        {
            if(interactive_ && (i==0 && !fetchEgo))
            {
                continue;
            }

            SE_GetObjectState(i, ref state);

            // Instantiate objects
            if (i > 0 && cars.Count <= i)
            {
                // Add scenario controlled objects
                cars.Add((GameObject)Instantiate(Resources.Load(objectNames[state.model_id])));
                print("Adding " + objectNames[state.model_id]);
                if (SE_ObjectHasGhost(i) == 1)
                {
                    AttachCameraToObj(i);
                }
            }

            GameObject car = cars[i];

            if (SE_ObjectHasGhost(state.id) == 1 && SE_GetSimulationTime() > 0.0)
            {
                // Get road and ghost sensors
                RoadInfo road_info = new RoadInfo();
                float speed_ghost = 0;

                SE_GetRoadInfoAtDistance(state.id, 40, ref road_info, 1);
                SE_GetRoadInfoAlongGhostTrail(state.id, 10, ref road_info, ref speed_ghost);

                Vector3 pos_delta = new Vector3(road_info.global_pos_x - state.x, road_info.global_pos_y - state.y, road_info.global_pos_z - state.z);
                Vector3 rot_delta = new Vector3(road_info.trail_heading - state.h, road_info.road_pitch - state.p, road_info.road_roll - state.r);

                state.x += 0.05f * (road_info.global_pos_x - state.x);
                state.y += 0.05f * (road_info.global_pos_y - state.y);
                state.z += 0.05f * (road_info.global_pos_z - state.z);
                state.h += 0.2f * (road_info.trail_heading - state.h);
                state.p += 0.2f * (road_info.road_pitch - state.p);
                state.r += 0.2f * (road_info.road_roll - state.r);
                //pos.x -= 0.1f;

                SE_ReportObjectPos(state.id, 0, state.x, state.y, state.z, state.h, state.p, state.r, 0);
            }
            // Adapt to Unity coordinate system
            car.transform.position = RH2Unity(new Vector3(state.x, state.y, state.z));            
            car.transform.rotation = Quaternion.Euler(RHHPR2UnityXYZ(new Vector3(state.h, state.p, state.r)));
        }
    }

    private void FixedUpdate()
    {
        if(!scenarioLoaded)
        {
            return;
        }

        if (SE_GetNumberOfObjects() > 0)
        {
            simTime += Time.deltaTime;
            SE_StepDT(Time.deltaTime);

            int nObj = SE_FetchSensorObjectList(0, objList);
            //print("Sensor hits: " + nObj);
            for (int i=0; i<nObj; i++)
            {
                SE_GetObjectState(objList[i], ref state);
                //print("state x: " + state.x + ", y: " + state.y);
            }

            UpdateObjectPositions(false);

            // Let camera follow first object - assumed to be the Ego vehicle
            _cam.transform.parent = cars[car_in_focus_].transform;

            Vector3 diffVec = camTarget.transform.position - _cam.transform.position;
            Vector3 diffUnitVec = diffVec.normalized;

            float acceleration = tension * diffVec.magnitude - 2*Mathf.Sqrt(tension) * speed;
    
            Vector3 newPos = _cam.transform.position + (speed + acceleration * Time.deltaTime) * diffUnitVec * Time.deltaTime;
            speed = (newPos - _cam.transform.position).magnitude / Time.deltaTime;

            _cam.transform.position = newPos;
            _cam.transform.LookAt(cars[car_in_focus_].transform);
        }
    }
}
