#define USE_STATE_REF

using UnityEngine;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.IO;
using UnityEngine.UI;


// Note: Hardcoded objects, only supporting scenarios with two objects with id 0 and 1

// A state struct may look something like this - not used yet

[StructLayout(LayoutKind.Sequential)]
public struct ScenarioObjectState
{
    public int id;
    [MarshalAsAttribute(UnmanagedType.ByValArray, SizeConst = 32)]
    public char[] name;
    public float timestamp;
    public float x;
    public float y;
    public float z;
    public float h;
    public float p;
    public float r;
    public int roadId;
    public int laneId;
    public float laneOffset;
    public float s;
    public float speed;
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
    public static extern int SE_ReportObjectPos(int id, string name, float timestamp, float x, float y, float z, float h, float p, float r, float speed, float acceleration);

    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_ReportObjectRoadPos")]
    public static extern int SE_ReportObjectRoadPos(int id, string name, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed, float acceleration);

    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_GetNumberOfObjects")]
    public static extern int SE_GetNumberOfObjects();

#if USE_STATE_REF
    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_GetObjectState")]
    public static extern int SE_GetObjectState(int index, ref ScenarioObjectState state);
#else
    [DllImport("ScenarioEngineDLL", EntryPoint = "SE_GetObjectState")]
    public static extern ScenarioObjectState SE_GetObjectState(int index);
#endif

    public string scenarioFolder;
    public Button btn_cutin, btn_cutin_ego, btn_ltapod, btn_ltapod_ego;
    public float tension = 1.0f;
    private Rigidbody egoBody;
    private float simTime;

    private Camera _cam;
    private GameObject camTarget;
    private GameObject envModel;
    private bool scenarioLoaded = false;
    private float speed = 0.0f;
    private bool control_ego_ = false;
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
        GameObject original = GameObject.FindWithTag("MainCamera");
        _cam = (Camera)Camera.Instantiate(original.GetComponent<Camera>());
        Destroy(original);

        camTarget = new GameObject("camTarget");

        Button btn1 = btn_cutin.GetComponent<Button>();
        Button btn1b = btn_cutin_ego.GetComponent<Button>();
        Button btn2 = btn_ltapod.GetComponent<Button>();
        Button btn3 = btn_ltapod_ego.GetComponent<Button>();

        //Calls the TaskOnClick/TaskWithParameters method when you click the Button
        btn1.onClick.AddListener(delegate {
            InitScenario(scenarioFolder + Path.DirectorySeparatorChar + "cut-in_mw.xosc",
            "e6mini",
            false);
        });
        btn1b.onClick.AddListener(delegate {
            InitScenario(scenarioFolder + Path.DirectorySeparatorChar + "cut-in_mw_external.xosc",
            "e6mini",
            true);
        });
        btn2.onClick.AddListener(delegate {
            InitScenario(scenarioFolder + Path.DirectorySeparatorChar + "ltap-od-variant_C_two_targets_internal.xosc",
            "fabriksg",
            false);
        });
        btn3.onClick.AddListener(delegate {
            InitScenario(scenarioFolder + Path.DirectorySeparatorChar + "ltap-od-variant_C_two_targets.xosc",
            "fabriksg",
            true);
        });
    }

    void OnApplicationQuit()
    {
        SE_Close();
    }

    private void InitScenario(string scenarioFile, string modelName, bool control_ego)
    {
        print("Init scenario " + scenarioFile);
        scenarioLoaded = false;
        speed = 0;
        control_ego_ = control_ego;
        simTime = 0;

        // Detach camera from any previous parent, then init its transform
        camTarget.transform.parent = null;
        camTarget.transform.position = new Vector3(0.0f, 2.5f, -6.0f);
        camTarget.transform.rotation = Quaternion.Euler(15, 0, 0);
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
        int returnvalue = SE_Init(scenarioFile);
        if (returnvalue != 0)
        {
            print("failed to load " + scenarioFile);
            return;
        }

        // Instantiate objects
        for (int i = 0; i < SE_GetNumberOfObjects(); i++)
        {
            if (i == 0 && control_ego_)
            {
                // Load Ego
                cars.Add((GameObject)Instantiate(Resources.Load("S90")));
                egoBody = cars[0].GetComponent<Rigidbody>();
                print("Adding Ego");
            }
            else
            {
                // Add scenario controlled objects
                cars.Add((GameObject)Instantiate(Resources.Load(objectNames[i % objectNames.Count])));
                print("Adding " + objectNames[i % objectNames.Count]);
            }
        }
        if (SE_GetNumberOfObjects() > 0)
        {
            camTarget.transform.parent = cars[0].transform;
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

    private void UpdateObjectPositions(bool fetchEgo)
    {
        if (control_ego_ && !fetchEgo)
        {
            Transform c = cars[0].transform;

            // Report ego position
            SE_ReportObjectPos(0, "Ego", simTime, c.position.z, -c.position.x, c.position.y,
                -c.eulerAngles.y * Mathf.PI / 180.0f, -c.eulerAngles.x * Mathf.PI / 180.0f, c.eulerAngles.z * Mathf.PI / 180.0f,
                egoBody.velocity.magnitude);
        }

        float x, y, z, x_rot, y_rot, z_rot;
        ScenarioObjectState state = new ScenarioObjectState();

        // Check nr of objects
        for (int i = 0; i < SE_GetNumberOfObjects(); i++ )
        {
            if(control_ego_ && (i==0 && !fetchEgo))
            {
                continue;
            }

            GameObject car = cars[i];

#if USE_STATE_REF
            SE_GetObjectState(i, ref state);
#else
            ScenarioObjectState state = SE_GetObjectState(i);
#endif

            // Adapt to Unity coordinate system
            x = -state.y;
            y = state.z;
            z = state.x;
            car.transform.position = new Vector3(x, y, z);

            y_rot = -state.h * 180.0f / Mathf.PI;
            x_rot = -state.p * 180.0f / Mathf.PI;
            z_rot = state.r * 180.0f / Mathf.PI;
            car.transform.rotation = Quaternion.Euler(x_rot, y_rot, z_rot);
        }
    }

    private void Update()
    {

        if(!scenarioLoaded)
        {
            return;
        }

        if (SE_GetNumberOfObjects() > 0)
        {
            simTime += Time.deltaTime;
            SE_Step(Time.deltaTime);
            UpdateObjectPositions(false);

            // Let camera follow first object - assumed to be the Ego vehicle
            Vector3 diffVec = camTarget.transform.position - _cam.transform.position;
            Vector3 diffUnitVec = diffVec.normalized;

            float acceleration = tension * diffVec.magnitude - Mathf.Sqrt(2 * tension) * speed;

            Vector3 newPos = _cam.transform.position + (speed + acceleration * Time.deltaTime) * diffUnitVec * Time.deltaTime;
            speed = (newPos - _cam.transform.position).magnitude / Time.deltaTime;

            _cam.transform.position = newPos;
            _cam.transform.LookAt(cars[0].transform);
        }
    }
}
