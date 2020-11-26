/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

using UnityEngine;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System;
using System.IO;
using UnityEngine.UI;
using ESMini;

public class esminiUnityExample : MonoBehaviour
{
    private string scenarioFolder;
    public Button btn_cutin, btn_cutin_interactive, btn_ltapod, btn_follow_ghost;
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
        Button btn2 = btn_cutin_interactive.GetComponent<Button>();
        Button btn3 = btn_ltapod.GetComponent<Button>();
        Button btn4 = btn_follow_ghost.GetComponent<Button>();
        Toggle t_osg = toggle_osg.GetComponent<Toggle>();

        //Calls the TaskOnClick/TaskWithParameters method when you click the Button
        btn1.onClick.AddListener(delegate {
            InitScenario(scenarioFolder + Path.DirectorySeparatorChar + "cut-in_external.xosc", "e6mini", 1, t_osg.isOn, false);
        });
        btn2.onClick.AddListener(delegate {
            InitScenario(scenarioFolder + Path.DirectorySeparatorChar + "cut-in_external.xosc", "e6mini", 0, t_osg.isOn, true);
        });
        btn3.onClick.AddListener(delegate {
            InitScenario(scenarioFolder + Path.DirectorySeparatorChar + "ltap-od.xosc", "fabriksg", 1, t_osg.isOn, false);
        });
        btn4.onClick.AddListener(delegate {
            InitScenario(scenarioFolder + Path.DirectorySeparatorChar + "follow_ghost.xosc", "e6mini", 0, t_osg.isOn, false);
        });

        esminiLib.SE_AddPath("c:/tmp");
    }

    void OnApplicationQuit()
    {
        esminiLib.SE_Close();
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
        if (esminiLib.SE_Init(scenarioFile, disable_controller, osg_viewer ? 1 : 0, 0, 0) != 0)
        {
            print("failed to load " + scenarioFile);
            return;
        }

        // Add sensors
        if (esminiLib.SE_AddObjectSensor(0, 4, 0, 0.5f, 0.0f, 5.0f, 50.0f, (float)(50 * Math.PI / 180.0), 10) != 0)
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


        if (esminiLib.SE_GetNumberOfObjects() > 0)
        {
            AttachCameraToObj(0);
        }

        // Load environment 3D model
        envModel = (GameObject)Instantiate(Resources.Load(modelName));

        // Fetch the initial object positions
        UpdateObjectPositions(true);

        if (esminiLib.SE_GetNumberOfObjects() > 0)
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
            esminiLib.SE_ReportObjectPos(0, simTime, c.position.z, -c.position.x, c.position.y,
                -c.eulerAngles.y * Mathf.PI / 180.0f, c.eulerAngles.x * Mathf.PI / 180.0f, -c.eulerAngles.z * Mathf.PI / 180.0f,
                egoBody.velocity.magnitude);
        }

        // Check nr of objects
        for (int i = 0; i < esminiLib.SE_GetNumberOfObjects(); i++)
        {
            if (interactive_ && (i == 0 && !fetchEgo))
            {
                continue;
            }

            esminiLib.SE_GetObjectState(i, ref state);

            // Instantiate objects
            if (i > 0 && cars.Count <= i)
            {
                // Add scenario controlled objects
                cars.Add((GameObject)Instantiate(Resources.Load(objectNames[state.model_id])));
                print("Adding " + objectNames[state.model_id]);
                if (esminiLib.SE_ObjectHasGhost(i) == 1)
                {
                    AttachCameraToObj(i);
                }
            }

            GameObject car = cars[i];
            if (esminiLib.SE_ObjectHasGhost(state.id) == 1 && esminiLib.SE_GetSimulationTime() > 0.0)
            {
                // Get road and ghost sensors
                RoadInfo road_info = new RoadInfo();
                float speed_ghost = 0;

                esminiLib.SE_GetRoadInfoAtDistance(state.id, 40, ref road_info, 1);
                esminiLib.SE_GetRoadInfoAlongGhostTrail(state.id, 10, ref road_info, ref speed_ghost);

                Vector3 pos_delta = new Vector3(road_info.global_pos_x - state.x, road_info.global_pos_y - state.y, road_info.global_pos_z - state.z);
                Vector3 rot_delta = new Vector3(road_info.trail_heading - state.h, road_info.road_pitch - state.p, road_info.road_roll - state.r);

                state.x += 0.05f * (road_info.global_pos_x - state.x);
                state.y += 0.05f * (road_info.global_pos_y - state.y);
                state.z += 0.05f * (road_info.global_pos_z - state.z);
                state.h += 0.2f * (road_info.trail_heading - state.h);
                state.p += 0.2f * (road_info.road_pitch - state.p);
                state.r += 0.2f * (road_info.road_roll - state.r);
                //pos.x -= 0.1f;

                esminiLib.SE_ReportObjectPos(state.id, 0, state.x, state.y, state.z, state.h, state.p, state.r, 0);
            }
            // Adapt to Unity coordinate system
            car.transform.position = RH2Unity(new Vector3(state.x, state.y, state.z));
            car.transform.rotation = Quaternion.Euler(RHHPR2UnityXYZ(new Vector3(state.h, state.p, state.r)));
        }
    }

    private void FixedUpdate()
    {
        if (!scenarioLoaded)
        {
            return;
        }

        if (esminiLib.SE_GetNumberOfObjects() > 0)
        {
            simTime += Time.deltaTime;
            esminiLib.SE_StepDT(Time.deltaTime);

            int nObj = esminiLib.SE_FetchSensorObjectList(0, objList);
            //print("Sensor hits: " + nObj);
            for (int i = 0; i < nObj; i++)
            {
                esminiLib.SE_GetObjectState(objList[i], ref state);
                //print("state x: " + state.x + ", y: " + state.y);
            }

            UpdateObjectPositions(false);

            // Let camera follow first object - assumed to be the Ego vehicle
            _cam.transform.parent = cars[car_in_focus_].transform;

            Vector3 diffVec = camTarget.transform.position - _cam.transform.position;
            Vector3 diffUnitVec = diffVec.normalized;

            float acceleration = tension * diffVec.magnitude - 2 * Mathf.Sqrt(tension) * speed;

            Vector3 newPos = _cam.transform.position + (speed + acceleration * Time.deltaTime) * diffUnitVec * Time.deltaTime;
            speed = (newPos - _cam.transform.position).magnitude / Time.deltaTime;

            _cam.transform.position = newPos;
            _cam.transform.LookAt(cars[car_in_focus_].transform);
        }
    }
}
