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

/* A simple example of how to run esmini within a Unity script/application
 *
 * Preparations:
 * - put OpenSCENARIO files in folder Assets/StreamingAssets/xosc 
 * - put OpenDRIVE files in folder Assets/StreamingAssets/xodr
 * - put OSG models in folder Assets/StreamingAssets/models
 * - put prefabs of Unity versions of road and vehicle models in folder Assets/Resources
 * - put this script together with ESMiniWrapper.cs in folder Assets/Scripts
 * - put esminiLib (esminiLib.DLL, libesminiLib.so...) in folder Assets/Plugins
 */

using UnityEngine;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using System;
using System.IO;
using UnityEngine.UI;
using UnityEditor;
using ESMini;


public class esminiUnityExample : MonoBehaviour
{
    public string OSC_filename = "/xosc/cut-in.xosc";
    [Tooltip("Open a separate OSG rendering window.")]
    public bool OSG_visualization = true;
    [Tooltip("Check to run OSG in separate thread - try pause Unity and you can still operate OSG window.")]
    public bool threads = false;
    [Tooltip("Disregard any controllers - esmini will apply default behaviour.")]
    public bool disable_controllers = true;
    
    private GameObject cam;
    private ScenarioObjectState state;
    private List<GameObject> cars = new List<GameObject>();
    private GameObject envModel;
    private List<string> objectNames = new List<string>
        {
            "car1",
            "car2",
            "car3",
            "car4",
            "car5",
            "car6",
            "bus1",
            "bus2",
        };

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

    private void Start()
    {
        state = new ScenarioObjectState();
        cam = GameObject.FindWithTag("MainCamera");
              
        InitScenario();
    }

    private void InitScenario()
    {
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


        if (ESMiniLib.SE_Init( Application.streamingAssetsPath + OSC_filename, 
            disable_controllers ? 1 : 0, 
            OSG_visualization ? 1 : 0, 
            threads ? 1 : 0, 
            0) != 0)  // don't create .dat-recording for replayer
        {
            print("failed to load scenario");
            return;
        }

        // Load environment 3D model
        string sceneGraphFilename = Marshal.PtrToStringAnsi(ESMiniLib.SE_GetSceneGraphFilename());
        Debug.Log("Loading " + Path.GetFileNameWithoutExtension(sceneGraphFilename));
        envModel = (GameObject)Instantiate(Resources.Load(Path.GetFileNameWithoutExtension(sceneGraphFilename)));
    }


    void OnApplicationQuit()
    {
        Debug.Log("Quit");
        ESMiniLib.SE_Close();
    }

    public void Reload()
    {
        Debug.Log("Reload");
        cam.transform.SetParent(null);
        ESMiniLib.SE_Close();
        InitScenario();
    }

    private void Update()
    {
        ESMiniLib.SE_StepDT(Time.deltaTime);
 
        if (ESMiniLib.SE_GetQuitFlag())
        {
     #if UNITY_EDITOR
            // Application.Quit() does not work in the editor so
            // UnityEditor.EditorApplication.isPlaying need to be set to false to end the game
            ESMiniLib.SE_Close();
            UnityEditor.EditorApplication.isPlaying = false;
     #else
            Application.Quit();
     #endif
        }

        // Check nr of objects
        for (int i = 0; i < ESMiniLib.SE_GetNumberOfObjects(); i++)
        {
            ESMiniLib.SE_GetObjectState(i, ref state);

            // Instantiate objects
            if (cars.Count <= i)
            {
                // Add scenario controlled objects
                int model_id = Mathf.Min(state.model_id, objectNames.Count-1);
                cars.Add((GameObject)Instantiate(Resources.Load(objectNames[model_id])));
                Debug.Log("Adding " + objectNames[model_id]);
                
                // Attach camera to first object
                if (i==0)
                {
                    cam.transform.SetParent(cars[0].transform);
                    cam.transform.position = new Vector3(0.0f, 4f, -12.0f);
                    cam.transform.rotation = Quaternion.Euler(10, 0, 0);
                }
            }

            // Adapt to Unity coordinate system
            cars[i].transform.position = RH2Unity(new Vector3(state.x, state.y, state.z));
            cars[i].transform.rotation = Quaternion.Euler(RHHPR2UnityXYZ(new Vector3(state.h, state.p, state.r)));
        }
    }
}

[CustomEditor(typeof(esminiUnityExample))]
public class customButton : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        esminiUnityExample myScript = (esminiUnityExample)target;
        if (GUILayout.Button("Reload"))
        {
            myScript.Reload();
        }
    }
}
