
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

/*
 * This module provides an example of how RoadManagerDLL can be integrated with c# Unity applications
 * It simply mirrors the interface in terms of datatypes and functions
 * To use RoadManagerDLL in Unity:
 *  - put the RoadManagerDLL.dll (or a library format relevant to the platform) in a folder named "plugins" in the Unity project
 *  - put this c# script in a folder named "scripts" in the Unity project
 *  - now the application script can use the functionality of the dll
 */

using UnityEngine;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System;

namespace OpenDRIVE
{

    [StructLayout(LayoutKind.Sequential)]
    public struct OpenDrivePositionData
    {
        public float x;
        public float y;
        public float z;
        public float h;
        public float p;
        public float r;
        public float hRelative;
        public int roadId;
        public int laneId;
        public float laneOffset;
        public float s;
    };

    public struct WorldPose
    {
        public Vector3 position;
        public Quaternion rotation;
    }


    [StructLayout(LayoutKind.Sequential)]
    public struct PositionXYZ
    {
        [MarshalAsAttribute(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] pos;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct SteeringTargetData
    {
        [MarshalAsAttribute(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] global_pos;          // steering target position, in global coordinate system
        [MarshalAsAttribute(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] local_pos;           // steering target position, relative vehicle (pivot position object) coordinate system
        public float angle;                 // heading angle to steering target from and relatove to vehicle (pivot position)
        public float road_heading;          // road heading at steering target point
        public float road_pitch;            // road pitch (inclination) at steering target point
        public float road_roll;             // road roll (camber) at steering target point
        public float curvature;             // road curvature at steering target point
        public float speed_limit;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct RoadLaneInfo
    {
        [MarshalAsAttribute(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] pos;            // position, in global coordinate system
        public float heading;          // road heading 
        public float pitch;            // road pitch 
        public float roll;			   // road roll
        public float width;            // Lane width 
        public float curvature;        // curvature (1/radius), >0 for left curves, <0 for right curves
        public float speed_limit;      // road speed limit 
    };


    [StructLayout(LayoutKind.Sequential)]
    public struct PositionDiff
    {
        public float ds;                       // delta s (longitudinal distance)
        public float dt;                       // delta t (lateral distance)
        public int dLaneId;			        // delta laneId (increasing left and decreasing to the right)
    };

    enum JunctionStrategy { Random, Straight };  // must correlate to RoadManager::Junction::JunctionStrategyType

    public static class OpenDriveUtil
    {

        private static OpenDrivePositionData tmpPosData = new OpenDrivePositionData();
        private static Vector3 tmpPos = new Vector3();
        private const float RAD2DEG = 180f / Mathf.PI;
        private const float deg2Rad = Mathf.Deg2Rad;

        private const string LIB_NAME = "RoadManagerDLL";

        /// <summary>Initialize the OpenDRIVE utility manager</summary>
        /// <param name="odrFilename">OpenDRIVE file name</param>
        /// <returns>0 on success, -1 on failure for any reason</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_Init")]
        public static extern int Init(string odrFilename);

        /// <summary>Close down the OpenDRIVE utility manager respectfully</summary>
        [DllImport(LIB_NAME, EntryPoint = "RM_Close")]
        public static extern void Close();

        /// <summary>Create a position object</summary>
        /// <returns>Index (handle) to the position object, to use for operations</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_CreatePosition")]
        public static extern int CreatePosition();

        /// <summaryGet the total number fo roads in the road network of the currently loaded OpenDRIVE file</summary>
        /// <returns>Number of roads</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetNumberOfRoads")]
        public static extern int GetNumberOfRoads();

        /// <summary>
        /// Get the road ID, as specified in the OpenDRIVE description, of the road with specified index. E.g. if there are 4 roads, index 3 means the last one.
        /// </summary>
        /// <param name="index">The index of the road</param>
        /// <returns>The OpenDRIVE ID of the road</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetIdOfRoadFromIndex")]
        public static extern int GetIdOfRoadFromIndex(int index);

        /// <summary>
        /// Get the lenght of road with specified ID
        /// </summary>
        /// <param name="id">The OpenDRIVE road ID</param>
        /// <returns> The length of the road if ID exists, else 0.0</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetRoadLength")]
        public static extern float GetRoadLength(int id);

        /// <summary>
        /// Get the number of drivable lanes of specified road
        /// </summary>
        /// <param name="roadId">The OpenDRIVE road ID</param>
        /// <param name="s">The distance along the road at what point to check number of lanes (which can vary along the road)</param>
        /// <returns>The number of drivable lanes</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetRoadNumberOfLanes")]
        public static extern int GetRoadNumberOfLanes(int roadId, float s);

        /// <summary>
        /// Get the OpenDRIVE ID of the lane given by index
        /// </summary>
        /// <param name="roadId">The OpenDRIVE road ID</param>
        /// <param name="laneIndex">The index of the lane </param>
        /// <param name="s">The distance along the road at what point to look up the lane ID</param>
        /// <returns>The lane ID - as specified in the OpenDRIVE description</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetLaneIdByIndex")]
        public static extern int GetLaneIdByIndex(int roadId, int laneIndex, float s);

        /// <summary>
        /// Set position from road coordinates, world coordinates being calculated
        /// </summary>
        /// <param name="index">Handle to the position object</param>
        /// <param name="roadId">The OpenDRIVE road ID</param>
        /// <param name="laneID">Lane specifier</param>
        /// <param name="laneOffset">Offset from lane center</param>
        /// <param name="s">Distance along the specified road</param>
        /// <param name="align">If true the heading will be reset to the lane driving direction (typically only at initialization)</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetLanePosition")]
        public static extern int SetLanePosition(int index, int roadId, int laneId, float laneOffset, float s, bool align);

        /// <summary>
        /// Set s (distance) part of a lane position, world coordinates being calculated
        /// </summary>
        /// <param name="index">Handle to the position object</param>
        /// <param name="s">Distance along the specified road</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetS")]
        public static extern int SetS(int index, float s);

        /// <summary>
        /// Set position from world coordinates in the OpenDRIVE coordinate system.
        /// </summary>
        /// <param name="index">Handle to the position object</param>
        /// <param name="x">cartesian coordinate x value</param>
        /// <param name="y">cartesian coordinate y value</param>
        /// <param name="z">cartesian coordinate z value</param>
        /// <param name="h">rotation heading value</param>
        /// <param name="p">rotation pitch value</param>
        /// <param name="r">rotation roll value</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetWorldPosition")]
        public static extern int SetOpenDriveWorldPosition(int index, float x, float y, float z, float h, float p, float r);

        /// <summary>
        /// Set position from world coordinates in the Unity coordinate system.
        /// </summary>
        public static void SetWorldPosition(int index, Vector3 position, Vector3 rotationEuler)
        {
            Vector3 odrPos = GetOpenDrivePosition(position);
            Vector3 odrRot = GetOpenDriveRotation(rotationEuler);
            SetOpenDriveWorldPosition(index, odrPos.x, odrPos.y, odrPos.z, odrRot.x, odrRot.y, odrRot.z);
        }
        
        /// <summary>
        /// Set position from world X, Y and heading coordinates; Z, pitch and road coordinates being calculated
        /// </summary>
        /// <param name="index">Handle to the position object</param>
        /// <param name="x">cartesian coordinate x value</param>
        /// <param name="y">cartesian coordinate y value</param>
        /// <param name="z">cartesian coordinate z value</param>
        /// <param name="h">rotation heading value</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport("RoadManagerDLL", EntryPoint = "RM_SetWorldXYZHPosition")]
        public static extern int SetOpenDriveWorldXYZHPosition(int index, float x, float y, float z, float h);

        /// <summary>
        /// Move position forward along the road. Choose way randomly though any junctions.
        /// </summary>
        /// <param name="index">Handle to the position object</param>
        /// <param name="dist">Distance (in meter) to move</param>
        /// <param name="strategy">How to move in a junction where multiple route options appear, use enum JunctionStrategy</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_PositionMoveForward")]
        public static extern int PositionMoveForward(int index, float dist, int strategy);

        /// <summary>
        /// Get the fields of the position of specified index
        /// </summary>
        /// <param name="index">Handle to the position object</param>
        /// <param name="data">Struct to fill in the values</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetPositionData")]
        public static extern int GetOpenDrivePositionData(int index, ref OpenDrivePositionData data);

        /// <summary>
        /// Retrieve lane information from the position object (at current road, s-value and lane)
        /// </summary>
        /// <param name="index">Handle to the position object from which to measure</param>
        /// <param name="data">Struct including all result values, see RoadLaneInfo typedef</param>
        /// <param name "lookAheadMode">Measurement strategy: Along reference lane, lane center or current lane offset. See roadmanager::Position::LookAheadMode enum</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport("RoadManagerDLL", EntryPoint = "RM_GetLaneInfo")]
        public static extern int GetLaneInfo(int index, float lookahead_distance, ref RoadLaneInfo data, int lookAheadMode);

        /// <summary>
        /// Retrieve current speed limit (at current road, s-value and lane) based on ODR type elements or nr of lanes
        /// </summary>
        /// <param name="index">Handle to the position object from which to measure</param>
        /// <returns>SpeedLimit in m/s</returns>
        [DllImport("RoadManagerDLL", EntryPoint = "RM_GetSpeedLimit")]
        public static extern float GetSpeedLimit(int index);

        /// <summary>
        /// Get the location, in global coordinate system, of the point at a specified distance from starting position along the road ahead
        /// </summary>
        /// <param name="index">Handle to the position object from which to measure</param>
        /// <param name="lookahead_distance">The distance, along the road, to the point</param>
        /// <param name "lookAheadMode">Measurement strategy: Along reference lane, lane center or current lane offset. See roadmanager::Position::LookAheadMode enum</param>
        /// <param name="data">Struct including all result values, see SteeringTargetData typedef</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport("RoadManagerDLL", EntryPoint = "RM_GetSteeringTarget")]
        public static extern int GetSteeringTarget(int index, float lookahead_distance, ref SteeringTargetData data, int lookAheadMode);

        /// <summary>
        /// Find out the difference between two position objects, i.e. delta distance (long and lat) and delta laneId
        /// </summary>
        /// <param name="handleA">Handle to the position object from which to measure</param>
        /// <param name="handleB">Handle to the position object to which the distance is measured</param>
        /// <param name="pos_diff">Struct including all result values, see PositionDiff typedef</param>
        /// <returns>true if a valid path between the road positions was found and calculations could be performed</returns>
        [DllImport("RoadManagerDLL", EntryPoint = "RM_SubtractAFromB")]
        public static extern bool SubtractAFromB(int handleA, int handleB, ref PositionDiff pos_diff);

        /// <summary>
        /// Returns the world position and rotation of the road user with handle index.
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public static WorldPose GetWorldPose(int index)
        {
            WorldPose pose = new WorldPose();
            GetOpenDrivePositionData(index, ref tmpPosData);
            pose.position.x = -tmpPosData.y;
            pose.position.y = tmpPosData.z;
            pose.position.z = tmpPosData.x;

            pose.rotation = Quaternion.Euler(tmpPosData.p * RAD2DEG, -tmpPosData.h * RAD2DEG, tmpPosData.r * RAD2DEG);

            return pose;
        }

        /// <summary>
        /// Returns the world rotation of the road user with handle index.
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public static Quaternion GetWorldRotation(int index)
        {
            GetOpenDrivePositionData(index, ref tmpPosData);

            float rel_heading = -tmpPosData.hRelative * RAD2DEG;
            float pitch = tmpPosData.p * RAD2DEG;
            if (rel_heading < -90f || rel_heading > 90f) {
                pitch *= -1;
            }

            return Quaternion.Euler(pitch, -tmpPosData.h * RAD2DEG, tmpPosData.r * RAD2DEG);
        }
        
        /// <summary>
        /// Returns the OpenDrive world coordinates given a position in Unity's coordinate system.
        /// </summary>
        public static Vector3 GetOpenDrivePosition(Vector3 unityPosition)
        {
            return new Vector3(unityPosition.z, -unityPosition.x, unityPosition.y);
        }


        /// <summary>
        /// Returns the OpenDRIVE rotation given a rotation in Unity's coordinate system.
        /// </summary>
        public static Vector3 GetOpenDriveRotation(Vector3 unityRotationEuler)
        {
            return new Vector3(-unityRotationEuler.y * deg2Rad, unityRotationEuler.x * deg2Rad, unityRotationEuler.z * deg2Rad);
        }

        /// <summary>
        /// Returns the world position of the road user with handle index.
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public static Vector3 GetWorldPosition(int index)
        {
            GetOpenDrivePositionData(index, ref tmpPosData);
            return new Vector3(-tmpPosData.y, tmpPosData.z, tmpPosData.x);
        }


        /// <summary>
        /// Sets the given transform to match the position and rotation of the road user with handle index.
        /// </summary>
        /// <param name="index"></param>
        /// <param name="gameObjTransform">Transform of the road user game object.</param>
        public static void AlignGameObjectToHandle(int index, GameObject go)
        {
            GetOpenDrivePositionData(index, ref tmpPosData);
            tmpPos.x = -tmpPosData.y;
            tmpPos.y = tmpPosData.z;
            tmpPos.z = tmpPosData.x;

            // Find out pitch of road in driving direction
            float rel_heading = (-tmpPosData.hRelative * RAD2DEG) % 360 + 360;
            float pitch = tmpPosData.p * RAD2DEG;


            go.transform.SetPositionAndRotation(tmpPos, Quaternion.Euler(pitch, -tmpPosData.h * RAD2DEG, tmpPosData.r * RAD2DEG));
        }

    }

}