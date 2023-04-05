
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
 * This module complements RoadManagerLibraryCS.cs with helper methods adapting to Unity coordinate system
 * To use RoadManagerDLL in Unity:
 *  - put the RoadManagerDLL.dll (or a library format relevant to the platform) in a folder named "plugins" in the Unity project
 *  - put the RoadManagerLibraryCS.cs (generic C# wrapper) in a folder named "scripts" in the Unity project
 *  - also put this C# script in the same folder
 *  - now the application script can use the functionality of the dll
 */

using UnityEngine;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System;
using System.Text;

namespace OpenDRIVE
{
    public struct WorldPose
    {
        public Vector3 position;
        public Quaternion rotation;
    }

    public struct OpenDrivePositionDataUnityCoordinates
    {
        public Vector3 position;
        public Quaternion rotation;
        public float hRelative;
        public int roadId;
        public int laneId;
        public float laneOffset;
        public float s;
        public int junctionId; // junction id is -1 if the position is not in a junction
    };

    public struct RoadLaneInfoUnityCoordinates
    {
        public Vector3 position;
        public Quaternion rotation;
        public float width;            // Lane width
        public float curvature;        // curvature (1/radius), >0 for left curves, <0 for right curves
        public float speedLimit;       // road speed limit
        public int roadId;             // road ID
        public int laneId;             // lane ID
        public float laneOffset;       // lane offset (lateral distance from lane center)
        public float s;                // s (longitudinal distance along reference line)
        public float t;                // t (lateral distance from reference line)
        public int junctionId;         // junction id is -1 if the position is not in a junction
    };

    public struct RoadProbeInfoUnityCoordinates
    {
        public RoadLaneInfoUnityCoordinates roadLaneInfo;
        public Vector3 relativePosition; // probe position, relative vehicle (pivot position object) coordinate system
        public float relativeHeading;    // heading angle to steering target from and relatove to vehicle (pivot position)    };
    };

    public static class OpenDriveUtil
    {
        #region Properties
        public enum LookAheadMode
        {
            LaneCenter,
            RoadCenter,
            CurrentLateralOffset
        }

        private static OpenDrivePositionData tmpPosData = new OpenDrivePositionData();
        private static RoadLaneInfo tmpLaneInfo = new RoadLaneInfo();
        private static RoadProbeInfo tmpProbeInfo = new RoadProbeInfo();
        private const float RAD2DEG = Mathf.Rad2Deg;
        private const float DEG2RAD = Mathf.Deg2Rad;

        #endregion

        #region Public Methods

        /// <summary>
        /// Set position from world coordinates in the Unity coordinate system.
        /// </summary>
        public static void SetWorldPosition(int openDriveIndex, Vector3 position)
        {
            Vector3 odrPos = GetOpenDrivePosition(position);
            RoadManagerLibraryCS.SetWorldXYHPosition(openDriveIndex, odrPos.x, odrPos.y, 0);
            RoadManagerLibraryCS.GetPositionData(openDriveIndex, ref tmpPosData);
            RoadManagerLibraryCS.SetLanePosition(openDriveIndex, tmpPosData.roadId, tmpPosData.laneId, tmpPosData.laneOffset, tmpPosData.s, true);
        }

        /// <summary>
        /// Set position and rotation from world coordinates in the Unity coordinate system.
        /// </summary>
        public static void SetWorldPosition(int openDriveIndex, Vector3 position, Vector3 rotationEuler)
        {
            Vector3 odrPos = GetOpenDrivePosition(position);
            Vector3 odrRot = GetOpenDriveRotation(rotationEuler);
            RoadManagerLibraryCS.SetWorldPosition(openDriveIndex, odrPos.x, odrPos.y, odrPos.z, odrRot.x, odrRot.y, odrRot.z);
        }

        /// <summary>
        /// Returns the world position and rotation of the road user with handle index.
        /// </summary>
        /// <param name="openDriveIndex"></param>
        /// <returns></returns>
        public static WorldPose GetWorldPose(int openDriveIndex)
        {
            WorldPose pose = new WorldPose();
            RoadManagerLibraryCS.GetPositionData(openDriveIndex, ref tmpPosData);
            pose.position = GetUnityPosition(tmpPosData);
            pose.rotation = GetUnityRotation(tmpPosData);

            return pose;
        }

        public static void GetOpenDrivePositionDataUnityCoordinates(int openDriveIndex, ref OpenDrivePositionDataUnityCoordinates unityPosData)
        {
            RoadManagerLibraryCS.GetPositionData(openDriveIndex, ref tmpPosData);
            unityPosData.position = GetUnityPosition(tmpPosData);
            unityPosData.rotation = GetUnityRotation(tmpPosData);
            unityPosData.hRelative = tmpPosData.hRelative;
            unityPosData.laneId = tmpPosData.laneId;
            unityPosData.laneOffset = tmpPosData.laneOffset;
            unityPosData.roadId = tmpPosData.roadId;
            unityPosData.s = tmpPosData.s;
            unityPosData.junctionId = tmpPosData.junctionId;
        }

        public static void GetLaneInfo(int openDriveIndex, float lookAheadDistance, ref RoadLaneInfoUnityCoordinates laneInfo, LookAheadMode lookAheadMode = LookAheadMode.LaneCenter, int laneId = 0, bool inRoadDrivingDirection = false)
        {
            RoadManagerLibraryCS.GetLaneInfo(openDriveIndex, lookAheadDistance, ref tmpLaneInfo, (int)lookAheadMode, inRoadDrivingDirection);
            laneInfo.position = GetUnityPosition(tmpLaneInfo.pos.x, tmpLaneInfo.pos.y, tmpLaneInfo.pos.z);
            laneInfo.rotation = GetUnityRotation(tmpLaneInfo.heading, tmpLaneInfo.pitch, tmpLaneInfo.roll);
            laneInfo.curvature = -Mathf.Sign(laneId) * tmpLaneInfo.curvature;
            laneInfo.speedLimit = tmpLaneInfo.speed_limit;
            laneInfo.width = tmpLaneInfo.width;
            laneInfo.roadId = tmpLaneInfo.roadId;
            laneInfo.laneId = tmpLaneInfo.laneId;
            laneInfo.laneOffset = tmpLaneInfo.laneOffset;
            laneInfo.s = tmpLaneInfo.s;
            laneInfo.t = tmpLaneInfo.t;
            laneInfo.junctionId = tmpLaneInfo.junctionId;
        }

        public static void GetProbeInfo(int openDriveIndex, float lookAheadDistance, ref RoadProbeInfoUnityCoordinates probeInfo, int lookAheadMode, bool inRoadDrivingDirection = false)
        {
            RoadManagerLibraryCS.GetProbeInfo(openDriveIndex, lookAheadDistance, ref tmpProbeInfo, lookAheadMode, inRoadDrivingDirection);
            probeInfo.roadLaneInfo.position = GetUnityPosition(tmpProbeInfo.laneInfo.pos.x, tmpProbeInfo.laneInfo.pos.y, tmpProbeInfo.laneInfo.pos.z);
            probeInfo.roadLaneInfo.rotation = GetUnityRotation(tmpProbeInfo.laneInfo.heading, tmpProbeInfo.laneInfo.pitch, tmpProbeInfo.laneInfo.roll);
            probeInfo.roadLaneInfo.curvature = tmpProbeInfo.laneInfo.curvature;
            probeInfo.roadLaneInfo.speedLimit = tmpProbeInfo.laneInfo.speed_limit;
            probeInfo.roadLaneInfo.width = tmpProbeInfo.laneInfo.width;
            probeInfo.roadLaneInfo.roadId = tmpProbeInfo.laneInfo.roadId;
            probeInfo.roadLaneInfo.laneId = tmpProbeInfo.laneInfo.laneId;
            probeInfo.roadLaneInfo.laneOffset = tmpProbeInfo.laneInfo.laneOffset;
            probeInfo.roadLaneInfo.s = tmpProbeInfo.laneInfo.s;
            probeInfo.roadLaneInfo.t = tmpProbeInfo.laneInfo.t;
            probeInfo.roadLaneInfo.junctionId = tmpProbeInfo.laneInfo.junctionId;
            probeInfo.relativePosition = GetUnityPosition(tmpProbeInfo.relativePos.x, tmpProbeInfo.relativePos.y, tmpProbeInfo.relativePos.z);
            probeInfo.relativeHeading = tmpProbeInfo.relativeHeading;
        }

#if ESMini
        public static string GetRoadReferencedByLoadedScenario()
        {
            byte[] str = new byte[256];
            return Marshal.PtrToStringAnsi(ESMini.ESMiniLib.SE_GetODRFilename());
        }
#endif

        /// <summary>
        /// Sets the given transform to match the position and rotation of the road user with handle index.
        /// </summary>
        /// <param name="openDriveIndex"></param>
        /// <param name="gameObjTransform">Transform of the road user game object.</param>
        public static void AlignGameObjectToHandle(int openDriveIndex, GameObject go)
        {
            RoadManagerLibraryCS.GetPositionData(openDriveIndex, ref tmpPosData);

            Vector3 pos = GetUnityPosition(tmpPosData);
            Quaternion rot = GetUnityRotation(tmpPosData);

            go.transform.SetPositionAndRotation(pos, rot);
        }

        public static Vector3 GetUnityPosition(float odrX, float odrY, float odrZ)
        {
            return new Vector3(-odrX, odrZ, -odrY);
        }

        public static Vector3 GetUnityPosition(OpenDrivePositionData openDrivePositionData)
        {
            return GetUnityPosition(openDrivePositionData.x, openDrivePositionData.y, openDrivePositionData.z);
        }

        /// <summary>
        /// Input arguments in radians.
        /// </summary>
        /// <param name="heading"></param>
        /// <param name="pitch"></param>
        /// <param name="roll"></param>
        /// <returns></returns>
        public static Quaternion GetUnityRotation(float heading, float pitch, float roll)
        {
            return Quaternion.Euler(pitch * RAD2DEG, 270 - heading * RAD2DEG, -roll * RAD2DEG);
        }

        /// <summary>
        /// Returns the world rotation of the road user with handle index.
        /// </summary>
        /// <param name="openDriveIndex"></param>
        /// <returns></returns>
        public static Quaternion GetUnityRotation(OpenDrivePositionData openDrivePositionData)
        {
            return GetUnityRotation(openDrivePositionData.h, openDrivePositionData.p, openDrivePositionData.r);
        }

        /// <summary>
        /// Returns the OpenDrive world coordinates given a position in Unity's coordinate system.
        /// </summary>
        public static Vector3 GetOpenDrivePosition(Vector3 unityPosition)
        {
            return new Vector3(-unityPosition.x, -unityPosition.z, unityPosition.y);
        }

        /// <summary>
        /// Returns the OpenDRIVE rotation given a rotation in Unity's coordinate system.
        /// </summary>
        public static Vector3 GetOpenDriveRotation(Vector3 unityRotationEuler)
        {
            //return new Vector3(-unityRotationEuler.y * DEG2RAD, unityRotationEuler.x * DEG2RAD, unityRotationEuler.z * DEG2RAD);
            return new Vector3( (270-unityRotationEuler.y) * DEG2RAD, unityRotationEuler.x * DEG2RAD, -unityRotationEuler.z * DEG2RAD);
        }

#endregion

    }

}