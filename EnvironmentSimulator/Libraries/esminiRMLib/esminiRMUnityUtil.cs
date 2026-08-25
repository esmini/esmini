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

// Disable naming styles warning, the below are esmini's naming conventions,
// which might differ from the consuming project's naming conventions
#pragma warning disable IDE1006

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
        public double hRelative;
        public int roadId;
        public int laneId;
        public double laneOffset;
        public double s;
        public int junctionId; // junction id is -1 if the position is not in a junction
    };

    public struct RoadLaneInfoUnityCoordinates
    {
        public Vector3 position;
        public Quaternion rotation;
        public double width;            // Lane width
        public double curvature;        // curvature (1/radius), >0 for left curves, <0 for right curves
        public double speedLimit;       // road speed limit
        public int roadId;             // road ID
        public int laneId;             // lane ID
        public double laneOffset;       // lane offset (lateral distance from lane center)
        public double s;                // s (longitudinal distance along reference line)
        public double t;                // t (lateral distance from reference line)
        public int junctionId;         // junction id is -1 if the position is not in a junction
    };

    public struct RoadProbeInfoUnityCoordinates
    {
        public RoadLaneInfoUnityCoordinates roadLaneInfo;
        public Vector3 relativePosition; // probe position, relative vehicle (pivot position object) coordinate system
        public double relativeHeading;    // heading angle to steering target from and relatove to vehicle (pivot position)    };
    };

    public static class OpenDriveUtil
    {
        public enum LookAheadMode
        {
            LaneCenter,
            RoadCenter,
            CurrentLateralOffset
        }

        private static OpenDrivePositionData s_tmpPosData = new OpenDrivePositionData();
        private static RoadLaneInfo s_tmpLaneInfo = new RoadLaneInfo();
        private static RoadProbeInfo s_tmpProbeInfo = new RoadProbeInfo();
        private const float Rad2DegFloat = Mathf.Rad2Deg;
        private const float Deg2RadFloat = Mathf.Deg2Rad;
        private const double Rad2DegDouble = 180.0 / System.Math.PI;
        private const double Deg2RadDouble = System.Math.PI / 180.0;

        /// <summary>
        /// Set position from world coordinates in the Unity coordinate system. Returns the sum of the return values from RoadManagerLibraryCS.SetWorldXYHPosition, RoadManagerLibraryCS.GetPositionData, and RoadManagerLibraryCS.SetLanePosition. Please see those methods for details.
        /// </summary>
        public static int SetWorldPosition(int openDriveIndex, Vector3 position)
        {
            Vector3 odrPos = GetOpenDrivePosition(position);
            int retVal = RoadManagerLibraryCS.SetWorldXYHPosition(openDriveIndex, odrPos.x, odrPos.y, 0);
            retVal += RoadManagerLibraryCS.GetPositionData(openDriveIndex, ref s_tmpPosData);
            retVal += RoadManagerLibraryCS.SetLanePosition(openDriveIndex, s_tmpPosData.roadId, s_tmpPosData.laneId, s_tmpPosData.laneOffset, s_tmpPosData.s, true);
            return retVal;
        }

        /// <summary>
        /// Set position and rotation from world coordinates in the Unity coordinate system. Please see RoadManagerLibraryCS.SetWorldPosition for return values.
        /// </summary>
        public static int SetWorldPosition(int openDriveIndex, Vector3 position, Vector3 rotationEuler)
        {
            Vector3 odrPos = GetOpenDrivePosition(position);
            Vector3 odrRot = GetOpenDriveRotation(rotationEuler);
            return RoadManagerLibraryCS.SetWorldPosition(openDriveIndex, odrPos.x, odrPos.y, odrPos.z, odrRot.x, odrRot.y, odrRot.z);
        }

        /// <summary>
        /// Returns the world position and rotation of the road user with handle index.
        /// </summary>
        /// <param name="openDriveIndex"></param>
        /// <returns></returns>
        public static WorldPose GetWorldPose(int openDriveIndex)
        {
            WorldPose pose = new WorldPose();
            RoadManagerLibraryCS.GetPositionData(openDriveIndex, ref s_tmpPosData);
            pose.position = GetUnityPosition(s_tmpPosData);
            pose.rotation = GetUnityRotation(s_tmpPosData);

            return pose;
        }

        /// <summary>
        /// Please see RoadManagerLibraryCS.GetPositionData for details.
        /// </summary>
        /// <param name="openDriveIndex"></param>
        /// <param name="unityPosData"></param>
        /// <returns>Returns >=0 if successful, <0 if not. See Position::ReturnCode RoadManager.hpp for details.</returns>
        public static int GetOpenDrivePositionDataUnityCoordinates(int openDriveIndex, ref OpenDrivePositionDataUnityCoordinates unityPosData)
        {
            int retVal = RoadManagerLibraryCS.GetPositionData(openDriveIndex, ref s_tmpPosData);
            if (retVal >= 0)
            {
                unityPosData.position = GetUnityPosition(s_tmpPosData);
                unityPosData.rotation = GetUnityRotation(s_tmpPosData);
                unityPosData.hRelative = s_tmpPosData.hRelative;
                unityPosData.laneId = s_tmpPosData.laneId;
                unityPosData.laneOffset = s_tmpPosData.laneOffset;
                unityPosData.roadId = s_tmpPosData.roadId;
                unityPosData.s = s_tmpPosData.s;
                unityPosData.junctionId = s_tmpPosData.junctionId;
            }
            return retVal;
        }

        /// <summary>
        /// Please see RoadManagerLibraryCS.GetLaneInfo for details.
        /// </summary>
        /// <param name="openDriveIndex"></param>
        /// <param name="lookAheadDistance"></param>
        /// <param name="laneInfo"></param>
        /// <param name="lookAheadMode"></param>
        /// <param name="laneId"></param>
        /// <param name="inRoadDrivingDirection"></param>
        /// <returns>Returns >=0 if successful, <0 if not. See Position::ReturnCode RoadManager.hpp for details.</returns>
        public static int GetLaneInfo(int openDriveIndex, double lookAheadDistance, ref RoadLaneInfoUnityCoordinates laneInfo,
            LookAheadMode lookAheadMode = LookAheadMode.LaneCenter, bool inRoadDrivingDirection = false)
        {
            int retVal = RoadManagerLibraryCS.GetLaneInfo(openDriveIndex, lookAheadDistance, ref s_tmpLaneInfo, (int)lookAheadMode, inRoadDrivingDirection);
            if (retVal >= 0)
            {
                if (double.IsNaN(s_tmpLaneInfo.pitch))
                {
                    s_tmpLaneInfo.pitch = 0;
                }
                laneInfo.position = GetUnityPosition(s_tmpLaneInfo.pos.x, s_tmpLaneInfo.pos.y, s_tmpLaneInfo.pos.z);
                laneInfo.rotation = GetUnityRotation(s_tmpLaneInfo.heading, s_tmpLaneInfo.pitch, s_tmpLaneInfo.roll);
                laneInfo.speedLimit = s_tmpLaneInfo.speedLimit;
                laneInfo.width = s_tmpLaneInfo.width;
                laneInfo.roadId = s_tmpLaneInfo.roadId;
                laneInfo.laneId = s_tmpLaneInfo.laneId;
                laneInfo.laneOffset = s_tmpLaneInfo.laneOffset;
                laneInfo.s = s_tmpLaneInfo.s;
                laneInfo.t = s_tmpLaneInfo.t;
                laneInfo.junctionId = s_tmpLaneInfo.junctionId;
                if (inRoadDrivingDirection)
                {
                    laneInfo.curvature = s_tmpLaneInfo.curvature;
                }
                else
                {
                    laneInfo.curvature = -Mathf.Sign(s_tmpLaneInfo.laneId) * s_tmpLaneInfo.curvature;
                }
            }
            return retVal;
        }

        /// <summary>
        /// Please see RoadManagerLibraryCS.GetProbeInfo for details.
        /// </summary>
        /// <param name="openDriveIndex"></param>
        /// <param name="lookAheadDistance"></param>
        /// <param name="probeInfo"></param>
        /// <param name="lookAheadMode"></param>
        /// <param name="inRoadDrivingDirection"></param>
        /// <returns>Returns >=0 if successful, <0 if not. See Position::ReturnCode RoadManager.hpp for details.</returns>
        public static int GetProbeInfo(int openDriveIndex, double lookAheadDistance, ref RoadProbeInfoUnityCoordinates probeInfo, int lookAheadMode, bool inRoadDrivingDirection = false)
        {
            int retVal = RoadManagerLibraryCS.GetProbeInfo(openDriveIndex, lookAheadDistance, ref s_tmpProbeInfo, lookAheadMode, inRoadDrivingDirection);
            if (retVal >= 0)
            {
                probeInfo.roadLaneInfo.position = GetUnityPosition(s_tmpProbeInfo.laneInfo.pos.x, s_tmpProbeInfo.laneInfo.pos.y, s_tmpProbeInfo.laneInfo.pos.z);
                probeInfo.roadLaneInfo.rotation = GetUnityRotation(s_tmpProbeInfo.laneInfo.heading, s_tmpProbeInfo.laneInfo.pitch, s_tmpProbeInfo.laneInfo.roll);
                probeInfo.roadLaneInfo.curvature = s_tmpProbeInfo.laneInfo.curvature;
                probeInfo.roadLaneInfo.speedLimit = s_tmpProbeInfo.laneInfo.speedLimit;
                probeInfo.roadLaneInfo.width = s_tmpProbeInfo.laneInfo.width;
                probeInfo.roadLaneInfo.roadId = s_tmpProbeInfo.laneInfo.roadId;
                probeInfo.roadLaneInfo.laneId = s_tmpProbeInfo.laneInfo.laneId;
                probeInfo.roadLaneInfo.laneOffset = s_tmpProbeInfo.laneInfo.laneOffset;
                probeInfo.roadLaneInfo.s = s_tmpProbeInfo.laneInfo.s;
                probeInfo.roadLaneInfo.t = s_tmpProbeInfo.laneInfo.t;
                probeInfo.roadLaneInfo.junctionId = s_tmpProbeInfo.laneInfo.junctionId;
                probeInfo.relativePosition = GetUnityPosition(s_tmpProbeInfo.relativePos.x, s_tmpProbeInfo.relativePos.y, s_tmpProbeInfo.relativePos.z);
                probeInfo.relativeHeading = s_tmpProbeInfo.relativeHeading;
            }
            return retVal;
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
            RoadManagerLibraryCS.GetPositionData(openDriveIndex, ref s_tmpPosData);

            Vector3 pos = GetUnityPosition(s_tmpPosData);
            Quaternion rot = GetUnityRotation(s_tmpPosData);

            go.transform.SetPositionAndRotation(pos, rot);
        }

        public static Vector3 GetUnityPosition(double odrX, double odrY, double odrZ)
        {
            return new Vector3((float)-odrX, (float)odrZ, (float)-odrY);
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
        public static Quaternion GetUnityRotation(double heading, double pitch, double roll)
        {
            return Quaternion.Euler((float)(pitch * Rad2DegDouble), (float)(270 - heading * Rad2DegDouble), (float)(-roll * Rad2DegDouble));
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
            return Quaternion.Euler(pitch * Rad2DegFloat, 270 - heading * Rad2DegFloat, -roll * Rad2DegFloat);
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
            return new Vector3((270 - unityRotationEuler.y) * Deg2RadFloat, unityRotationEuler.x * Deg2RadFloat, -unityRotationEuler.z * Deg2RadFloat);
        }

    }

#pragma warning restore IDE1006

}