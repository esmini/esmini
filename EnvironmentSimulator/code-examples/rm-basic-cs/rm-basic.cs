using System;
using OpenDRIVE;


namespace esmini_csharp
{
    class Program
    {
        static void PrintInfo(int pos_id)
        {
            RoadLaneInfo laneInfo = new RoadLaneInfo();
            OpenDrivePositionData posData = new OpenDrivePositionData();

            // Get some info
            RoadManagerLibraryCS.GetLaneInfo(pos_id, 0.0f, ref laneInfo, 2, false);
            RoadManagerLibraryCS.GetPositionData(pos_id, ref posData);

            Console.WriteLine("current pos: s {0:N2} laneId {1} offset {2:N2} x {3:N2} y {4:N2} z {5:N2}",
                posData.s, laneInfo.laneId, laneInfo.laneOffset, posData.x, posData.y, posData.z);
        }

        static void Main(string[] args)
        {
            if (RoadManagerLibraryCS.Init("../../../../resources/xodr/straight_500m.xodr") != 0)
            {
                Console.WriteLine("Failed to load OpenDRIVE file");
                return;
            }

            // Create a position object
            int p0 = RoadManagerLibraryCS.CreatePosition();
            float x = 20.0f;
            float y = -10.0f;

            // Any driving lane (default)
            // see enum roadmanager::lane::LANE_TYPE_ANY_DRIVING
            RoadManagerLibraryCS.SetSnapLaneTypes(p0, 1966082);
            RoadManagerLibraryCS.SetWorldXYHPosition(p0, x, y, 0.0f);
            PrintInfo(p0);

            // any lanetype (all lanes)
            // see enum roadmanager::lane::LANE_TYPE_ANY
            RoadManagerLibraryCS.SetSnapLaneTypes(p0, -1);
            RoadManagerLibraryCS.SetWorldXYHPosition(p0, x, y, 0.0f);
            PrintInfo(p0);

            // try out get lane methods
            int lane_id = 0;
            int nlanes = RoadManagerLibraryCS.GetRoadNumberOfLanes(1, 10.0f, -1);
            Console.WriteLine("total nr lanes: {0}", nlanes);
            for (int i = 0; i < nlanes; i++)
            {
                Console.WriteLine("lane {0}: {1}, {2}", i, RoadManagerLibraryCS.GetLaneIdByIndex(1, i, 10.0f, -1, out lane_id), lane_id);
            }

            nlanes = RoadManagerLibraryCS.GetRoadNumberOfLanes(1, 10.0f, 64);
            Console.WriteLine("nr border lanes: {0}", nlanes);
            for (int i = 0; i < nlanes; i++)
            {
                Console.WriteLine("lane {0}: {1}, {2}", i, RoadManagerLibraryCS.GetLaneIdByIndex(1, i, 10.0f, 64, out lane_id), lane_id);
            }

            nlanes = RoadManagerLibraryCS.GetRoadNumberOfLanes(1, 10.0f, 1966594);
            Console.WriteLine("any drivable nr lanes: {0}", nlanes);
            for (int i = 0; i < nlanes; i++)
            {
                Console.WriteLine("lane {0}: {1}, {2}", i, RoadManagerLibraryCS.GetLaneIdByIndex(1, i, 10.0f, 1966594, out lane_id), lane_id);
            }

            nlanes = RoadManagerLibraryCS.GetRoadNumberOfDrivableLanes(1, 10.0f);
            Console.WriteLine("nr drivable lanes: {0}", nlanes);
            for (int i = 0; i < nlanes; i++)
            {
                Console.WriteLine("lane {0}: {1}, {2}", i, RoadManagerLibraryCS.GetDrivableLaneIdByIndex(1, i, 1000.0f, out lane_id), lane_id);
            }

            float width = 0.0f;
            if (RoadManagerLibraryCS.GetLaneWidth(p0, 1, out width) == 0)
            {
                Console.WriteLine("lane width at current pos: {0:N2}", width);
            }
            else
            {
                Console.WriteLine("error getting lane width at current pos");
            }
            if (RoadManagerLibraryCS.GetLaneWidth(p0, -1, out width) == 0)
            {
                Console.WriteLine("total lane width at current pos: {0:N2}", width);
            }
            else
            {
                Console.WriteLine("error getting total lane width at current pos");
            }

            if (RoadManagerLibraryCS.GetLaneWidthByRoadId(1, -1, 10.0f, out width) == 0)
            {
                Console.WriteLine("lane width at road 1 s=10.0: {0:N2}", width);
            }
            else
            {
                Console.WriteLine("error getting total lane width at road 1 s=10.0");
            }

            if (RoadManagerLibraryCS.GetLaneWidthByRoadId(1, -1, 501.0f, out width) == -1)
            {
                Console.WriteLine("correctly got error getting total lane width at road 1 s=501.0 (out of range)");
            }

            RoadManagerLibraryCS.Close();
        }
    }
}
