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
            if (RoadManagerLibraryCS.Init("../resources/xodr/straight_500m.xodr") != 0)
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


            RoadManagerLibraryCS.Close();
        }
    }
}
