using System;
using System.Runtime.InteropServices;
using ESMini;

using Google.Protobuf;

using static Osi3.GroundTruth;

namespace esmini_csharp
{
    class Program
    {
        static void Main(string[] args)
        {
            // initialize esmini
            if (ESMiniLib.SE_Init("../resources/xosc/cut-in.xosc", 0, 1, 0, 0) != 0)
            {
                Console.WriteLine("failed to load scenario");
                return;
            }

            int size = 0;
            while (ESMiniLib.SE_GetQuitFlag() != 1)
            {
                // Step esmini
                ESMiniLib.SE_Step();

                // Get OSI message
                IntPtr int_ptr = ESMiniLib.SE_GetOSIGroundTruth(ref size);
                Byte[] byte_array = new Byte[size];
                Marshal.Copy(int_ptr, byte_array, 0, size);
                Osi3.GroundTruth gt_msg = Osi3.GroundTruth.Parser.ParseFrom(byte_array);

                // Write some info from OSI message
                Console.WriteLine("Time: {0:N3}", gt_msg.Timestamp.Seconds + 1e-9 * gt_msg.Timestamp.Nanos);
                foreach (Osi3.MovingObject o in gt_msg.MovingObject)
                {
                    Console.WriteLine("  Object[{0}], Pos: {1:N2}, {2:N2}, {3:N2}",
                        o.Id.Value, o.Base.Position.X, o.Base.Position.Y, o.Base.Position.Z);
                }
            }
        }
    }
}
