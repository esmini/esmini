using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Google.Protobuf;
using ESMini;

#if _USE_OSI
using static Osi3.GroundTruth;
#endif

namespace ESMiniTest
{
    class Program
    {
        static int successfulAsserts = 0;
        static List<string> successMessages = new List<string>();
        static string failureMessage = "";

        private static ESMiniLib.ParameterDeclarationCallback parameterCallback;
        static void MyParameterCallback(IntPtr theMagicNumber)
        {
            // modify a parameter value during initialization
            ESMiniLib.SE_SetParameterDouble("HeadwayTime_Brake", 0.75);
            ASSERT((int)theMagicNumber == 42, "Received correct magic number in callback");
        }

        private static ESMiniLib.ObjectCallback objectCallback;
        static void MyObjectCallback(IntPtr objState, IntPtr sValue)
        {
            SE_ScenarioObjectState state = (SE_ScenarioObjectState)Marshal.PtrToStructure(objState, typeof(SE_ScenarioObjectState));
            // modify object x position
            ESMiniLib.SE_ReportObjectRoadPos(0, 0, -3, 0.0, (double)sValue);
        }

        static bool NEAR(double a, double b)
        {
            return Math.Abs(a - b) < 1e-6;
        }

        static void PublishResultAndQuit(bool success)
        {
            Console.WriteLine("\n--- Successful Checks ---");
            for (int i = 0; i < successMessages.Count; i++)
            {
                Console.WriteLine($"  {i + 1,3}: OK: {successMessages[i]}");
            }

            if (success)
            {
                Console.WriteLine($"\n{successfulAsserts} checks passed\nTest OK");
                Environment.Exit(0);
            }
            else
            {
                Console.WriteLine($"Failed check: {failureMessage}\nTest FAILED");
                Environment.Exit(1);
            }
        }

        static int Main(string[] args)
        {
            try
            {
                // Basic settings (No Init required)
                ESMiniLib.SE_SetOption("disable_stdout");
                ESMiniLib.SE_SetSeed(12345);
                ASSERT(ESMiniLib.SE_GetSeed() == 12345, "Seed set to 12345");

                // Options
                ESMiniLib.SE_SetLogFilePath("esmini_test.log");
                ESMiniLib.SE_LogMessage("This is a test log message.");
                ASSERT(System.IO.File.Exists("esmini_test.log"), "Log file should exist after setting log file path");

                ASSERT(ESMiniLib.SE_SetOption("kalle") == -1, "Option 'kalle' not valid/supported");
                ASSERT(ESMiniLib.SE_SetOption("disable_controllers") == 0, "Option '--disable_controllers' valid");
                bool optionSet = ESMiniLib.SE_GetOptionSet("disable_controllers");
                ASSERT(optionSet, "Option 'disable_controllers' set");

                // Create the delegate instance and register it before calling SE_Init
                int magicNumber = 42; // Just an example of user data you might want to pass
                parameterCallback = new ESMiniLib.ParameterDeclarationCallback(MyParameterCallback);
                ESMiniLib.SE_RegisterParameterDeclarationCallback(parameterCallback, (IntPtr)magicNumber);

#if _USE_OSI
                ESMiniLib.SE_EnableOSIFile("esmini_test.osi");
#endif
                // Init
                ASSERT(ESMiniLib.SE_Init("../resources/xosc/cut-in.xosc", 0, 0, 0, 0) == 0, "Initialize the scenario");
                //ASSERT(ESMiniLib.SE_Init("../../../../../resources/xosc/cut-in.xosc", 0, 0, 0, 0) == 0, "Initialize the scenario");

                RunPostInitTests();

                PublishResultAndQuit(true);
                return 0;
            }
            catch (Exception ex)
            {
                failureMessage = $"An error occurred: {ex.Message}\nStack trace: {ex.StackTrace}";
                PublishResultAndQuit(false);
                return 1;
            }
        }

        static void RunPostInitTests()
        {
#if _USE_OSI
            ASSERT(System.IO.File.Exists("esmini_test.osi"), "OSI file created");
#endif

            ASSERT(NEAR(ESMiniLib.SE_GetSimulationTime(), 0.0), "Initial simulation time is 0.0");

            ESMiniLib.SE_StepDT(0.1);
            ASSERT(NEAR(ESMiniLib.SE_GetSimulationTime(), 0.1), "Simulation time is updated to 0.1");

            // Objects
            ASSERT(ESMiniLib.SE_GetNumberOfObjects() == 2, "Found expected (2) nr of objects");
            ASSERT(ESMiniLib.SE_GetId(1) == 1, "Object 1 has ID 1");
            SE_ScenarioObjectState obj_state;
            ASSERT(ESMiniLib.SE_GetObjectState(0, out obj_state) == 0, "Get state of object 0");
            ASSERT(NEAR(obj_state.length, 5.04), "Object 0 length is 5.04");
            ASSERT(NEAR(obj_state.x, 8.1845103), "Object 0 x position");
            ASSERT(NEAR(obj_state.y, 52.9701459), "Object 0 y position");

            ASSERT(ESMiniLib.SE_GetObjectState(1, out obj_state) == 0, "Get state of object 1");
            ASSERT(NEAR(obj_state.objectType, 1), "Object 1 type is 1 (Vehicle)");
            ASSERT(NEAR(obj_state.x, 4.5095786), "Object 1 x position");
            ASSERT(NEAR(obj_state.y, 24.9846439), "Object 1 y position");

            // Parameters
            ASSERT(ESMiniLib.SE_GetNumberOfParameters() == 7, "Find 7 parameters in the scenario");
            int type;
            string pname = Marshal.PtrToStringAnsi(ESMiniLib.SE_GetParameterName(6, out type));
            ASSERT(Marshal.PtrToStringAnsi(ESMiniLib.SE_GetParameterName(6, out type)) == "HeadwayTime_Brake", "Parameter name is 'HeadwayTime_Brake'");
            SE_Parameter param=new SE_Parameter();
            param.name = Marshal.StringToHGlobalAnsi("HeadwayTime_Brake");
            param.value = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(double)));
            ASSERT(ESMiniLib.SE_GetParameter(ref param) == 0, "Get parameter 'HeadwayTime_Brake'");
            ASSERT(NEAR((double)Marshal.PtrToStructure(param.value, typeof(double)), 0.75), "Brake TimeHeadway == 0.75");
            double speedFactor = 0.0;
            ASSERT(ESMiniLib.SE_GetParameterDouble("TargetSpeedFactor", out speedFactor) == 0, "Get 'TargetSpeedFactor'");
            ASSERT(NEAR(speedFactor, 1.2), "TargetSpeedFactor == 1.2");

            // Road Info
            SE_RoadInfo roadInfo;
            ASSERT(ESMiniLib.SE_GetRoadInfoAtDistance(1, 10.0, out roadInfo, 0, true) == 0, "Get Road Info for Object 1 at distance=0.0");
            ASSERT(NEAR(roadInfo.s, 35.000298), "Road Info s value is 10.0");
            ASSERT(roadInfo.roadId == 0, "Road ID is 0");
            ASSERT(roadInfo.laneId == -2, "Lane ID is -2");

            // Check Quit Flag
            ASSERT(ESMiniLib.SE_GetQuitFlag() == 0, "Quit Flag is 0");

            // Register object callback and test modification
            objectCallback = new ESMiniLib.ObjectCallback(MyObjectCallback);
            ESMiniLib.SE_RegisterObjectCallback(0, objectCallback, (IntPtr)120.0);

            ESMiniLib.SE_StepDT(0.1);
            ESMiniLib.SE_GetObjectState(0, out obj_state);
            ASSERT(NEAR(obj_state.s, 120.0), "Object 0 s position modified by callback");

#if _USE_OSI
            int size = 0;
            IntPtr int_ptr = ESMiniLib.SE_GetOSIGroundTruth(out size);
            Byte[] byte_array = new Byte[size];
            Marshal.Copy(int_ptr, byte_array, 0, size);
            Osi3.GroundTruth gt_msg = Osi3.GroundTruth.Parser.ParseFrom(byte_array);
            ASSERT(gt_msg.MovingObject[1].Id.Value == 37, "Object 1 OSI id is 37");
            ASSERT(NEAR(gt_msg.MovingObject[1].Base.Position.Y, 30.0353193), "Object 1 OSI Y position");
#endif
        }

        static void ASSERT(bool condition, string message)
        {
            if (!condition)
            {
                failureMessage = message;
                PublishResultAndQuit(false);
            }
            else
            {
                successMessages.Add(message);
                successfulAsserts++;
            }
        }
    }
}
