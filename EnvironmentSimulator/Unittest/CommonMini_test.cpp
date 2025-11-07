
#include <gtest/gtest.h>

#include "CommonMini.hpp"
#include "logger.hpp"
#include "esminiLib.hpp"
#include "Config.hpp"

struct Coordinate2D
{
    double x;
    double y;
};

class Local2Global : public ::testing::TestWithParam<std::tuple<Coordinate2D, Coordinate2D, double, Coordinate2D>>
{
public:
    Coordinate2D global_host, target_host, target_global, result;
    double       theta;  // angle between the coordinate systems
};

TEST_P(Local2Global, RotationAndTranslation)
{
    target_host = std::get<0>(GetParam());
    global_host = std::get<1>(GetParam());
    theta       = std::get<2>(GetParam());
    result      = std::get<3>(GetParam());

    Local2GlobalCoordinates(target_global.x, target_global.y, global_host.x, global_host.y, theta, target_host.x, target_host.y);

    EXPECT_DOUBLE_EQ(result.x, target_global.x);
    EXPECT_DOUBLE_EQ(result.y, target_global.y);
}

INSTANTIATE_TEST_SUITE_P(CommonMini,
                         Local2Global,
                         ::testing::Values(std::make_tuple(Coordinate2D{0, 1}, Coordinate2D{1, 1}, -M_PI / 2, Coordinate2D{2, 1}),
                                           std::make_tuple(Coordinate2D{1, 0}, Coordinate2D{1, 1}, -M_PI / 2, Coordinate2D{1, 0})));

TEST(VectorOperations, TestIsPointWithinSectorBetweenTwoLines)
{
    // Parallel lines
    SE_Vector l0p0   = SE_Vector(1.0, 0.1);
    SE_Vector l0p1   = SE_Vector(1.0, 2.1);
    SE_Vector l1p0   = SE_Vector(2.0, -0.1);
    SE_Vector l1p1   = SE_Vector(2.0, 10.0);
    SE_Vector p      = SE_Vector(1.8, 1.8);
    double    factor = 0.0;

    EXPECT_EQ(IsPointWithinSectorBetweenTwoLines(p, l0p0, l0p1, l1p0, l1p1, factor), true);
    EXPECT_NEAR(factor, 0.8, 1E-3);

    l0p0.Set(-5.0, -1.0);
    l0p1.Set(-8.0, 1.0);
    l1p0.Set(-9.0, -1.0);
    l1p1.Set(-12.0, 1.0);
    p.Set(-6.0, -1.0);

    EXPECT_EQ(IsPointWithinSectorBetweenTwoLines(p, l0p0, l0p1, l1p0, l1p1, factor), true);
    EXPECT_NEAR(factor, -0.25, 1E-3);

    // non parallel lines
    l0p0.Set(1.0, 0.0);
    l0p1.Set(2.0, 1.0);
    l1p0.Set(-1.0, 0.0);
    l1p1.Set(-2.0, 1.0);
    p.Set(0.0, 0.0);

    EXPECT_EQ(IsPointWithinSectorBetweenTwoLines(p, l0p0, l0p1, l1p0, l1p1, factor), true);
    EXPECT_NEAR(factor, -0.5, 1E-3);

    // non parallel lines
    l0p0.Set(1.0, 0.0);
    l0p1.Set(2.0, 1.0);
    l1p0.Set(-1.0, 0.0);
    l1p1.Set(-2.0, 1.0);
    p.Set(-3.0, 0.4);

    EXPECT_EQ(IsPointWithinSectorBetweenTwoLines(p, l0p0, l0p1, l1p0, l1p1, factor), false);
    p.Set(0.2, -2.0);
    EXPECT_EQ(IsPointWithinSectorBetweenTwoLines(p, l0p0, l0p1, l1p0, l1p1, factor), true);
    EXPECT_NEAR(factor, 0.6, 1E-3);
    p.Set(1.5, -2.0);
    EXPECT_EQ(IsPointWithinSectorBetweenTwoLines(p, l0p0, l0p1, l1p0, l1p1, factor), false);
    EXPECT_NEAR(factor, 0.354, 1E-3);
}

TEST(VectorOperations, TestProjectPointOnVector)
{
    double v_result[2] = {0.0, 0.0};

    ProjectPointOnVector2D(5.0, 1.0, 0.1, 0.0, v_result[0], v_result[1]);
    EXPECT_NEAR(v_result[0], 5.0, 1E-5);
    EXPECT_NEAR(v_result[1], 0.0, 1E-5);

    ProjectPointOnVector2D(0.1, -1.0, 4.0, 0.0, v_result[0], v_result[1]);
    EXPECT_NEAR(v_result[0], 0.1, 1E-5);
    EXPECT_NEAR(v_result[1], 0.0, 1E-5);

    ProjectPointOnVector2D(-1.0, -1.0, 2.0, 0.0, v_result[0], v_result[1]);
    EXPECT_NEAR(v_result[0], -1.0, 1E-5);
    EXPECT_NEAR(v_result[1], 0.0, 1E-5);

    ProjectPointOnVector2D(-1.0, -1.0, 0.0, 0.0, v_result[0], v_result[1]);
    EXPECT_NEAR(v_result[0], 0.0, 1E-5);
    EXPECT_NEAR(v_result[1], 0.0, 1E-5);

    ProjectPointOnVector2D(1.0, 0.0, 5.0, 5.0, v_result[0], v_result[1]);
    EXPECT_NEAR(v_result[0], 0.5, 1E-5);
    EXPECT_NEAR(v_result[1], 0.5, 1E-5);

    ProjectPointOnVector2D(1.0, 0.0, -5.0, 5.0, v_result[0], v_result[1]);
    EXPECT_NEAR(v_result[0], 0.5, 1E-5);
    EXPECT_NEAR(v_result[1], -0.5, 1E-5);

    ProjectPointOnVector2D(2.5, -10.0, -3.0, 5.0, v_result[0], v_result[1]);
    EXPECT_NEAR(v_result[0], 5.07353, 1E-5);
    EXPECT_NEAR(v_result[1], -8.45588, 1E-5);
}

TEST(VectorOperations, TestProjectPointOnVectorSignedLength)
{
    // https://www.desmos.com/calculator/wu0xiyqcnj

    double v_result[2]   = {0.0, 0.0};
    double signed_length = 0.0;

    signed_length = ProjectPointOnVector2DSignedLength(5.0, 1.0, 0.1, 0.0, v_result[0], v_result[1]);
    EXPECT_NEAR(v_result[0], 5.0, 1E-5);
    EXPECT_NEAR(v_result[1], 0.0, 1E-5);
    EXPECT_NEAR(signed_length, 5.0, 1E-5);

    signed_length = ProjectPointOnVector2DSignedLength(0.1, -1.0, 4.0, 0.0, v_result[0], v_result[1]);
    EXPECT_NEAR(v_result[0], 0.1, 1E-5);
    EXPECT_NEAR(v_result[1], 0.0, 1E-5);
    EXPECT_NEAR(signed_length, 0.1, 1E-5);

    signed_length = ProjectPointOnVector2DSignedLength(-1.0, -1.0, 2.0, 0.0, v_result[0], v_result[1]);
    EXPECT_NEAR(v_result[0], -1.0, 1E-5);
    EXPECT_NEAR(v_result[1], 0.0, 1E-5);
    EXPECT_NEAR(signed_length, -1.0, 1E-5);

    // zero vector, undefined
    signed_length = ProjectPointOnVector2DSignedLength(-1.0, -1.0, 0.0, 0.0, v_result[0], v_result[1]);
    EXPECT_NEAR(v_result[0], 0.0, 1E-5);
    EXPECT_NEAR(v_result[1], 0.0, 1E-5);
    EXPECT_NEAR(signed_length, 0.0, 1E-5);

    signed_length = ProjectPointOnVector2DSignedLength(1.0, 0.0, 5.0, 5.0, v_result[0], v_result[1]);
    EXPECT_NEAR(v_result[0], 0.5, 1E-5);
    EXPECT_NEAR(v_result[1], 0.5, 1E-5);
    EXPECT_NEAR(signed_length, 0.70711, 1E-5);

    signed_length = ProjectPointOnVector2DSignedLength(1.0, 0.0, -5.0, 5.0, v_result[0], v_result[1]);
    EXPECT_NEAR(v_result[0], 0.5, 1E-5);
    EXPECT_NEAR(v_result[1], -0.5, 1E-5);
    EXPECT_NEAR(signed_length, -0.70711, 1E-5);

    signed_length = ProjectPointOnVector2DSignedLength(2.5, -10.0, -3.0, 5.0, v_result[0], v_result[1]);
    EXPECT_NEAR(v_result[0], 5.07353, 1E-5);
    EXPECT_NEAR(v_result[1], -8.45588, 1E-5);
    EXPECT_NEAR(signed_length, -9.86117, 1E-5);
}

TEST(MatrixOperations, TestMatrixInvert)
{
    double m[3][3] = {{1.0, 0.0, 2.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}, m_out[3][3];

    InvertMatrix3(m, m_out);

    EXPECT_NEAR(m_out[0][0], 1.0, 1E-5);
    EXPECT_NEAR(m_out[0][1], 0.0, 1E-5);
    EXPECT_NEAR(m_out[0][2], -2.0, 1E-5);
    EXPECT_NEAR(m_out[1][0], 0.0, 1E-5);
    EXPECT_NEAR(m_out[1][1], 1.0, 1E-5);
    EXPECT_NEAR(m_out[1][2], 0.0, 1E-5);
    EXPECT_NEAR(m_out[2][0], 0.0, 1E-5);
    EXPECT_NEAR(m_out[2][1], 0.0, 1E-5);
    EXPECT_NEAR(m_out[2][2], 1.0, 1E-5);

    double m2[3][3] = {{1.0, 2.0, 3.0}, {3.0, 2.0, 1.0}, {2.0, 1.0, 3.0}};
    InvertMatrix3(m2, m_out);

    EXPECT_NEAR(m_out[0][0], -0.416667, 1E-5);
    EXPECT_NEAR(m_out[0][1], 0.25, 1E-5);
    EXPECT_NEAR(m_out[0][2], 0.333333, 1E-5);
    EXPECT_NEAR(m_out[1][0], 0.583333, 1E-5);
    EXPECT_NEAR(m_out[1][1], 0.25, 1E-5);
    EXPECT_NEAR(m_out[1][2], -0.666667, 1E-5);
    EXPECT_NEAR(m_out[2][0], 0.0833333, 1E-5);
    EXPECT_NEAR(m_out[2][1], -0.25, 1E-5);
    EXPECT_NEAR(m_out[2][2], 0.333333, 1E-5);

    double m3[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    MultMatrixMatrix3d(m2, m_out, m3);
    EXPECT_NEAR(m3[0][0], 1.0, 1E-5);
    EXPECT_NEAR(m3[0][1], 0.0, 1E-5);
    EXPECT_NEAR(m3[0][2], 0.0, 1E-5);
    EXPECT_NEAR(m3[1][0], 0.0, 1E-5);
    EXPECT_NEAR(m3[1][1], 1.0, 1E-5);
    EXPECT_NEAR(m3[1][2], 0.0, 1E-5);
    EXPECT_NEAR(m3[2][0], 0.0, 1E-5);
    EXPECT_NEAR(m3[2][1], 0.0, 1E-5);
    EXPECT_NEAR(m3[2][2], 1.0, 1E-5);
}

TEST(LinearAlgebra, TestAngleBetweenVectors)
{
    double v1[2] = {1.0, 0.0};
    double v2[2] = {1.0, 0.2};
    EXPECT_NEAR(GetAngleBetweenVectors(v1[0], v1[1], v2[0], v2[1]), 0.197, 1E-3);

    v1[0] = 0.0;
    v1[1] = 10.0;
    v2[0] = 0.0;
    v2[1] = -10.0;
    EXPECT_NEAR(GetAngleBetweenVectors(v1[0], v1[1], v2[0], v2[1]), M_PI, 1E-3);

    v1[0] = -3.0;
    v1[1] = 1.0;
    v2[0] = 1.0;
    v2[1] = 3.0;
    EXPECT_NEAR(GetAngleBetweenVectors(v1[0], v1[1], v2[0], v2[1]), M_PI_2, 1E-3);

    v1[0] = 1.0;
    v1[1] = 3.0;
    v2[0] = -3.0;
    v2[1] = 1.0;
    EXPECT_NEAR(GetAngleBetweenVectors(v1[0], v1[1], v2[0], v2[1]), M_PI_2, 1E-3);

    v1[0] = -5.0;
    v1[1] = 1.0;
    v2[0] = 1.0;
    v2[1] = -0.6;
    EXPECT_NEAR(GetAngleBetweenVectors(v1[0], v1[1], v2[0], v2[1]), 2.798, 1E-3);
}

TEST(ProgramOptions, TestConfigOptionPostprocessing)
{
    {
        const char*            args[] = {"esmini", "--osc", "../../../resources/xosc/cut-in_simple.xosc", "--osc_str", "osc_str_value"};
        esmini::common::Config config("esmini", sizeof(args) / sizeof(char*), const_cast<char**>(args));
        auto [argc, argv] = config.Load();
        EXPECT_EQ(argc, 3);
        EXPECT_EQ(strcmp(argv[0], "esmini"), 0);
        EXPECT_EQ(strcmp(argv[1], "--osc_str"), 0);
        EXPECT_EQ(strcmp(argv[2], "osc_str_value"), 0);
    }
    {
        const char*            args[] = {"esmini", "--osc_str", "osc_str_value", "--osc", "../../../resources/xosc/cut-in_simple.xosc"};
        esmini::common::Config config("esmini", sizeof(args) / sizeof(char*), const_cast<char**>(args));
        auto [argc, argv] = config.Load();
        EXPECT_EQ(argc, 3);
        EXPECT_EQ(strcmp(argv[0], "esmini"), 0);
        EXPECT_EQ(strcmp(argv[1], "--osc"), 0);
        EXPECT_EQ(strcmp(argv[2], "../../../resources/xosc/cut-in_simple.xosc"), 0);
    }
    {
        const char* args[] = {"esmini", "--window", "60", "60", "800", "400", "--headless", "--osc", "../../../resources/xosc/cut-in_simple.xosc"};
        esmini::common::Config config("esmini", sizeof(args) / sizeof(char*), const_cast<char**>(args));
        auto [argc, argv] = config.Load();
        EXPECT_EQ(argc, 4);
        EXPECT_EQ(strcmp(argv[0], "esmini"), 0);
        EXPECT_EQ(strcmp(argv[1], "--headless"), 0);
        EXPECT_EQ(strcmp(argv[2], "--osc"), 0);
        EXPECT_EQ(strcmp(argv[3], "../../../resources/xosc/cut-in_simple.xosc"), 0);
    }
    {
        // this is special case where we dont want to remove window argument if its found after headless
        const char* args[] = {"esmini", "--headless", "--window", "60", "60", "800", "400", "--osc", "../../../resources/xosc/cut-in_simple.xosc"};
        esmini::common::Config config("esmini", sizeof(args) / sizeof(char*), const_cast<char**>(args));
        auto [argc, argv] = config.Load();
        EXPECT_EQ(argc, 9);
        EXPECT_EQ(strcmp(argv[0], "esmini"), 0);
        EXPECT_EQ(strcmp(argv[1], "--headless"), 0);
        EXPECT_EQ(strcmp(argv[2], "--window"), 0);
        EXPECT_EQ(strcmp(argv[3], "60"), 0);
        EXPECT_EQ(strcmp(argv[4], "60"), 0);
        EXPECT_EQ(strcmp(argv[5], "800"), 0);
        EXPECT_EQ(strcmp(argv[6], "400"), 0);
        EXPECT_EQ(strcmp(argv[7], "--osc"), 0);
        EXPECT_EQ(strcmp(argv[8], "../../../resources/xosc/cut-in_simple.xosc"), 0);
    }
}

TEST(MathFunctions, TestGetAbsAngleDifference)
{
    EXPECT_NEAR(GetAbsAngleDifference(1.0, 2.0), 1.0, 1e-3);
    EXPECT_NEAR(GetAbsAngleDifference(1.0, 5.0), 2.2831, 1e-3);
    EXPECT_NEAR(GetAbsAngleDifference(5.0, 1.0), 2.2831, 1e-3);
    EXPECT_NEAR(GetAbsAngleDifference(-1.0, 2.0), 3.0, 1e-3);
}

TEST(CommonUtilityFunctions, TestCombineDirectoryPathAndFilepath)
{
    EXPECT_EQ(CombineDirectoryPathAndFilepath("/home/kalle", "my_file.txt"), "/home/kalle/./my_file.txt");
    EXPECT_EQ(CombineDirectoryPathAndFilepath("/home/Kalle", "my_File.txt"), "/home/Kalle/./my_File.txt");
    EXPECT_EQ(CombineDirectoryPathAndFilepath("../home/Kalle", "my_File.txt"), "../home/Kalle/./my_File.txt");
    EXPECT_EQ(CombineDirectoryPathAndFilepath("", "my_File.txt"), "./my_File.txt");
}

TEST(MathFunctions, TestGetAngleDifference)
{
    EXPECT_NEAR(GetAngleDifference(-0.4, 0.0), -0.4, 1e-5);
    EXPECT_NEAR(GetAngleDifference(0.1, 6.2), 0.183185, 1e-5);
    EXPECT_NEAR(GetAngleDifference(6.2, 0.1), -0.183185, 1e-5);
}

TEST(MathFunctions, TestGetIntersectionsOfLineAndCircle)
{
    double i0[2], i1[2];  // intersection points

    // some edge cases
    EXPECT_EQ(GetIntersectionsOfLineAndCircle({0.0, 0.0}, {1.0, 0.0}, {0.0, 2.0}, 1.0, i0, i1), 0);     // circle above the line
    EXPECT_EQ(GetIntersectionsOfLineAndCircle({0.0, 0.0}, {1.0, 0.0}, {0.0, -2.0}, 1.0, i0, i1), 0);    // circle below the line
    EXPECT_EQ(GetIntersectionsOfLineAndCircle({0.0, 0.0}, {1.0, 0.0}, {10.0, 1.0}, 1.0, i0, i1), 1);    // tangent/one intersection point
    EXPECT_EQ(GetIntersectionsOfLineAndCircle({0.0, 0.0}, {1.0, 0.0}, {10.0, -1.0}, 1.0, i0, i1), 1);   // tangent/one intersection point
    EXPECT_EQ(GetIntersectionsOfLineAndCircle({0.0, 0.0}, {1.0, 0.0}, {10.0, -1.0}, 1.1, i0, i1), 2);   // two intersection points
    EXPECT_EQ(GetIntersectionsOfLineAndCircle({0.0, 0.0}, {1.0, 0.0}, {10.0, -1.0}, 0.99, i0, i1), 0);  // no intersection point, but very close to it
    EXPECT_EQ(GetIntersectionsOfLineAndCircle({-1.0, 0.0}, {-1.0, 5.0}, {1.0, 1.0}, 1.99, i0, i1), 0);  // no intersection point, but very close to it
    EXPECT_EQ(GetIntersectionsOfLineAndCircle({-1.0, 0.0}, {-1.0, 5.0}, {1.0, 1.0}, 2.0, i0, i1), 1);   // one intersection
    EXPECT_EQ(GetIntersectionsOfLineAndCircle({-1.0, 0.0}, {-1.0, 5.0}, {1.0, 1.0}, 2.01, i0, i1), 2);  // two intersection points
}

TEST(MathFunctions, TestGetDistanceFromPointToLine2DWithAngle)
{
    EXPECT_NEAR(DistanceFromPointToLine2DWithAngle(0.1, 0.9, 10.0, 10.0, M_PI_4), -0.5656, 1e-3);
    EXPECT_NEAR(DistanceFromPointToLine2DWithAngle(0.1, 0.1, 10.0, 10.0, M_PI_4), 0.0, 1e-3);
    EXPECT_NEAR(DistanceFromPointToLine2DWithAngle(0.9, 0.1, 10.0, 10.0, M_PI_4), 0.5656, 1e-3);

    EXPECT_NEAR(DistanceFromPointToLine2DWithAngle(-0.1, 0.9, -10.0, 10.0, 3 * M_PI_4), 0.5656, 1e-3);
    EXPECT_NEAR(DistanceFromPointToLine2DWithAngle(-0.1, 0.1, -10.0, 10.0, 3 * M_PI_4), 0.0, 1e-3);
    EXPECT_NEAR(DistanceFromPointToLine2DWithAngle(-0.9, 0.1, -10.0, 10.0, 3 * M_PI_4), -0.5656, 1e-3);
}

static bool log_msg_received = false;

static void log_callback(const std::string& msg)
{
    EXPECT_EQ(msg, "[] [warn] Expected message\n");
    log_msg_received = true;
}

TEST(LogFeatures, TestLogCallback)
{
    EXPECT_EQ(txtLogger.GetNumberOfCallbacks(), 0);

    txtLogger.RegisterCallback(log_callback);
    EXPECT_EQ(txtLogger.GetNumberOfCallbacks(), 1);

    LOG_WARN("Expected message");
    EXPECT_EQ(log_msg_received, true);

    txtLogger.ClearCallbacks();
    EXPECT_EQ(txtLogger.GetNumberOfCallbacks(), 0);
}

TEST(LogFeatures, TestLogBuffer)
{
    const std::deque<std::string>& buffer = txtLogger.GetBuffer();
    EXPECT_EQ(buffer.size(), 0);
    EXPECT_EQ(txtLogger.GetBufferCapacity(), 0);

    LOG_INFO("Should not be stored in buffer");
    EXPECT_EQ(buffer.size(), 0);
    EXPECT_EQ(txtLogger.GetBufferCapacity(), 0);

    txtLogger.SetBufferCapacity(3);
    LOG_INFO("Should be stored in buffer");
    EXPECT_EQ(buffer.size(), 1);
    EXPECT_EQ(txtLogger.GetBufferCapacity(), 3);
    EXPECT_EQ(buffer[0], "[] [info] Should be stored in buffer\n");

    LOG_INFO("Should also be stored in buffer");
    EXPECT_EQ(buffer.size(), 2);
    EXPECT_EQ(txtLogger.GetBufferCapacity(), 3);
    EXPECT_EQ(buffer[0], "[] [info] Should be stored in buffer\n");
    EXPECT_EQ(buffer[1], "[] [info] Should also be stored in buffer\n");

    LOG_INFO("Should also, again, be stored in buffer");
    EXPECT_EQ(buffer.size(), 3);
    EXPECT_EQ(txtLogger.GetBufferCapacity(), 3);
    EXPECT_EQ(buffer[0], "[] [info] Should be stored in buffer\n");
    EXPECT_EQ(buffer[1], "[] [info] Should also be stored in buffer\n");
    EXPECT_EQ(buffer[2], "[] [info] Should also, again, be stored in buffer\n");

    LOG_INFO("Should replace first message");
    EXPECT_EQ(buffer.size(), 3);
    EXPECT_EQ(txtLogger.GetBufferCapacity(), 3);
    EXPECT_EQ(buffer[0], "[] [info] Should also be stored in buffer\n");
    EXPECT_EQ(buffer[1], "[] [info] Should also, again, be stored in buffer\n");
    EXPECT_EQ(buffer[2], "[] [info] Should replace first message\n");

    txtLogger.ClearBufferAndCapacity();
    EXPECT_EQ(buffer.size(), 0);
    EXPECT_EQ(txtLogger.GetBufferCapacity(), 0);
}

int main(int argc, char** argv)
{
    // testing::GTEST_FLAG(filter) = "*TestIsPointWithinSectorBetweenTwoLines*";
    testing::InitGoogleTest(&argc, argv);

    if (argc > 1)
    {
        if (!strcmp(argv[1], "--disable_stdout"))
        {
            // disable logging to stdout from the test cases
            SE_Env::Inst().GetOptions().SetOptionValue("disable_stdout", "", false, true);
        }
        else
        {
            printf("Usage: %s [--disable_stout] [google test options...]\n", argv[0]);
            return -1;
        }
    }

    return RUN_ALL_TESTS();
}
