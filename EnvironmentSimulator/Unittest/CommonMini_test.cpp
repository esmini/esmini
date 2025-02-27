#include <gtest/gtest.h>

#include "CommonMini.hpp"
#include "esminiLib.hpp"

#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#error "Missing <filesystem> header"
#endif

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

TEST(ProgramOptions, TestNonPersisted)
{
    std::string paramName  = "density";
    std::string paramValue = "10";
    const char* args[]     = {"--osc", "../../../resources/xosc/cut-in_simple.xosc"};
    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    SE_SetOptionValue(paramName.c_str(), paramValue.c_str());
    const char* value = SE_GetOptionValue(paramName.c_str());
    ASSERT_NE(value, nullptr);
    std::string strValue(value);
    EXPECT_EQ(strValue, paramValue);
    SE_Close();

    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    value = SE_GetOptionValue(paramName.c_str());
    ASSERT_EQ(value, nullptr);
    SE_Close();
}

TEST(ProgramOptions, TestPersisted)
{
    std::string paramValue = "10";
    std::string paramName  = "density";
    const char* args[]     = {"--osc", "../../../resources/xosc/cut-in_simple.xosc"};
    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    SE_SetOptionValuePersistent(paramName.c_str(), paramValue.c_str());
    const char* value = SE_GetOptionValue(paramName.c_str());
    ASSERT_NE(value, nullptr);
    std::string optionValue(value);
    EXPECT_EQ(optionValue, paramValue);
    SE_Close();

    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    optionValue = SE_GetOptionValue(paramName.c_str());
    EXPECT_EQ(optionValue, paramValue);
    // make it non-persistent for cleanup
    SE_SetOptionValue(paramName.c_str(), paramValue.c_str());
    SE_Close();
}

TEST(ProgramOptions, TestAutoApply)
{
    std::string param  = "logfile_path";
    const char* args[] = {"--osc", "../../../resources/xosc/cut-in_simple.xosc"};
    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    const char* value = SE_GetOptionValue(param.c_str());
    ASSERT_NE(value, nullptr);
    std::string optionValue(value);
    EXPECT_EQ(optionValue, LOG_FILENAME);
    SE_Close();
}

TEST(ProgramOptions, TestAutoNotAppliedWhenSetEmpty)
{
    std::string paramName = "logfile_path";
    SE_SetOptionValue(paramName.c_str(), "");
    const char* args[] = {"--osc", "../../../resources/xosc/cut-in_simple.xosc"};
    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    const char* value = SE_GetOptionValue(paramName.c_str());
    std::string optionValue(value);
    ASSERT_EQ(optionValue, "");
    SE_Close();
}

TEST(ProgramOptions, TestDefaultValueSetIfMentionedOnly)
{
    std::string paramName = "record";
    const char* args[]    = {"--osc", "../../../resources/xosc/cut-in_simple.xosc"};
    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    const char* value = SE_GetOptionValue(paramName.c_str());
    ASSERT_EQ(value, nullptr);
    SE_Close();

    SE_SetOption(paramName.c_str());
    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    value = SE_GetOptionValue(paramName.c_str());
    ASSERT_NE(value, nullptr);
    std::string optionValue(value);
    EXPECT_EQ(optionValue, DAT_FILENAME);
    SE_Close();
}

TEST(ProgramOptions, TestMixOfPersistedAndNonPersisted)
{
    double elapsed_time = 0;
    SE_SetLogFilePath("my_test.txt");
    SE_SetOptionPersistent("log_append");
    SE_SetOption("osi_file");
    SE_SetOptionValue("log_level", "info");
    SE_SetOptionValue("fixed_timestep", "0.01");
    SE_Init("../../../resources/xosc/cut-in.xosc", 0, 0, 0, 0);

    while (SE_GetQuitFlag() == 0)
    {
        if (elapsed_time > 3.0)
        {
            const char* value = SE_GetOptionValue("logfile_path");
            std::string optionValue(value);
            ASSERT_EQ(optionValue, "my_test.txt");
            SE_SetOptionPersistent("log_meta_data");
            break;
        }
        SE_Step();
        elapsed_time += 0.5;
    }
    SE_Close();

    const char* value = SE_GetOptionValue("logfile_path");
    std::string optionValue(value);
    ASSERT_EQ(optionValue, "my_test.txt");

    // check log_meta_data if its set
    bool isSet = SE_GetOptionSet("log_meta_data");
    EXPECT_TRUE(isSet);

    isSet = SE_GetOptionSet("log_append");
    EXPECT_TRUE(isSet);

    isSet = SE_GetOptionSet("log_level");
    EXPECT_FALSE(isSet);

    isSet = SE_GetOptionSet("osi_file");
    EXPECT_FALSE(isSet);

    isSet = SE_GetOptionSet("fixed_timestep");
    EXPECT_FALSE(isSet);

    value = SE_GetOptionValue("log_level");
    ASSERT_EQ(value, nullptr);

    // check log file path

    SE_SetLogFilePath("my_test_error.txt");
    // SE_SetOptionValuePersistent("log_level", "error");
    SE_SetOptionPersistent("osi_file");  // Works
    SE_Init("../../../resources/xosc/cut-in.xosc", 0, 0, 0, 0);

    value = SE_GetOptionValue("logfile_path");
    std::string val1(value);
    ASSERT_EQ(val1, "my_test_error.txt");
    elapsed_time = 0;
    while (SE_GetQuitFlag() == 0)
    {
        if (elapsed_time > 3.0)
        {
            break;
        }
        elapsed_time += 0.5;
        SE_Step();
    }
    SE_Close();

    value = SE_GetOptionValue("logfile_path");
    std::string logFilePath(value);
    ASSERT_EQ(logFilePath, "my_test_error.txt");
}

TEST(ProgramOptions, FindsDefaultConfigFile)
{
    // just check if the default config file is present at esmini root folder
    fs::directory_entry entry{"../../../config.yml"};
    EXPECT_TRUE(entry.exists());

}

TEST(ProgramOptions, LastOptionOverrides)
{
    std::string optionName = "osc";
    std::string optionValue("../../../resources/xosc/cut-in_simple.xosc");
    const char* args[]    = {"--osc", "../../../resources/xosc/acc-test.xosc", "--osc", optionValue.c_str()};
    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    const char* value = SE_GetOptionValue(optionName.c_str());
    ASSERT_NE(value, nullptr);
    EXPECT_EQ(value, optionValue);
    SE_Close();
}


TEST(ProgramOptions, CommandPromptOverridesApi)
{
    std::string optionName = "osc";
    std::string optionValue("../../../resources/xosc/cut-in.xosc");
    SE_SetOptionValue(optionName.c_str(), "../../../resources/xosc/cut-in_simple.xosc");
    SE_Init(optionValue.c_str(), 0, 0, 0, 0);
    const char* value = SE_GetOptionValue(optionName.c_str());
    ASSERT_NE(value, nullptr);
    EXPECT_EQ(value, optionValue);
    SE_Close();
}


TEST(ProgramOptions, LastFileOptionsOverride)
{
    std::string firstConfigFileName = "config1.yml";
    std::string secondConfigFileName = "config2.yml";
    {
        // create first config file
        std::ofstream file(firstConfigFileName);
        if (!file)
        {
            std::cerr << "Failed to create file: " << firstConfigFileName << std::endl;
            return;
        }
        // Write YAML content
        file << "esmini: \n";
        // file << "  window: 60 60 800 400\n";
        file << "  logfile_path: log1.txt\n";
        file << "  osc: ../../../resources/xosc/cut-in.xosc\n";
        file << "replayer:\n";
        file << "  file: sim1.dat";
        file.close();
    }

    // // investigation
    // SE_SetOptionValue("config_file_path", "config1.yml");
    // SE_SetOptionValue("config_file_path", "config2.yml");
    // // SE_Init("../../../resources/xosc/cut-in.xosc", 0, 0, 0, 0);
    // const char* args[] = {"--osc", "../../../resources/xosc/cut-in.xosc"};
    // ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    // SE_Close();

    {
        // create second config file
        std::ofstream file(secondConfigFileName);
        if (!file)
        {
            std::cerr << "Failed to create file: " << secondConfigFileName << std::endl;
            return;
        }
        // Write YAML content
        file << "esmini: \n";
        //file << "  window: 60 60 800 400\n";
        file << "  logfile_path: log2.txt\n";
        file << "  osc: ../../../resources/xosc/cut-in_simple.xosc\n";
        file << "replayer:\n";
        file << "  file: sim2.dat";
        file.close();
    }

    {
        // firstly we will put config1.yml and then config2.yml - and check if the options from config2.yml are taken
        const char* args[] = {"--config_file_path", firstConfigFileName.c_str(), "--config_file_path", secondConfigFileName.c_str()};
        // const char* args[] = {"--osc", "../../../resources/xosc/cut-in_simple.xosc"};
        ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
        const char* value = SE_GetOptionValue("logfile_path");
        ASSERT_NE(value, nullptr);
        std::string expectedValue = "log2.txt";
        EXPECT_EQ(value, expectedValue);
        value = SE_GetOptionValue("osc");
        ASSERT_NE(value, nullptr);
        expectedValue = "../../../resources/xosc/cut-in_simple.xosc";
        EXPECT_EQ(value, expectedValue);
        SE_Close();
    }

    {
        // secondly we will put config2.yml and then config1.yml - and check if the options from config1.yml are taken
        const char* args[] = {"--config_file_path", secondConfigFileName.c_str(), "--config_file_path", firstConfigFileName.c_str()};
        ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
        const char* value = SE_GetOptionValue("logfile_path");
        ASSERT_NE(value, nullptr);
        std::string expectedValue = "log1.txt";
        EXPECT_EQ(value, expectedValue);
        value = SE_GetOptionValue("osc");
        ASSERT_NE(value, nullptr);
        expectedValue = "../../../resources/xosc/cut-in.xosc";
        EXPECT_EQ(value, expectedValue);
        SE_Close();
    }
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

int main(int argc, char** argv)
{
    // testing::GTEST_FLAG(filter) = "*TestIsPointWithinSectorBetweenTwoLines*";
    testing::InitGoogleTest(&argc, argv);

    if (argc > 1)
    {
        if (!strcmp(argv[1], "--disable_stdout"))
        {
            // disable logging to stdout from the esminiLib
            SE_SetOptionPersistent("disable_stdout");

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
