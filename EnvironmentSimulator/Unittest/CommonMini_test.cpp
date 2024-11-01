#include <gtest/gtest.h>

#include "CommonMini.hpp"
#include "esminiLib.hpp"
#include "TestHelper.hpp"
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

TEST(ProgramOptions, NonPersisted)
{
    std::string paramName  = "density";
    std::string paramValue = "10";
    const char* args[]     = {"--osc", "../../../resources/xosc/cut-in_simple.xosc"};
    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    SE_SetOptionValue(paramName.c_str(), paramValue.c_str());
    std::string optionValue = SE_GetOptionValue(paramName.c_str());
    EXPECT_EQ(optionValue, paramValue);
    SE_Close();

    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    optionValue = SE_GetOptionValue(paramName.c_str());
    EXPECT_NE(optionValue, paramValue);
    SE_Close();
}

TEST(ProgramOptions, Persisted)
{
    std::string paramValue = "10";
    std::string paramName  = "density";
    const char* args[]     = {"--osc", "../../../resources/xosc/cut-in_simple.xosc"};
    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    SE_SetOptionValuePersistent(paramName.c_str(), paramValue.c_str());
    std::string optionValue = SE_GetOptionValue(paramName.c_str());
    EXPECT_EQ(optionValue, paramValue);
    SE_Close();

    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    optionValue = SE_GetOptionValue(paramName.c_str());
    EXPECT_EQ(optionValue, paramValue);
    // make it non-persistent for cleanup
    SE_SetOptionValue(paramName.c_str(), paramValue.c_str());
    SE_Close();
}

int main(int argc, char** argv)
{
    // testing::GTEST_FLAG(filter) = "*TestIsPointWithinSectorBetweenTwoLines*";
    testing::InitGoogleTest(&argc, argv);

    if (ParseAndSetLoggerOptions(argc, argv) != 0)
    {
        return -1;
    }

    return RUN_ALL_TESTS();
}
