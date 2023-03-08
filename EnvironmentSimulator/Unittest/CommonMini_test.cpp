#include <gtest/gtest.h>

#include "CommonMini.hpp"
#include "esminiLib.hpp"

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

INSTANTIATE_TEST_SUITE_P(CommonMini,
                         Local2Global,
                         ::testing::Values(std::make_tuple(Coordinate2D{0, 1}, Coordinate2D{1, 1}, -M_PI / 2, Coordinate2D{2, 1}),

                                           std::make_tuple(Coordinate2D{1, 0}, Coordinate2D{1, 1}, -M_PI / 2, Coordinate2D{1, 0})));

int main(int argc, char **argv)
{
    // testing::GTEST_FLAG(filter) = "*TestIsPointWithinSectorBetweenTwoLines*";

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
