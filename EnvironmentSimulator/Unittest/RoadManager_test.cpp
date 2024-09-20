#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <stdexcept>

#include "RoadManager.hpp"
#include "TestHelper.hpp"

using namespace roadmanager;

#define TRIG_ERR_MARGIN 0.001

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> Polynomial //////////
//////////////////////////////////////////////////////////////////////

class PolynomialTestFixture : public testing::Test
{
public:
    PolynomialTestFixture();
    PolynomialTestFixture(double a, double b, double c, double d, double p_scale);
    virtual ~PolynomialTestFixture();

protected:
    Polynomial polynomial;
};

PolynomialTestFixture::PolynomialTestFixture()
{
}

PolynomialTestFixture::~PolynomialTestFixture()
{
}

TEST_F(PolynomialTestFixture, TestConstructorEmpty)
{
    ASSERT_EQ(0, polynomial.GetA());
    ASSERT_EQ(0, polynomial.GetB());
    ASSERT_EQ(0, polynomial.GetC());
    ASSERT_EQ(0, polynomial.GetD());
    ASSERT_EQ(1, polynomial.GetPscale());
}

TEST_F(PolynomialTestFixture, TestConstructorArgument)
{
    polynomial = Polynomial(1, -2, 3, -4);
    ASSERT_EQ(1, polynomial.GetA());
    ASSERT_EQ(-2, polynomial.GetB());
    ASSERT_EQ(3, polynomial.GetC());
    ASSERT_EQ(-4, polynomial.GetD());
    ASSERT_EQ(1, polynomial.GetPscale());
}

TEST_F(PolynomialTestFixture, TestConstructorArgumentPscale)
{
    polynomial = Polynomial(1, -2, 3, -4, 2);
    ASSERT_EQ(1, polynomial.GetA());
    ASSERT_EQ(-2, polynomial.GetB());
    ASSERT_EQ(3, polynomial.GetC());
    ASSERT_EQ(-4, polynomial.GetD());
    ASSERT_EQ(2, polynomial.GetPscale());
}

TEST_F(PolynomialTestFixture, TestSetGet)
{
    polynomial.Set(1, -2, 3, -4);
    ASSERT_EQ(1, polynomial.GetA());
    ASSERT_EQ(-2, polynomial.GetB());
    ASSERT_EQ(3, polynomial.GetC());
    ASSERT_EQ(-4, polynomial.GetD());
    ASSERT_EQ(1, polynomial.GetPscale());
}

TEST_F(PolynomialTestFixture, TestSetGetPscale)
{
    polynomial.Set(1, -2, 3, -4, 2);
    ASSERT_EQ(1, polynomial.GetA());
    ASSERT_EQ(-2, polynomial.GetB());
    ASSERT_EQ(3, polynomial.GetC());
    ASSERT_EQ(-4, polynomial.GetD());
    ASSERT_EQ(2, polynomial.GetPscale());
}

TEST_F(PolynomialTestFixture, TestSetGetIndividual)
{
    polynomial.SetA(1);
    polynomial.SetB(-2);
    polynomial.SetC(3);
    polynomial.SetD(-4);
    ASSERT_EQ(1, polynomial.GetA());
    ASSERT_EQ(-2, polynomial.GetB());
    ASSERT_EQ(3, polynomial.GetC());
    ASSERT_EQ(-4, polynomial.GetD());
}

class PolynomialTestEvaluateEmptyParametrized : public testing::TestWithParam<std::tuple<double, double>>
{
public:
protected:
    Polynomial polynomial_empty;
};

TEST_P(PolynomialTestEvaluateEmptyParametrized, TestEvaluateEmptyConstructor)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    polynomial_empty.SetA(1);
    polynomial_empty.SetB(-2);
    polynomial_empty.SetC(3);
    polynomial_empty.SetD(-4);
    ASSERT_EQ(polynomial_empty.Evaluate(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluateParamEmpty,
                         PolynomialTestEvaluateEmptyParametrized,
                         testing::Values(std::make_tuple(2, -23), std::make_tuple(0, 1)));

class PolynomialTestEvaluateArgumentParametrized : public testing::TestWithParam<std::tuple<double, double>>
{
public:
protected:
    Polynomial polynomial_argument{1, -2, 3, -4, 2};
};

TEST_P(PolynomialTestEvaluateArgumentParametrized, TestEvaluateArgumentConstructor)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    ASSERT_EQ(polynomial_argument.Evaluate(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluateParamArgument,
                         PolynomialTestEvaluateArgumentParametrized,
                         testing::Values(std::make_tuple(2, -215), std::make_tuple(0, 1)));

class PolynomialTestEvaluatePrimEmptyParametrized : public testing::TestWithParam<std::tuple<double, double>>
{
public:
protected:
    Polynomial polynomial_empty;
};

TEST_P(PolynomialTestEvaluatePrimEmptyParametrized, TestEvaluatePrimEmptyConstructor)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    polynomial_empty.SetA(1);
    polynomial_empty.SetB(-2);
    polynomial_empty.SetC(3);
    polynomial_empty.SetD(-4);
    ASSERT_EQ(polynomial_empty.EvaluatePrim(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluatePrimParamEmpty,
                         PolynomialTestEvaluatePrimEmptyParametrized,
                         testing::Values(std::make_tuple(2, -38), std::make_tuple(0, -2)));

class PolynomialTestEvaluatePrimArgumentParametrized : public testing::TestWithParam<std::tuple<double, double>>
{
public:
protected:
    Polynomial polynomial_argument{1, -2, 3, -4, 2};
};

TEST_P(PolynomialTestEvaluatePrimArgumentParametrized, TestEvaluatePrimArgumentConstructor)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    ASSERT_EQ(polynomial_argument.EvaluatePrim(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluatePrimParamArgument,
                         PolynomialTestEvaluatePrimArgumentParametrized,
                         testing::Values(std::make_tuple(2, -170), std::make_tuple(0, -2)));

class PolynomialTestEvaluatePrimPrimEmptyParametrized : public testing::TestWithParam<std::tuple<double, double>>
{
public:
protected:
    Polynomial polynomial_empty;
};

TEST_P(PolynomialTestEvaluatePrimPrimEmptyParametrized, TestEvaluatePrimPrimEmptyConstructor)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    polynomial_empty.SetA(1);
    polynomial_empty.SetB(-2);
    polynomial_empty.SetC(3);
    polynomial_empty.SetD(-4);
    ASSERT_EQ(polynomial_empty.EvaluatePrimPrim(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluatePrimPrimParamEmpty,
                         PolynomialTestEvaluatePrimPrimEmptyParametrized,
                         testing::Values(std::make_tuple(2, -42), std::make_tuple(0, 6)));

class PolynomialTestEvaluatePrimPrimArgumentParametrized : public testing::TestWithParam<std::tuple<double, double>>
{
public:
protected:
    Polynomial polynomial_argument{1, -2, 3, -4, 2};
};

TEST_P(PolynomialTestEvaluatePrimPrimArgumentParametrized, TestEvaluatePrimPrimArgumentConstructor)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    ASSERT_EQ(polynomial_argument.EvaluatePrimPrim(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluatePrimPrimParamArgument,
                         PolynomialTestEvaluatePrimPrimArgumentParametrized,
                         testing::Values(std::make_tuple(2, -90), std::make_tuple(0, 6)));

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> OSIPoints //////////
//////////////////////////////////////////////////////////////////////

class OSIPointsTestFixture : public testing::Test
{
public:
    OSIPointsTestFixture();
    OSIPointsTestFixture(std::vector<double> s, std::vector<double> x, std::vector<double> y, std::vector<double> z, std::vector<double> h);
    virtual ~OSIPointsTestFixture();

protected:
    OSIPoints osi_points;
};

OSIPointsTestFixture::OSIPointsTestFixture()
{
}

OSIPointsTestFixture::~OSIPointsTestFixture()
{
}

TEST_F(OSIPointsTestFixture, TestConstructorEmpty)
{
    ASSERT_EQ(0, osi_points.GetPoints().size());
}

TEST_F(OSIPointsTestFixture, TestConstructorArgument)
{
    std::vector<PointStruct> osi_points_test_set = {{0, 0, 0, 0, 0}, {-1, -1, -1, -1, -1}, {2, 2, 2, 2, 2}};

    OSIPoints osi_points_test_object = OSIPoints(osi_points_test_set);

    std::vector<PointStruct> osi_point_expected = osi_points_test_object.GetPoints();

    for (size_t i = 0; i < osi_points_test_set.size(); i++)
    {
        ASSERT_EQ(osi_points_test_object.GetPoints()[i].s, osi_point_expected[i].s);
        ASSERT_EQ(osi_points_test_object.GetPoints()[i].x, osi_point_expected[i].x);
        ASSERT_EQ(osi_points_test_object.GetPoints()[i].y, osi_point_expected[i].y);
        ASSERT_EQ(osi_points_test_object.GetPoints()[i].z, osi_point_expected[i].z);
        ASSERT_EQ(osi_points_test_object.GetPoints()[i].h, osi_point_expected[i].h);
    }
}

TEST_F(OSIPointsTestFixture, TestSetGet)
{
    std::vector<PointStruct> osi_points_test_set = {{0, 0, 0, 0, 0}, {-1, -1, -1, -1, -1}, {2, 2, 2, 2, 2}};

    osi_points.Set(osi_points_test_set);

    std::vector<PointStruct> osi_point_expected = osi_points.GetPoints();

    for (size_t i = 0; i < osi_points_test_set.size(); i++)
    {
        ASSERT_EQ(osi_points.GetPoints()[i].s, osi_point_expected[i].s);
        ASSERT_EQ(osi_points.GetPoints()[i].x, osi_point_expected[i].x);
        ASSERT_EQ(osi_points.GetPoints()[i].y, osi_point_expected[i].y);
        ASSERT_EQ(osi_points.GetPoints()[i].z, osi_point_expected[i].z);
        ASSERT_EQ(osi_points.GetPoints()[i].h, osi_point_expected[i].h);
    }
}

TEST_F(OSIPointsTestFixture, TestGetFromIdxEmpty)
{
    ASSERT_THROW(osi_points.GetXfromIdx(-1), std::runtime_error);
    ASSERT_THROW(osi_points.GetYfromIdx(-1), std::runtime_error);
    ASSERT_THROW(osi_points.GetZfromIdx(-1), std::runtime_error);
    ASSERT_THROW(osi_points.GetXfromIdx(0), std::runtime_error);
    ASSERT_THROW(osi_points.GetYfromIdx(0), std::runtime_error);
    ASSERT_THROW(osi_points.GetZfromIdx(0), std::runtime_error);
    ASSERT_THROW(osi_points.GetXfromIdx(1), std::runtime_error);
    ASSERT_THROW(osi_points.GetYfromIdx(1), std::runtime_error);
    ASSERT_THROW(osi_points.GetZfromIdx(1), std::runtime_error);
}

TEST_F(OSIPointsTestFixture, TestGetFromIdx)
{
    std::vector<PointStruct> osi_points_test_set = {{0, 0, 0, 0, 0}, {-1, -1, -1, -1, -1}, {2, 2, 2, 2, 2}};

    OSIPoints osi_points_test_object = OSIPoints(osi_points_test_set);

    ASSERT_EQ(osi_points_test_object.GetXfromIdx(0), 0);
    ASSERT_EQ(osi_points_test_object.GetXfromIdx(1), -1);
    ASSERT_EQ(osi_points_test_object.GetXfromIdx(2), 2);

    ASSERT_EQ(osi_points_test_object.GetYfromIdx(0), 0);
    ASSERT_EQ(osi_points_test_object.GetYfromIdx(1), -1);
    ASSERT_EQ(osi_points_test_object.GetYfromIdx(2), 2);

    ASSERT_EQ(osi_points_test_object.GetZfromIdx(0), 0);
    ASSERT_EQ(osi_points_test_object.GetZfromIdx(1), -1);
    ASSERT_EQ(osi_points_test_object.GetZfromIdx(2), 2);
}

TEST_F(OSIPointsTestFixture, TestGetNumOfOSIPoints)
{
    ASSERT_EQ(osi_points.GetNumOfOSIPoints(), 0);

    std::vector<PointStruct> osi_points_test_set = {{0, 0, 0, 0, 0}, {-1, -1, -1, -1, -1}, {2, 2, 2, 2, 2}};
    std::vector<double>      s{0, -1, 2};
    std::vector<double>      x{0, -1, 2};
    std::vector<double>      y{0, -1, 2};
    std::vector<double>      z{0, -1, 2};
    std::vector<double>      h{0, -1, 2};

    OSIPoints osi_points_second = OSIPoints(osi_points_test_set);
    ASSERT_EQ(osi_points_second.GetNumOfOSIPoints(), 3);
}

class OSIPointsCloseCheck : public ::testing::TestWithParam<std::tuple<float, float, float, float, float, float, float, float, int>>
{
};
// eight points (start and end for two lines)
// l1_p1_x, l1_p1_y, l1_p2_x, l1_p2_y, l2_p1_x, l2_p1_y, l2_p2_x, l2_p2_y, expected amount points same
TEST_P(OSIPointsCloseCheck, ClosenessChecker)
{
    PointStruct s1;
    s1.x = std::get<0>(GetParam());
    s1.y = std::get<1>(GetParam());

    PointStruct s2;
    s2.x = std::get<2>(GetParam());
    s2.y = std::get<3>(GetParam());

    PointStruct s3;
    s3.x = std::get<4>(GetParam());
    s3.y = std::get<5>(GetParam());

    PointStruct s4;
    s4.x = std::get<6>(GetParam());
    s4.y = std::get<7>(GetParam());

    std::vector<PointStruct> vec1;
    vec1.push_back(s1);
    vec1.push_back(s2);

    std::vector<PointStruct> vec2;
    vec2.push_back(s3);
    vec2.push_back(s4);

    OSIPoints line1 = OSIPoints();
    line1.Set(vec1);
    OSIPoints line2 = OSIPoints();
    line2.Set(vec2);

    int num_intersecting_points = CheckOverlapingOSIPoints(&line1, &line2, 0.001);
    ASSERT_EQ(num_intersecting_points, std::get<8>(GetParam()));
}

INSTANTIATE_TEST_SUITE_P(OSI_Points_check_test,
                         OSIPointsCloseCheck,
                         ::testing::Values(std::make_tuple(0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1),
                                           std::make_tuple(0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 2),
                                           std::make_tuple(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 2.0f, 1.0f, 1.0f, 0),
                                           std::make_tuple(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 2.0f, 0.0f, 0.0f, 1),
                                           std::make_tuple(0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 2)));
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> Line (Geometry) //////////
//////////////////////////////////////////////////////////////////////

class LineGeomTestFixture : public testing::Test
{
public:
    LineGeomTestFixture();
    LineGeomTestFixture(double s, double x, double y, double hdg, double length);
    virtual ~LineGeomTestFixture();

protected:
    Line line;
};

LineGeomTestFixture::LineGeomTestFixture()
{
}

LineGeomTestFixture::~LineGeomTestFixture()
{
}

TEST_F(LineGeomTestFixture, TestConstructorArgument)
{
    ASSERT_EQ(0, line.GetS());
    ASSERT_EQ(0, line.GetX());
    ASSERT_EQ(0, line.GetY());
    ASSERT_EQ(0, line.GetHdg());
    ASSERT_EQ(0, line.GetLength());
    EXPECT_EQ(line.GetType(), Geometry::GEOMETRY_TYPE_UNKNOWN);

    Line line_second = Line(2, -1, 1, 5 * M_PI, 4);
    ASSERT_EQ(2, line_second.GetS());
    ASSERT_EQ(-1, line_second.GetX());
    ASSERT_EQ(1, line_second.GetY());
    ASSERT_EQ(M_PI, line_second.GetHdg());
    ASSERT_EQ(4, line_second.GetLength());
    EXPECT_EQ(line_second.GetType(), Geometry::GEOMETRY_TYPE_LINE);

    Line line_third = Line(2, -1, 1, -5 * M_PI, 4);
    ASSERT_EQ(2, line_third.GetS());
    ASSERT_EQ(-1, line_third.GetX());
    ASSERT_EQ(1, line_third.GetY());
    ASSERT_EQ(M_PI, line_third.GetHdg());
    ASSERT_EQ(4, line_third.GetLength());
    EXPECT_EQ(line_third.GetType(), Geometry::GEOMETRY_TYPE_LINE);
}

TEST_F(LineGeomTestFixture, TestEvaluateCurvatureDS)
{
    ASSERT_EQ(line.EvaluateCurvatureDS(0), 0.0);
    ASSERT_EQ(line.EvaluateCurvatureDS(10), 0.0);
    ASSERT_EQ(line.EvaluateCurvatureDS(100), 0.0);
    ASSERT_EQ(line.EvaluateCurvatureDS(1000), 0.0);
}

class LineGeomTestEvaluateDsEmptyConstructor : public testing::TestWithParam<std::tuple<double, double, double, double>>
{
public:
protected:
    Line line;
};

TEST_P(LineGeomTestEvaluateDsEmptyConstructor, TestLineGeomEvaluateDsEmpty)
{
    std::tuple<double, double, double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    double *x, *y, *h;
    double  my_x = line.GetX();
    double  my_y = line.GetY();
    double  my_h = line.GetHdg();
    x            = &my_x;
    y            = &my_y;
    h            = &my_h;
    line.EvaluateDS(std::get<0>(tuple), x, y, h);
    ASSERT_EQ(*x, std::get<1>(tuple));
    ASSERT_EQ(*y, std::get<2>(tuple));
    ASSERT_EQ(*h, std::get<3>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluateDsEmptyParam,
                         LineGeomTestEvaluateDsEmptyConstructor,
                         testing::Values(std::make_tuple(0, 0, 0, 0), std::make_tuple(1, 1, 0, 0), std::make_tuple(100, 100, 0, 0)));

class LineGeomTestEvaluateDsArgumentConstructor : public testing::TestWithParam<std::tuple<double, double, double, double>>
{
public:
protected:
    Line line{2.0, -1.0, 1.0, 5 * M_PI, 4.0};
};

TEST_P(LineGeomTestEvaluateDsArgumentConstructor, TestLineGeomEvaluateDsArgument)
{
    std::tuple<double, double, double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    double *x, *y, *h;
    double  my_x = line.GetX();
    double  my_y = line.GetY();
    double  my_h = line.GetHdg();
    x            = &my_x;
    y            = &my_y;
    h            = &my_h;
    line.EvaluateDS(std::get<0>(tuple), x, y, h);
    ASSERT_EQ(*x, std::get<1>(tuple));
    ASSERT_EQ(*y, std::get<2>(tuple));
    ASSERT_EQ(*h, std::get<3>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluateLineDsArgumentParam,
                         LineGeomTestEvaluateDsArgumentConstructor,
                         testing::Values(std::make_tuple(0.0, -1.0, 1.0, M_PI),
                                         std::make_tuple(1.0, -2.0, 1.0 + sin(M_PI), M_PI),
                                         std::make_tuple(100.0, -101.0, 1.0 + 100 * sin(M_PI), M_PI)));

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> Arc (Geometry) //////////
//////////////////////////////////////////////////////////////////////

class ArcGeomTestFixture : public testing::Test
{
public:
    ArcGeomTestFixture();
    ArcGeomTestFixture(double s, double x, double y, double hdg, double length, double curvature);
    virtual ~ArcGeomTestFixture();

protected:
    roadmanager::Arc arc;
};

ArcGeomTestFixture::ArcGeomTestFixture()
{
}

ArcGeomTestFixture::~ArcGeomTestFixture()
{
}

TEST_F(ArcGeomTestFixture, TestConstructorArgument)
{
    ASSERT_EQ(0.0, arc.GetCurvature());
    EXPECT_EQ(arc.GetType(), Geometry::GEOMETRY_TYPE_UNKNOWN);

    roadmanager::Arc arc_second = roadmanager::Arc(2, -1, 1, 5 * M_PI, 4, 5);
    ASSERT_EQ(5.0, arc_second.GetCurvature());
    EXPECT_EQ(arc_second.GetType(), Geometry::GEOMETRY_TYPE_ARC);
}

TEST_F(ArcGeomTestFixture, TestEvaluateCurvatureDS)
{
    ASSERT_EQ(arc.EvaluateCurvatureDS(0), 0.0);
    ASSERT_EQ(arc.EvaluateCurvatureDS(10), 0.0);
    ASSERT_EQ(arc.EvaluateCurvatureDS(100), 0.0);
    ASSERT_EQ(arc.EvaluateCurvatureDS(1000), 0.0);

    roadmanager::Arc arc_second = roadmanager::Arc(2, -1, 1, 5 * M_PI, 4, 5);

    ASSERT_EQ(arc_second.EvaluateCurvatureDS(0), 5.0);
    ASSERT_EQ(arc_second.EvaluateCurvatureDS(10), 5.0);
    ASSERT_EQ(arc_second.EvaluateCurvatureDS(100), 5.0);
    ASSERT_EQ(arc_second.EvaluateCurvatureDS(1000), 5.0);
}

TEST_F(ArcGeomTestFixture, TestGetRadius)
{
    roadmanager::Arc arc_second = roadmanager::Arc(2, -1, 1, 5 * M_PI, 4, 5);
    ASSERT_EQ(arc_second.GetRadius(), 0.2);

    roadmanager::Arc arc_third = roadmanager::Arc(2, -1, 1, 5 * M_PI, 4, -10);
    ASSERT_EQ(arc_third.GetRadius(), 0.1);
}

class ArcGeomTestEvaluateDsCurvPositive : public testing::TestWithParam<std::tuple<double, double, double, double>>
{
public:
protected:
    roadmanager::Arc arc{2.0, -1.0, 1.0, 5 * M_PI, 4.0, 1.0};
};

TEST_P(ArcGeomTestEvaluateDsCurvPositive, TestArcGeomEvaluateDsArgument)
{
    std::tuple<double, double, double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    double *x, *y, *h;
    double  my_x = arc.GetX();
    double  my_y = arc.GetY();
    double  my_h = arc.GetHdg();
    x            = &my_x;
    y            = &my_y;
    h            = &my_h;
    arc.EvaluateDS(std::get<0>(tuple), x, y, h);
    ASSERT_THAT(*x, testing::AllOf(testing::Gt(std::get<1>(tuple) - TRIG_ERR_MARGIN), testing::Lt(std::get<1>(tuple) + TRIG_ERR_MARGIN)));
    ASSERT_THAT(*y, testing::AllOf(testing::Gt(std::get<2>(tuple) - TRIG_ERR_MARGIN), testing::Lt(std::get<2>(tuple) + TRIG_ERR_MARGIN)));
    ASSERT_EQ(*h, std::get<3>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluateArcDsArgumentParam,
                         ArcGeomTestEvaluateDsCurvPositive,
                         testing::Values(std::make_tuple(0.0, -1.0, 1.0, M_PI),
                                         std::make_tuple(M_PI / 2, -2.0, 0.0, M_PI + M_PI / 2),
                                         std::make_tuple(M_PI, -1.0, -1.0, 2 * M_PI)));

class ArcGeomTestEvaluateDsCurvNegative : public testing::TestWithParam<std::tuple<double, double, double, double>>
{
public:
protected:
    roadmanager::Arc arc{2.0, -1.0, 1.0, 5 * M_PI, 4.0, -1.0};
};

TEST_P(ArcGeomTestEvaluateDsCurvNegative, TestArcGeomEvaluateDsArgument)
{
    std::tuple<double, double, double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    double *x, *y, *h;
    double  my_x = arc.GetX();
    double  my_y = arc.GetY();
    double  my_h = arc.GetHdg();
    x            = &my_x;
    y            = &my_y;
    h            = &my_h;
    arc.EvaluateDS(std::get<0>(tuple), x, y, h);
    ASSERT_THAT(*x, testing::AllOf(testing::Gt(std::get<1>(tuple) - TRIG_ERR_MARGIN), testing::Lt(std::get<1>(tuple) + TRIG_ERR_MARGIN)));
    ASSERT_THAT(*y, testing::AllOf(testing::Gt(std::get<2>(tuple) - TRIG_ERR_MARGIN), testing::Lt(std::get<2>(tuple) + TRIG_ERR_MARGIN)));
    ASSERT_EQ(*h, std::get<3>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluateArcDsArgumentParam,
                         ArcGeomTestEvaluateDsCurvNegative,
                         testing::Values(std::make_tuple(0.0, -1.0, 1.0, M_PI),
                                         std::make_tuple(M_PI / 2, -2.0, 2.0, M_PI - M_PI / 2),
                                         std::make_tuple(M_PI, -1.0, 3.0, 0)));

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> Spiral (Geometry) //////////
//////////////////////////////////////////////////////////////////////

class SpiralGeomTestFixture : public testing::Test
{
public:
    SpiralGeomTestFixture();
    SpiralGeomTestFixture(double s, double x, double y, double hdg, double length, double curv_start, double curv_end);
    virtual ~SpiralGeomTestFixture();

protected:
    Spiral spiral;
};

SpiralGeomTestFixture::SpiralGeomTestFixture()
{
}

SpiralGeomTestFixture::~SpiralGeomTestFixture()
{
}

TEST_F(SpiralGeomTestFixture, TestConstructorArgument)
{
    ASSERT_EQ(0.0, spiral.GetCurvStart());
    ASSERT_EQ(0.0, spiral.GetCurvEnd());
    ASSERT_EQ(0.0, spiral.GetCDot());
    ASSERT_EQ(0.0, spiral.GetX0());
    ASSERT_EQ(0.0, spiral.GetY0());
    ASSERT_EQ(0.0, spiral.GetH0());
    ASSERT_EQ(0.0, spiral.GetS0());
    EXPECT_EQ(spiral.GetType(), Geometry::GEOMETRY_TYPE_UNKNOWN);

    Spiral spiral_second = Spiral(2, -1, 1, 5 * M_PI, 4, 2, 10);
    ASSERT_EQ(2.0, spiral_second.GetCurvStart());
    ASSERT_EQ(10.0, spiral_second.GetCurvEnd());
    // EMIL added constructor definition for Spiral constructor. Will be fixed later.
    /*ASSERT_EQ(0.0, spiral_second.GetCDot());
    ASSERT_EQ(0.0, spiral_second.GetX0());
    ASSERT_EQ(0.0, spiral_second.GetY0());
    ASSERT_EQ(0.0, spiral_second.GetH0());
    ASSERT_EQ(0.0, spiral_second.GetS0());*/
    EXPECT_EQ(spiral_second.GetType(), Geometry::GEOMETRY_TYPE_SPIRAL);
}

TEST_F(SpiralGeomTestFixture, TestGetSet)
{
    spiral.SetX0(5);
    spiral.SetY0(-10);
    spiral.SetH0(15);
    spiral.SetS0(-20);
    spiral.SetCDot(0);

    ASSERT_EQ(5.0, spiral.GetX0());
    ASSERT_EQ(-10.0, spiral.GetY0());
    ASSERT_EQ(15.0, spiral.GetH0());
    ASSERT_EQ(-20.0, spiral.GetS0());
    ASSERT_EQ(0.0, spiral.GetCDot());
}

TEST_F(SpiralGeomTestFixture, TestEvaluateCurvatureDS)
{
    Spiral spiral_second = Spiral(2, -1, 1, 5 * M_PI, 4, 2, 10);

    ASSERT_EQ(spiral_second.EvaluateCurvatureDS(0), 2.0);
    ASSERT_EQ(spiral_second.EvaluateCurvatureDS(10), 22.0);
    ASSERT_EQ(spiral_second.EvaluateCurvatureDS(100), 202.0);
    ASSERT_EQ(spiral_second.EvaluateCurvatureDS(1000), 2002.0);
}

/*
TODO: Remaining Test for this class is to test EvaluateDs function which inclides odrSpiral function
as extern void -> Check this later.
*/

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> Poly3 (Geometry) //////////
//////////////////////////////////////////////////////////////////////

class Poly3GeomTestFixture : public testing::Test
{
public:
    Poly3GeomTestFixture();
    Poly3GeomTestFixture(double s, double x, double y, double hdg, double length, double a, double b, double c, double d);
    virtual ~Poly3GeomTestFixture();

protected:
    Poly3 poly3;
};

Poly3GeomTestFixture::Poly3GeomTestFixture()
{
}

Poly3GeomTestFixture::~Poly3GeomTestFixture()
{
}

TEST_F(Poly3GeomTestFixture, TestPoly3UMaxGetSetArgumentConstructor)
{
    ASSERT_EQ(0.0, poly3.GetUMax());
    EXPECT_EQ(poly3.GetType(), Geometry::GEOMETRY_TYPE_UNKNOWN);
    poly3.SetUMax(5);
    ASSERT_EQ(poly3.GetUMax(), 5.0);

    Poly3 poly3_second = poly3 = Poly3(0, 0, 0, 0, 1, 0, 0, 1, -2);
    ASSERT_THAT(poly3_second.GetUMax(), testing::AllOf(testing::Gt(0.707107 - TRIG_ERR_MARGIN), testing::Lt(0.707107 + TRIG_ERR_MARGIN)));
    EXPECT_EQ(poly3_second.GetType(), Geometry::GEOMETRY_TYPE_POLY3);
    ASSERT_EQ(poly3_second.poly3_.GetA(), 0.0);
    ASSERT_EQ(poly3_second.poly3_.GetB(), 0.0);
    ASSERT_EQ(poly3_second.poly3_.GetC(), 1.0);
    ASSERT_EQ(poly3_second.poly3_.GetD(), -2.0);
    ASSERT_EQ(poly3_second.poly3_.GetPscale(), 1.0);

    Polynomial my_polynomial = poly3_second.GetPoly3();
    ASSERT_EQ(poly3_second.poly3_.GetA(), my_polynomial.GetA());
    ASSERT_EQ(poly3_second.poly3_.GetB(), my_polynomial.GetB());
    ASSERT_EQ(poly3_second.poly3_.GetC(), my_polynomial.GetC());
    ASSERT_EQ(poly3_second.poly3_.GetD(), my_polynomial.GetD());
    ASSERT_EQ(poly3_second.poly3_.GetPscale(), my_polynomial.GetPscale());
}

TEST_F(Poly3GeomTestFixture, TestEvaluateCurvatureDS)
{
    Poly3 poly3_second = poly3 = Poly3(0, 0, 0, 0, 1, 0, 0, 1, -2);

    ASSERT_EQ(poly3_second.EvaluateCurvatureDS(0), 2.0);
    ASSERT_THAT(poly3_second.EvaluateCurvatureDS(0.1), testing::AllOf(testing::Gt(0.8 - TRIG_ERR_MARGIN), testing::Lt(0.8 + TRIG_ERR_MARGIN)));
    ASSERT_THAT(poly3_second.EvaluateCurvatureDS(0.8), testing::AllOf(testing::Gt(-7.6 - TRIG_ERR_MARGIN), testing::Lt(-7.6 + TRIG_ERR_MARGIN)));
    ASSERT_THAT(poly3_second.EvaluateCurvatureDS(1.0), testing::AllOf(testing::Gt(-10 - TRIG_ERR_MARGIN), testing::Lt(-10 + TRIG_ERR_MARGIN)));
}

class Poly3GeomTestEvaluateDsCurvUmaxZero : public testing::TestWithParam<std::tuple<double, double, double, double>>
{
public:
protected:
    Poly3 poly3 = Poly3(0, 0, 0, 0, 1, 0, 0, 1, -2);
};

TEST_P(Poly3GeomTestEvaluateDsCurvUmaxZero, TestPoly3GeomEvaluateDsArgument)
{
    std::tuple<double, double, double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    double *x, *y, *h;
    double  my_x = poly3.GetX();
    double  my_y = poly3.GetY();
    double  my_h = poly3.GetHdg();
    x            = &my_x;
    y            = &my_y;
    h            = &my_h;
    poly3.EvaluateDS(std::get<0>(tuple), x, y, h);
    ASSERT_THAT(*x, testing::AllOf(testing::Gt(std::get<1>(tuple) - TRIG_ERR_MARGIN), testing::Lt(std::get<1>(tuple) + TRIG_ERR_MARGIN)));
    ASSERT_THAT(*y, testing::AllOf(testing::Gt(std::get<2>(tuple) - TRIG_ERR_MARGIN), testing::Lt(std::get<2>(tuple) + TRIG_ERR_MARGIN)));
    ASSERT_THAT(*h, testing::AllOf(testing::Gt(std::get<3>(tuple) - TRIG_ERR_MARGIN), testing::Lt(std::get<3>(tuple) + TRIG_ERR_MARGIN)));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluatePoly3DsArgumentParam,
                         Poly3GeomTestEvaluateDsCurvUmaxZero,
                         testing::Values(std::make_tuple(0.1, 0.1, 0.008, 0.139),
                                         std::make_tuple(0.2, 0.199, 0.024, 0.159),
                                         std::make_tuple(0.8, 0.721, -0.230, -1.033),
                                         std::make_tuple(1.0, 0.707, -0.207, -1.008)));

class Poly3GeomTestEvaluateDsCurvUmaxNonZero : public testing::TestWithParam<std::tuple<double, double, double, double>>
{
public:
protected:
    Poly3 poly3 = Poly3(0, 0, 0, 0, 2, 0, 0, 1, -2);
};

TEST_P(Poly3GeomTestEvaluateDsCurvUmaxNonZero, TestPoly3GeomEvaluateDsArgument)
{
    std::tuple<double, double, double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    double *x, *y, *h;
    double  my_x = poly3.GetX();
    double  my_y = poly3.GetY();
    double  my_h = poly3.GetHdg();
    x            = &my_x;
    y            = &my_y;
    h            = &my_h;
    poly3.SetUMax(1.0);
    poly3.EvaluateDS(std::get<0>(tuple), x, y, h);
    ASSERT_THAT(*x, testing::AllOf(testing::Gt(std::get<1>(tuple) - TRIG_ERR_MARGIN), testing::Lt(std::get<1>(tuple) + TRIG_ERR_MARGIN)));
    ASSERT_THAT(*y, testing::AllOf(testing::Gt(std::get<2>(tuple) - TRIG_ERR_MARGIN), testing::Lt(std::get<2>(tuple) + TRIG_ERR_MARGIN)));
    ASSERT_THAT(*h, testing::AllOf(testing::Gt(std::get<3>(tuple) - TRIG_ERR_MARGIN), testing::Lt(std::get<3>(tuple) + TRIG_ERR_MARGIN)));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluatePoly3DsArgumentParam,
                         Poly3GeomTestEvaluateDsCurvUmaxNonZero,
                         testing::Values(std::make_tuple(0.0, 0.0, 0.0, 0.0), std::make_tuple(2.0, 1.0, -1.0, -1.326)));

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> ParamPoly3 (Geometry) //////////
//////////////////////////////////////////////////////////////////////

class ParamPoly3GeomTestFixture : public testing::Test
{
public:
    ParamPoly3GeomTestFixture();
    ParamPoly3GeomTestFixture(double                 s,
                              double                 x,
                              double                 y,
                              double                 hdg,
                              double                 length,
                              double                 aU,
                              double                 bU,
                              double                 cU,
                              double                 dU,
                              double                 aV,
                              double                 bV,
                              double                 cV,
                              double                 dV,
                              ParamPoly3::PRangeType p_range);
    virtual ~ParamPoly3GeomTestFixture();

protected:
    ParamPoly3 parampoly3;
};

ParamPoly3GeomTestFixture::ParamPoly3GeomTestFixture()
{
}

ParamPoly3GeomTestFixture::~ParamPoly3GeomTestFixture()
{
}

TEST_F(ParamPoly3GeomTestFixture, TestParamPoly3ArgumentConstructor)
{
    EXPECT_EQ(parampoly3.GetType(), Geometry::GEOMETRY_TYPE_UNKNOWN);

    ParamPoly3 parampoly3_second = ParamPoly3(2, -1, 1, 5 * M_PI, 4, 1, -2, 3, -4, 1, -2, 3, -4, ParamPoly3::PRangeType::P_RANGE_NORMALIZED);
    EXPECT_EQ(parampoly3_second.GetType(), Geometry::GEOMETRY_TYPE_PARAM_POLY3);
    ASSERT_EQ(parampoly3_second.poly3U_.GetA(), 1.0);
    ASSERT_EQ(parampoly3_second.poly3U_.GetB(), -2.0);
    ASSERT_EQ(parampoly3_second.poly3U_.GetC(), 3.0);
    ASSERT_EQ(parampoly3_second.poly3U_.GetD(), -4.0);
    ASSERT_EQ(parampoly3_second.poly3U_.GetPscale(), 1.0 / 4.0);
    ASSERT_EQ(parampoly3_second.poly3V_.GetPscale(), 1.0 / 4.0);

    ParamPoly3 parampoly3_third = ParamPoly3(2, -1, 1, 5 * M_PI, 4, 1, -2, 3, -4, 1, -2, 3, -4, ParamPoly3::PRangeType::P_RANGE_UNKNOWN);
    ASSERT_EQ(parampoly3_third.poly3U_.GetPscale(), 1.0);
    ASSERT_EQ(parampoly3_third.poly3V_.GetPscale(), 1.0);

    Polynomial my_polynomialU = parampoly3_second.GetPoly3U();
    ASSERT_EQ(parampoly3_second.poly3U_.GetA(), my_polynomialU.GetA());
    ASSERT_EQ(parampoly3_second.poly3U_.GetB(), my_polynomialU.GetB());
    ASSERT_EQ(parampoly3_second.poly3U_.GetC(), my_polynomialU.GetC());
    ASSERT_EQ(parampoly3_second.poly3U_.GetD(), my_polynomialU.GetD());
    ASSERT_EQ(parampoly3_second.poly3U_.GetPscale(), my_polynomialU.GetPscale());

    Polynomial my_polynomialV = parampoly3_second.GetPoly3V();
    ASSERT_EQ(parampoly3_second.poly3V_.GetA(), my_polynomialV.GetA());
    ASSERT_EQ(parampoly3_second.poly3V_.GetB(), my_polynomialV.GetB());
    ASSERT_EQ(parampoly3_second.poly3V_.GetC(), my_polynomialV.GetC());
    ASSERT_EQ(parampoly3_second.poly3V_.GetD(), my_polynomialV.GetD());
    ASSERT_EQ(parampoly3_second.poly3V_.GetPscale(), my_polynomialV.GetPscale());
}

TEST_F(ParamPoly3GeomTestFixture, TestEvaluateCurvatureDS)
{
    ParamPoly3 parampoly3_second = ParamPoly3(0, 0, 0, 0, 1, 0, 10, 10, -10, 0, 10, -10, 10, ParamPoly3::PRangeType::P_RANGE_UNKNOWN);

    // https://www.desmos.com/calculator/cp3fyc0oxy

    EXPECT_NEAR(parampoly3_second.EvaluateCurvatureDS(0.0), -0.141421, 1e-5);
    EXPECT_NEAR(parampoly3_second.EvaluateCurvatureDS(0.1), -0.094853, 1e-5);
    EXPECT_NEAR(parampoly3_second.EvaluateCurvatureDS(0.5), 0.064564, 1e-5);
    EXPECT_NEAR(parampoly3_second.EvaluateCurvatureDS(1.0), 0.100000, 1e-5);
}

class ParamPoly3GeomTestEvaluateDsCurv : public testing::TestWithParam<std::tuple<double, double, double, double>>
{
public:
protected:
    ParamPoly3 parampoly3 = ParamPoly3(2, -1, 1, 5 * M_PI, 4, 1, -2, 3, -4, 1, -2, 3, -4, ParamPoly3::PRangeType::P_RANGE_UNKNOWN);
};

TEST_P(ParamPoly3GeomTestEvaluateDsCurv, TestParamPoly3GeomEvaluateDsArgument)
{
    std::tuple<double, double, double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    double *x, *y, *h;
    double  my_x = parampoly3.GetX();
    double  my_y = parampoly3.GetY();
    double  my_h = parampoly3.GetHdg();
    x            = &my_x;
    y            = &my_y;
    h            = &my_h;
    parampoly3.EvaluateDS(std::get<0>(tuple), x, y, h);
    ASSERT_THAT(*x, testing::AllOf(testing::Gt(std::get<1>(tuple) - TRIG_ERR_MARGIN), testing::Lt(std::get<1>(tuple) + TRIG_ERR_MARGIN)));
    ASSERT_THAT(*y, testing::AllOf(testing::Gt(std::get<2>(tuple) - TRIG_ERR_MARGIN), testing::Lt(std::get<2>(tuple) + TRIG_ERR_MARGIN)));
    ASSERT_EQ(*h, std::get<3>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluateParamPoly3DsArgumentParam,
                         ParamPoly3GeomTestEvaluateDsCurv,
                         testing::Values(std::make_tuple(0.0, -2.0, 0.0, M_PI + atan2(-2.0, -2.0)),
                                         std::make_tuple(10.0, 214.0, 216.0, M_PI + atan2(-2.0, -2.0))));

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> Elevation /////////////////////////////
//////////////////////////////////////////////////////////////////////

class ElevationTestFixture : public testing::Test
{
public:
    ElevationTestFixture();
    ElevationTestFixture(double s, double a, double b, double c, double d);
    virtual ~ElevationTestFixture();

protected:
    Elevation elevation;
};

ElevationTestFixture::ElevationTestFixture()
{
}

ElevationTestFixture::~ElevationTestFixture()
{
}

TEST_F(ElevationTestFixture, TestElevation)
{
    ASSERT_EQ(elevation.GetLength(), 0.0);
    ASSERT_EQ(elevation.GetS(), 0.0);

    elevation.SetLength(4.0);
    ASSERT_EQ(elevation.GetLength(), 4.0);

    Elevation elevation_second = Elevation(2.0, 1.0, -2.0, 3.0, -4.0);
    ASSERT_EQ(elevation_second.GetS(), 2.0);
    ASSERT_EQ(elevation_second.poly3_.GetA(), 1.0);
    ASSERT_EQ(elevation_second.poly3_.GetB(), -2.0);
    ASSERT_EQ(elevation_second.poly3_.GetC(), 3.0);
    ASSERT_EQ(elevation_second.poly3_.GetD(), -4.0);
    ASSERT_EQ(elevation_second.poly3_.GetPscale(), 1.0);
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> LaneLink /////////////////////////////
//////////////////////////////////////////////////////////////////////

TEST(LaneLinkTest, DefaultConstructor)
{
    LaneLink lane_link(NONE, 0);
    EXPECT_EQ(NONE, lane_link.GetType());
    EXPECT_EQ(0, lane_link.GetId());
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> LaneWidth /////////////////////////////
//////////////////////////////////////////////////////////////////////

TEST(LaneWidthTest, DefaultConstructor)
{
    LaneWidth lane_width(1, 1, 1, 1, 1);
    EXPECT_EQ(1, lane_width.GetSOffset());
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> LaneBoundaryOSI ////////////////////////
//////////////////////////////////////////////////////////////////////

// TODO
// TODO
// TODO

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> LaneRoadMarkTypeLine ///////////////////
//////////////////////////////////////////////////////////////////////

class LaneRoadMarkTypeLineTest
    : public ::testing::TestWithParam<std::tuple<double, double, double, double, LaneRoadMarkTypeLine::RoadMarkTypeLineRule, double>>
{
};
// inp: length,space,t_offset,s_offset,rule,width

TEST_P(LaneRoadMarkTypeLineTest, DefaultConstructor)
{
    LaneRoadMarkTypeLine lane_roadmarking = LaneRoadMarkTypeLine(std::get<0>(GetParam()),
                                                                 std::get<1>(GetParam()),
                                                                 std::get<2>(GetParam()),
                                                                 std::get<3>(GetParam()),
                                                                 std::get<4>(GetParam()),
                                                                 std::get<5>(GetParam()));

    EXPECT_EQ(lane_roadmarking.GetLength(), std::get<0>(GetParam()));
    EXPECT_EQ(lane_roadmarking.GetSpace(), std::get<1>(GetParam()));
    EXPECT_EQ(lane_roadmarking.GetTOffset(), std::get<2>(GetParam()));
    EXPECT_EQ(lane_roadmarking.GetSOffset(), std::get<3>(GetParam()));

    // Test SetGlobalId method
    // OSI related stuff not implemented yet
    // lane_roadmarking.SetGlobalId();
    // EXPECT_EQ(lane_roadmarking.GetGlobalId(), 0);
}

INSTANTIATE_TEST_SUITE_P(LaneRoadMarkTypeLineTests,
                         LaneRoadMarkTypeLineTest,
                         ::testing::Values(std::make_tuple(100, 100, 0, 0, LaneRoadMarkTypeLine::NO_PASSING, 2),
                                           std::make_tuple(10, 10, -1, 1, LaneRoadMarkTypeLine::CAUTION, 6)));

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> LaneRoadMarkType ///////////////////////
//////////////////////////////////////////////////////////////////////

class LaneRoadMarkTypeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        lane_test_0 = std::make_unique<LaneRoadMarkType>("test", 0.2);
    }
    std::unique_ptr<LaneRoadMarkType> lane_test_0;
};

TEST_F(LaneRoadMarkTypeTest, DefaultConstructor)
{
    EXPECT_EQ(lane_test_0->GetName(), "test");
    EXPECT_EQ(lane_test_0->GetWidth(), 0.2);
}

TEST_F(LaneRoadMarkTypeTest, AddLine)
{
    std::shared_ptr<LaneRoadMarkTypeLine> line_0 = std::make_shared<LaneRoadMarkTypeLine>(100, 100, 0, 0, LaneRoadMarkTypeLine::NO_PASSING, 2);
    lane_test_0->AddLine(line_0);
    EXPECT_EQ(lane_test_0->GetNumberOfRoadMarkTypeLines(), 1);

    // test GetLaneRoadMarkTypeLineByIdx method
    EXPECT_EQ(lane_test_0->GetLaneRoadMarkTypeLineByIdx(0)->GetLength(), line_0->GetLength());
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> LaneRoadMark ///////////////////////////
//////////////////////////////////////////////////////////////////////

class LaneRoadMarkTest : public ::testing::TestWithParam<std::tuple<double,
                                                                    LaneRoadMark::RoadMarkType,
                                                                    LaneRoadMark::RoadMarkWeight,
                                                                    RoadMarkColor,
                                                                    LaneRoadMark::RoadMarkMaterial,
                                                                    LaneRoadMark::RoadMarkLaneChange,
                                                                    double,
                                                                    double>>
{
};
// inp: s_offset,type,weight,color,material,lane_change,width,height

TEST_P(LaneRoadMarkTest, DefaultConstructor)
{
    LaneRoadMark lane_test_0 = LaneRoadMark(std::get<0>(GetParam()),
                                            std::get<1>(GetParam()),
                                            std::get<2>(GetParam()),
                                            std::get<3>(GetParam()),
                                            std::get<4>(GetParam()),
                                            std::get<5>(GetParam()),
                                            std::get<6>(GetParam()),
                                            std::get<7>(GetParam()));

    EXPECT_EQ(lane_test_0.GetSOffset(), std::get<0>(GetParam()));
    EXPECT_EQ(lane_test_0.GetType(), std::get<1>(GetParam()));
    EXPECT_EQ(lane_test_0.GetColor(), std::get<3>(GetParam()));
    EXPECT_EQ(lane_test_0.GetWidth(), std::get<6>(GetParam()));
    EXPECT_EQ(lane_test_0.GetHeight(), std::get<7>(GetParam()));

    std::shared_ptr<LaneRoadMarkType> type_test_0 = std::make_shared<LaneRoadMarkType>("test", 0.2);
    lane_test_0.AddType(type_test_0);
    EXPECT_EQ(lane_test_0.GetNumberOfRoadMarkTypes(), 1);
    EXPECT_EQ(lane_test_0.GetLaneRoadMarkTypeByIdx(0)->GetName(), type_test_0->GetName());
}

INSTANTIATE_TEST_SUITE_P(LaneRoadMarkTests,
                         LaneRoadMarkTest,
                         ::testing::Values(std::make_tuple(0,
                                                           LaneRoadMark::NONE_TYPE,
                                                           LaneRoadMark::STANDARD,
                                                           RoadMarkColor::STANDARD_COLOR,
                                                           LaneRoadMark::STANDARD_MATERIAL,
                                                           LaneRoadMark::INCREASE,
                                                           0.2,
                                                           0),
                                           std::make_tuple(100,
                                                           LaneRoadMark::SOLID,
                                                           LaneRoadMark::BOLD,
                                                           RoadMarkColor::BLUE,
                                                           LaneRoadMark::STANDARD_MATERIAL,
                                                           LaneRoadMark::DECREASE,
                                                           0.2,
                                                           -1)));

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> LaneOffSet ///////////////////////////
//////////////////////////////////////////////////////////////////////

class LaneOffsetTestFixture : public testing::Test
{
public:
    LaneOffsetTestFixture();
    LaneOffsetTestFixture(double s, double a, double b, double c, double d);
    virtual ~LaneOffsetTestFixture();

protected:
    LaneOffset laneoffset;
};

LaneOffsetTestFixture::LaneOffsetTestFixture()
{
}

LaneOffsetTestFixture::~LaneOffsetTestFixture()
{
}

TEST_F(LaneOffsetTestFixture, TestLaneOffSetCommon)
{
    ASSERT_EQ(laneoffset.GetLength(), 0.0);
    ASSERT_EQ(laneoffset.GetS(), 0.0);

    laneoffset.SetLength(4.0);
    ASSERT_EQ(laneoffset.GetLength(), 4.0);

    LaneOffset laneoffset_second = LaneOffset(2.0, 1.0, -2.0, 3.0, -4.0);
    laneoffset_second.Set(2.0, 1.0, -2.0, 3.0, -4.0);
    ASSERT_EQ(laneoffset_second.GetS(), 2.0);
    ASSERT_EQ(laneoffset_second.GetPolynomial().GetA(), 1.0);
    ASSERT_EQ(laneoffset_second.GetPolynomial().GetB(), -2.0);
    ASSERT_EQ(laneoffset_second.GetPolynomial().GetC(), 3.0);
    ASSERT_EQ(laneoffset_second.GetPolynomial().GetD(), -4.0);
    ASSERT_EQ(laneoffset_second.GetPolynomial().GetPscale(), 1.0);
}

class LaneOffsetGetLaneOffsetTest : public testing::TestWithParam<std::tuple<double, double>>
{
public:
protected:
    LaneOffset laneoffset = LaneOffset(2.0, 1.0, -2.0, 3.0, -4.0);
};

TEST_P(LaneOffsetGetLaneOffsetTest, TestGetLaneOffset)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_EQ(laneoffset.GetLaneOffset(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestGetLaneOffsetParam,
                         LaneOffsetGetLaneOffsetTest,
                         testing::Values(std::make_tuple(0.0, 49.0), std::make_tuple(10.0, -1871.0)));

class LaneOffsetGetLaneOffsetPrimTest : public testing::TestWithParam<std::tuple<double, double>>
{
public:
protected:
    LaneOffset laneoffset = LaneOffset(2.0, 1.0, -2.0, 3.0, -4.0);
};

TEST_P(LaneOffsetGetLaneOffsetPrimTest, TestGetLaneOffsetPrim)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_EQ(laneoffset.GetLaneOffsetPrim(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestGetLaneOffsetPrimParam,
                         LaneOffsetGetLaneOffsetPrimTest,
                         testing::Values(std::make_tuple(0.0, -62), std::make_tuple(10.0, -722.0)));

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> Lane ///////////////////////////////////
//////////////////////////////////////////////////////////////////////

class LaneTestFixture : public testing::Test
{
public:
    LaneTestFixture();
    LaneTestFixture(double s, double a, double b, double c, double d);
    virtual ~LaneTestFixture();

protected:
    Lane lane;
};

LaneTestFixture::LaneTestFixture()
{
}

LaneTestFixture::~LaneTestFixture()
{
}

TEST_F(LaneTestFixture, TestLaneBaseGetConstructor)
{
    ASSERT_EQ(lane.GetId(), 0);
    ASSERT_EQ(lane.GetLaneType(), Lane::LaneType::LANE_TYPE_NONE);
    ASSERT_EQ(lane.GetGlobalId(), 0.0);

    Lane lane_second = Lane(1, Lane::LaneType::LANE_TYPE_DRIVING);
    ASSERT_EQ(lane_second.GetId(), 1);
    ASSERT_EQ(lane_second.GetLaneType(), Lane::LaneType::LANE_TYPE_DRIVING);
    ASSERT_EQ(lane_second.GetGlobalId(), 0.0);
}

TEST_F(LaneTestFixture, TestLaneAddFunctions)
{
    ASSERT_EQ(lane.GetNumberOfLinks(), 0);
    ASSERT_EQ(lane.GetNumberOfRoadMarks(), 0);
    ASSERT_EQ(lane.GetNumberOfLaneWidths(), 0);

    LaneLink     *lanelink     = new LaneLink(LinkType::SUCCESSOR, 3);
    LaneWidth    *lanewidth    = new LaneWidth(2.0, 1.0, -2.0, -3.0, 4.0);
    LaneRoadMark *laneroadmark = new LaneRoadMark(2.0,
                                                  LaneRoadMark::RoadMarkType::BROKEN,
                                                  LaneRoadMark::RoadMarkWeight::STANDARD,
                                                  RoadMarkColor::RED,
                                                  LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL,
                                                  LaneRoadMark::RoadMarkLaneChange::BOTH,
                                                  4.0,
                                                  2.0);

    lane.AddLink(lanelink);
    lane.AddLaneWidth(lanewidth);
    lane.AddLaneRoadMark(laneroadmark);

    ASSERT_EQ(lane.GetNumberOfLinks(), 1);
    ASSERT_EQ(lane.GetNumberOfLaneWidths(), 1);
    ASSERT_EQ(lane.GetNumberOfRoadMarks(), 1);
}

TEST_F(LaneTestFixture, TestLaneGetLink)
{
    LaneLink *lanelink = new LaneLink(LinkType::SUCCESSOR, 3);
    lane.AddLink(lanelink);
    LaneLink *mylanelink = lane.GetLink(LinkType::PREDECESSOR);
    LaneLink *dummylink  = 0;
    ASSERT_EQ(mylanelink, dummylink);
    LaneLink *mylanelink_second = lane.GetLink(LinkType::SUCCESSOR);
    ASSERT_EQ(mylanelink_second->GetType(), LinkType::SUCCESSOR);
    ASSERT_EQ(mylanelink_second->GetId(), 3);
}

TEST_F(LaneTestFixture, TestLaneGetWidth)
{
    LaneWidth *lanewidth        = new LaneWidth(2.0, 1.0, -2.0, 3.0, -4.0);
    LaneWidth *lanewidth_second = new LaneWidth(10.0, 2.0, -3.0, 4.0, -5.0);

    LaneWidth *dummywidth = 0;
    ASSERT_EQ(lane.GetWidthByS(0), dummywidth);
    ASSERT_EQ(lane.GetWidthByS(5), dummywidth);
    ASSERT_EQ(lane.GetWidthByS(10), dummywidth);

    lane.AddLaneWidth(lanewidth);
    lane.AddLaneWidth(lanewidth_second);

    ASSERT_THROW(lane.GetWidthByIndex(-1), std::runtime_error);
    ASSERT_THROW(lane.GetWidthByIndex(2), std::runtime_error);
    ASSERT_THROW(lane.GetWidthByIndex(3), std::runtime_error);

    LaneWidth *mylanewidth   = lane.GetWidthByIndex(0);
    LaneWidth *mylanewidth_s = lane.GetWidthByS(1);
    ASSERT_EQ(mylanewidth->GetSOffset(), 2.0);
    ASSERT_EQ(mylanewidth->poly3_.GetA(), 1.0);
    ASSERT_EQ(mylanewidth->poly3_.GetB(), -2.0);
    ASSERT_EQ(mylanewidth->poly3_.GetC(), 3.0);
    ASSERT_EQ(mylanewidth->poly3_.GetD(), -4.0);
    ASSERT_EQ(mylanewidth->poly3_.GetPscale(), 1.0);
    ASSERT_EQ(mylanewidth_s->GetSOffset(), 2.0);
    ASSERT_EQ(mylanewidth_s->poly3_.GetA(), 1.0);
    ASSERT_EQ(mylanewidth_s->poly3_.GetB(), -2.0);
    ASSERT_EQ(mylanewidth_s->poly3_.GetC(), 3.0);
    ASSERT_EQ(mylanewidth_s->poly3_.GetD(), -4.0);
    ASSERT_EQ(mylanewidth_s->poly3_.GetPscale(), 1.0);

    LaneWidth *mylanewidth_second   = lane.GetWidthByIndex(1);
    LaneWidth *mylanewidth_s_second = lane.GetWidthByS(5);
    ASSERT_EQ(mylanewidth_second->GetSOffset(), 10.0);
    ASSERT_EQ(mylanewidth_second->poly3_.GetA(), 2.0);
    ASSERT_EQ(mylanewidth_second->poly3_.GetB(), -3.0);
    ASSERT_EQ(mylanewidth_second->poly3_.GetC(), 4.0);
    ASSERT_EQ(mylanewidth_second->poly3_.GetD(), -5.0);
    ASSERT_EQ(mylanewidth_second->poly3_.GetPscale(), 1.0);
    ASSERT_EQ(mylanewidth_s_second->GetSOffset(), 2.0);
    ASSERT_EQ(mylanewidth_s_second->poly3_.GetA(), 1.0);
    ASSERT_EQ(mylanewidth_s_second->poly3_.GetB(), -2.0);
    ASSERT_EQ(mylanewidth_s_second->poly3_.GetC(), 3.0);
    ASSERT_EQ(mylanewidth_s_second->poly3_.GetD(), -4.0);
    ASSERT_EQ(mylanewidth_s_second->poly3_.GetPscale(), 1.0);

    LaneWidth *mylanewidth_s_final = lane.GetWidthByS(1000000);
    ASSERT_EQ(mylanewidth_s_final->GetSOffset(), 10.0);
    ASSERT_EQ(mylanewidth_s_final->poly3_.GetA(), 2.0);
    ASSERT_EQ(mylanewidth_s_final->poly3_.GetB(), -3.0);
    ASSERT_EQ(mylanewidth_s_final->poly3_.GetC(), 4.0);
    ASSERT_EQ(mylanewidth_s_final->poly3_.GetD(), -5.0);
    ASSERT_EQ(mylanewidth_s_final->poly3_.GetPscale(), 1.0);
}

TEST_F(LaneTestFixture, TestLaneGetRoadMark)
{
    LaneRoadMark *laneroadmark = new LaneRoadMark(2.0,
                                                  LaneRoadMark::RoadMarkType::BROKEN,
                                                  LaneRoadMark::RoadMarkWeight::STANDARD,
                                                  RoadMarkColor::RED,
                                                  LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL,
                                                  LaneRoadMark::RoadMarkLaneChange::BOTH,
                                                  4.0,
                                                  2.0);
    lane.AddLaneRoadMark(laneroadmark);

    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(-1), std::runtime_error);
    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(1), std::runtime_error);
    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(2), std::runtime_error);

    LaneRoadMark *mylaneroadmark = lane.GetLaneRoadMarkByIdx(0);
    ASSERT_EQ(mylaneroadmark->GetSOffset(), 2.0);
    ASSERT_EQ(mylaneroadmark->GetWidth(), 4.0);
    ASSERT_EQ(mylaneroadmark->GetHeight(), 2.0);
    ASSERT_EQ(mylaneroadmark->GetType(), LaneRoadMark::RoadMarkType::BROKEN);
    ASSERT_EQ(mylaneroadmark->GetWeight(), LaneRoadMark::RoadMarkWeight::STANDARD);
    ASSERT_EQ(mylaneroadmark->GetColor(), RoadMarkColor::RED);
    ASSERT_EQ(mylaneroadmark->GetMaterial(), LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL);
    ASSERT_EQ(mylaneroadmark->GetLaneChange(), LaneRoadMark::RoadMarkLaneChange::BOTH);
}

TEST_F(LaneTestFixture, TestLaneGetRoadMark2)
{
    LaneRoadMark *laneroadmark = new LaneRoadMark(2.0,
                                                  LaneRoadMark::RoadMarkType::BROKEN_BROKEN,
                                                  LaneRoadMark::RoadMarkWeight::STANDARD,
                                                  RoadMarkColor::RED,
                                                  LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL,
                                                  LaneRoadMark::RoadMarkLaneChange::BOTH,
                                                  4.0,
                                                  2.0);
    lane.AddLaneRoadMark(laneroadmark);

    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(-1), std::runtime_error);
    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(1), std::runtime_error);
    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(2), std::runtime_error);

    LaneRoadMark *mylaneroadmark = lane.GetLaneRoadMarkByIdx(0);
    ASSERT_EQ(mylaneroadmark->GetType(), LaneRoadMark::RoadMarkType::BROKEN_BROKEN);
}

TEST_F(LaneTestFixture, TestLaneGetRoadMark3)
{
    LaneRoadMark *laneroadmark = new LaneRoadMark(2.0,
                                                  LaneRoadMark::RoadMarkType::SOLID_SOLID,
                                                  LaneRoadMark::RoadMarkWeight::STANDARD,
                                                  RoadMarkColor::RED,
                                                  LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL,
                                                  LaneRoadMark::RoadMarkLaneChange::BOTH,
                                                  4.0,
                                                  2.0);
    lane.AddLaneRoadMark(laneroadmark);

    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(-1), std::runtime_error);
    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(1), std::runtime_error);
    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(2), std::runtime_error);

    LaneRoadMark *mylaneroadmark = lane.GetLaneRoadMarkByIdx(0);
    ASSERT_EQ(mylaneroadmark->GetType(), LaneRoadMark::RoadMarkType::SOLID_SOLID);
}

TEST_F(LaneTestFixture, TestLaneGetRoadMark4)
{
    LaneRoadMark *laneroadmark = new LaneRoadMark(2.0,
                                                  LaneRoadMark::RoadMarkType::BROKEN_SOLID,
                                                  LaneRoadMark::RoadMarkWeight::STANDARD,
                                                  RoadMarkColor::RED,
                                                  LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL,
                                                  LaneRoadMark::RoadMarkLaneChange::BOTH,
                                                  4.0,
                                                  2.0);
    lane.AddLaneRoadMark(laneroadmark);

    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(-1), std::runtime_error);
    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(1), std::runtime_error);
    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(2), std::runtime_error);

    LaneRoadMark *mylaneroadmark = lane.GetLaneRoadMarkByIdx(0);
    ASSERT_EQ(mylaneroadmark->GetType(), LaneRoadMark::RoadMarkType::BROKEN_SOLID);
}

/*
TODO: The test for Lane::GetRoadMarkInfoByS.
*/
#if 0
TEST_F(LaneTestFixture, TestLaneGetOSIPoints)
{
    OSIPoints *osi_points = lane.GetOSIPoints();
    ASSERT_EQ(osi_points->GetPoints().size(), 0);

    std::vector<PointStruct> osi_points_test_set =
    {
        {0, 0, 0, 0, 0},
        {-1, -1, -1, -1, -1},
        {2, 2, 2, 2, 2}
    };

    lane.osi_points_.Set(osi_points_test_set);

    OSIPoints *osi_points_second = lane.GetOSIPoints();
    ASSERT_EQ(osi_points->GetPoints().size(), 3);

    ASSERT_EQ(osi_points->GetPoint(0).s, 0);
    ASSERT_EQ(osi_points->GetPoint(1).s, -1);
    ASSERT_EQ(osi_points->GetPoint(2).s, 2);
}
#endif

TEST_F(LaneTestFixture, TestLaneGetLineGlobalIds)
{
    LaneRoadMark *laneroadmark        = new LaneRoadMark(0,
                                                  LaneRoadMark::RoadMarkType::BROKEN,
                                                  LaneRoadMark::RoadMarkWeight::STANDARD,
                                                  RoadMarkColor::STANDARD_COLOR,
                                                  LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL,
                                                  LaneRoadMark::RoadMarkLaneChange::BOTH,
                                                  1.0,
                                                  1.0);
    LaneRoadMark *laneroadmark_second = new LaneRoadMark(50,
                                                         LaneRoadMark::RoadMarkType::BROKEN,
                                                         LaneRoadMark::RoadMarkWeight::STANDARD,
                                                         RoadMarkColor::STANDARD_COLOR,
                                                         LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL,
                                                         LaneRoadMark::RoadMarkLaneChange::BOTH,
                                                         2.0,
                                                         2.0);

    std::shared_ptr<LaneRoadMarkType> laneroadmarktype        = std::make_shared<LaneRoadMarkType>("type1", 1.0);
    std::shared_ptr<LaneRoadMarkType> laneroadmarktype_second = std::make_shared<LaneRoadMarkType>("type2", 1.0);

    std::shared_ptr<LaneRoadMarkTypeLine> laneRoadMarktypeline =
        std::make_shared<LaneRoadMarkTypeLine>(3.0, 1.0, 0.5, 0.0, LaneRoadMarkTypeLine::RoadMarkTypeLineRule::CAUTION, 1.0);
    std::shared_ptr<LaneRoadMarkTypeLine> laneRoadMarktypeline_second =
        std::make_shared<LaneRoadMarkTypeLine>(3.0, 1.0, 0.5, 50.0, LaneRoadMarkTypeLine::RoadMarkTypeLineRule::CAUTION, 1.0);

    laneroadmark->AddType(laneroadmarktype);
    laneroadmark->AddType(laneroadmarktype_second);

    laneroadmarktype->AddLine(laneRoadMarktypeline);
    laneroadmarktype->AddLine(laneRoadMarktypeline_second);
    laneroadmarktype_second->AddLine(laneRoadMarktypeline_second);

    lane.AddLaneRoadMark(laneroadmark);
    lane.AddLaneRoadMark(laneroadmark_second);

    OpenDrive *odr = new OpenDrive();
    odr->InitGlobalLaneIds();
    laneRoadMarktypeline->SetGlobalId();
    laneRoadMarktypeline_second->SetGlobalId();

    std::vector<int> all_glob_ids = lane.GetLineGlobalIds();

    ASSERT_THAT(all_glob_ids.size(), 3);
    ASSERT_THAT(laneroadmarktype->GetLaneRoadMarkTypeLineByIdx(0)->GetGlobalId(), 0);
    ASSERT_THAT(laneroadmarktype->GetLaneRoadMarkTypeLineByIdx(1)->GetGlobalId(), 1);
    ASSERT_THAT(laneroadmarktype_second->GetLaneRoadMarkTypeLineByIdx(0)->GetGlobalId(), 1);

    delete odr;
}

TEST(RoadTest, RoadWidthAllLanes)
{
    ASSERT_EQ(roadmanager::Position::LoadOpenDrive("../../../resources/xodr/soderleden.xodr"), true);
    roadmanager::OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 5);

    Road *road = odr->GetRoadByIdx(1);
    EXPECT_EQ(road->GetId(), 1);

    EXPECT_NEAR(road->GetWidth(0, -1), 5.8, 1e-5);
    EXPECT_NEAR(road->GetWidth(0, 1), 2.3, 1e-5);
    EXPECT_NEAR(road->GetWidth(0, 0), 8.1, 1e-5);

    odr->Clear();
}

TEST(RoadTest, RoadWidthDrivingLanes)
{
    roadmanager::OpenDrive *odr = new OpenDrive("../../../resources/xodr/soderleden.xodr");

    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 5);

    Road *road = odr->GetRoadByIdx(1);
    EXPECT_EQ(road->GetId(), 1);

    EXPECT_DOUBLE_EQ(road->GetWidth(0, -1, Lane::LaneType::LANE_TYPE_ANY_DRIVING), 3.5);
    EXPECT_DOUBLE_EQ(road->GetWidth(0, 1, Lane::LaneType::LANE_TYPE_ANY_DRIVING), 0.0);
    EXPECT_DOUBLE_EQ(road->GetWidth(0, 0, Lane::LaneType::LANE_TYPE_ANY_DRIVING), 3.5);

    delete odr;
}

TEST(TrajectoryTest, PolyLineBase_YawInterpolation)
{
    PolyLineBase pline;
    TrajVertex   v = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ID_UNDEFINED, 0.0, 0.0, 0.0, 0.0, 0};

    // Simple case - no interpolation
    pline.AddVertex({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ID_UNDEFINED, 0.0, 0.0, 0.0, 0.0, Position::PosMode::H_ABS});
    pline.AddVertex({std::nan(""), 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, ID_UNDEFINED, 0.0, 0.0, 0.0, 0.0, Position::PosMode::H_ABS});
    pline.Evaluate(0.5, v);

    EXPECT_NEAR(v.x, 0.5, 1e-5);
    EXPECT_NEAR(v.y, 0.0, 1e-5);
    EXPECT_NEAR(v.h, 0.0, 1e-5);

    // Same but with interpolation
    pline.Reset(true);
    pline.interpolation_mode_ = PolyLineBase::InterpolationMode::INTERPOLATE_SEGMENT;
    pline.AddVertex({std::nan(""), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ID_UNDEFINED, 0.0, 0.0, 0.0, 0.0, Position::PosMode::H_ABS});
    pline.AddVertex({std::nan(""), 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, ID_UNDEFINED, 0.0, 0.0, 0.0, 0.0, Position::PosMode::H_ABS});
    pline.Evaluate(0.5, v);

    EXPECT_NEAR(v.x, 0.5, 1e-5);
    EXPECT_NEAR(v.y, 0.0, 1e-5);
    EXPECT_NEAR(v.h, 0.5, 1e-5);

    // Wrap around case 1
    pline.Reset(true);
    pline.interpolation_mode_ = PolyLineBase::InterpolationMode::INTERPOLATE_SEGMENT;
    pline.AddVertex({std::nan(""), 0.0, 0.0, 0.0, 2 * M_PI - 0.01, 0.0, 0.0, ID_UNDEFINED, 0.0, 0.0, 0.0, 0.0, Position::PosMode::H_ABS});
    pline.AddVertex({std::nan(""), 1.0, 0.0, 0.0, 0.09, 0.0, 0.0, ID_UNDEFINED, 0.0, 0.0, 0.0, 0.0, Position::PosMode::H_ABS});
    pline.Evaluate(0.5, v);

    EXPECT_NEAR(v.x, 0.5, 1e-5);
    EXPECT_NEAR(v.y, 0.0, 1e-5);
    EXPECT_NEAR(v.h, 0.04, 1e-5);

    // Wrap around case 2
    pline.Reset(true);
    pline.interpolation_mode_ = PolyLineBase::InterpolationMode::INTERPOLATE_SEGMENT;
    pline.AddVertex({std::nan(""), 10.0, 10.0, 0.0, 0.1, 0.0, 0.0, ID_UNDEFINED, 0.0, 0.0, 0.0, 0.0, Position::PosMode::H_ABS});
    pline.AddVertex({std::nan(""), 10.0, 0.0, 0.0, -0.5, 0.0, 0.0, ID_UNDEFINED, 0.0, 0.0, 0.0, 0.0, Position::PosMode::H_ABS});
    pline.Evaluate(5.0, v);

    EXPECT_NEAR(v.x, 10.0, 1e-5);
    EXPECT_NEAR(v.y, 5.0, 1e-5);
    EXPECT_NEAR(v.h, 6.0831853, 1e-5);

    // Wrap around case 3
    pline.Reset(true);
    pline.interpolation_mode_ = PolyLineBase::InterpolationMode::INTERPOLATE_SEGMENT;
    pline.AddVertex({std::nan(""), 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, ID_UNDEFINED, 0.0, 0.0, 0.0, 0.0, Position::PosMode::H_ABS});
    pline.AddVertex({std::nan(""), 0.0, 10.0, 0.0, 8.1, 0.0, 0.0, ID_UNDEFINED, 0.0, 0.0, 0.0, 0.0, Position::PosMode::H_ABS});
    pline.Evaluate(5.0, v);

    EXPECT_NEAR(v.x, 0.0, 1e-5);
    EXPECT_NEAR(v.y, 5.0, 1e-5);
    EXPECT_NEAR(v.h, 0.958407, 1e-5);
}

TEST(DistanceTest, CalcDistanceLong)
{
    double dist = 0.0;
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/fabriksgatan.xodr");
    OpenDrive *odr = Position::GetOpenDrive();

    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 16);

    Position pos0 = Position(0, 1, 10.0, 0);
    pos0.SetHeading(1.6);

    // move backwards 20 meter - road is slightly curved
    Position pos1 = Position(0, 1, 20.0, 0);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ENTITY, RelativeDistanceType::REL_DIST_EUCLIDIAN, dist), 0);
    EXPECT_NEAR(dist, -9.98797, 1e-5);

    // move through junction, measure euclidian
    pos1.SetLanePos(1, 1, 15.0, 0);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ENTITY, RelativeDistanceType::REL_DIST_EUCLIDIAN, dist), 0);
    EXPECT_NEAR(dist, 28.178276, 1e-5);

    // measure along road
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, dist), 0);
    EXPECT_NEAR(dist, 34.141086, 1e-5);

    // move back 1 m and measure along road
    pos1.SetLanePos(1, 1, 14.0, 0);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, dist), 0);
    EXPECT_NEAR(dist, 33.141086, 1e-5);

    // facing other direction should give negative distance, indicating the target position is behind
    pos0.SetHeading(5.0);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, dist), 0);
    EXPECT_NEAR(dist, -33.141086, 1e-5);

    // another lane should give same result
    pos1.SetLanePos(1, -1, 14.0, 0);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, dist), 0);
    EXPECT_NEAR(dist, -33.141086, 1e-5);

    // however the lateral distance should be affected - road 1 is opposite directed, so lane 1 should be on other side
    pos1.SetLanePos(1, 1, 14.0, 0);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, dist), 0);
    EXPECT_NEAR(dist, 3.5, 1e-5);

    // moving to the other lane should give no lateral distance
    pos1.SetLanePos(1, -1, 14.0, 0);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, dist), 0);
    EXPECT_NEAR(dist, 0.0, 1e-5);

    // Test a few other cases
    pos0.SetLanePos(3, -1, 5.0, -0.25);
    pos1.SetLanePos(0, -1, 15.0, 0.15);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, dist), 0);
    EXPECT_NEAR(dist, 0.4, 1e-5);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, dist), 0);
    EXPECT_NEAR(dist, 134.051729, 1e-5);

    pos0.SetLanePos(3, -1, 5.0, -0.25);
    pos1.SetLanePos(0, 1, 15.0, 0.15);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, dist), 0);
    EXPECT_NEAR(dist, 3.9, 1e-5);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, dist), 0);
    EXPECT_NEAR(dist, 134.051729, 1e-5);

    // starting from left lane will take another lane in intersection, increasing the total distance
    // Change position mode so that relative heading will be preserved when changing location.
    pos0.SetMode(roadmanager::Position::PosModeType::SET, roadmanager::Position::PosMode::H_REL);
    pos0.SetHeadingRelative(0.0);
    pos0.SetLanePos(3, 1, 5.0, -0.25);
    pos1.SetLanePos(0, -1, 15.0, 0.15);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, dist), 0);
    EXPECT_NEAR(dist, -3.1, 1e-5);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, dist), 0);
    EXPECT_NEAR(dist, 139.317791, 1e-5);

    // Route is found, but two lanes delta = 3.6 m
    pos0.SetLanePos(3, -1, 5.0, -0.25);
    pos1.SetLanePos(2, -1, 1.0, 0.15);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, dist), 0);
    EXPECT_NEAR(dist, -3.6, 1e-5);

    // No valid route is to be found between connecting roads (following directed connectivity in OpenDRIVE file)
    pos0.SetLanePos(16, -1, 5.0, -0.25);
    pos1.SetLanePos(8, -1, 1.0, 0.15);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, dist), -1);
    EXPECT_NEAR(dist, LARGE_NUMBER, 1e-5);

    // target position in front
    pos0.SetLanePos(2, -1, 300.0, -0.25);
    pos1.SetLanePos(1, -1, 1.0, 0.15);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ENTITY, RelativeDistanceType::REL_DIST_LATERAL, dist), 0);
    EXPECT_NEAR(dist, 10.663675, 1e-5);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ENTITY, RelativeDistanceType::REL_DIST_LONGITUDINAL, dist), 0);
    EXPECT_NEAR(dist, 13.481041, 1e-5);

    // target position behind
    pos0.SetLanePos(2, 1, 300.0, -0.25);
    pos0.SetHeading(M_PI_2);
    pos1.SetLanePos(1, -1, 1.0, 0.15);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ENTITY, RelativeDistanceType::REL_DIST_LATERAL, dist), 0);
    EXPECT_NEAR(dist, -9.48687, 1e-5);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ENTITY, RelativeDistanceType::REL_DIST_LONGITUDINAL, dist), 0);
    EXPECT_NEAR(dist, -11.96060, 1e-5);

    // Target position in front, to the right
    pos0.SetHeading(0.0);
    pos1.SetLanePos(1, -1, 1.0, 0.15);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ENTITY, RelativeDistanceType::REL_DIST_LATERAL, dist), 0);
    EXPECT_NEAR(dist, -11.96060, 1e-5);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ENTITY, RelativeDistanceType::REL_DIST_LONGITUDINAL, dist), 0);
    EXPECT_NEAR(dist, 9.48687, 1e-5);
}

TEST(NurbsTest, TestNurbsPosition)
{
    NurbsShape n(4);

    n.AddControlPoint(Position(-4.0, -4.0, 0.0, 0.0, 0.0, 0.0), 0.0, 1.0);
    n.AddControlPoint(Position(-2.0, 4.0, 0.0, 0.0, 0.0, 0.0), 0.0, 1.0);
    n.AddControlPoint(Position(2.0, -4.0, 0.0, 0.0, 0.0, 0.0), 0.0, 1.0);
    n.AddControlPoint(Position(4.0, 4.0, 0.0, 0.0, 0.0, 0.0), 0.0, 1.0);

    std::vector<double> knots = {0, 0, 0, 0, 1, 1, 1, 1};
    n.AddKnots(knots);
    n.CalculatePolyLine();

    TrajVertex v;

    n.Evaluate(0, Shape::TrajectoryParamType::TRAJ_PARAM_TYPE_S, v);
    EXPECT_DOUBLE_EQ(v.x, -4.0);
    EXPECT_DOUBLE_EQ(v.y, -4.0);

    n.Evaluate(0.5 * n.GetLength(), Shape::TrajectoryParamType::TRAJ_PARAM_TYPE_S, v);
    EXPECT_NEAR(v.x, 0.0, 1e-3);
    EXPECT_NEAR(v.y, 0.0, 1e-3);

    n.Evaluate(1.0 * n.GetLength(), Shape::TrajectoryParamType::TRAJ_PARAM_TYPE_S, v);
    EXPECT_NEAR(v.x, 4.0, 1e-3);
    EXPECT_NEAR(v.y, 4.0, 1e-3);

    n.Evaluate(0.40045 * n.GetLength(), Shape::TrajectoryParamType::TRAJ_PARAM_TYPE_S, v);
    EXPECT_NEAR(v.x, -1.248623, 1e-5);
    EXPECT_NEAR(v.y, -0.087722, 1e-5);
    EXPECT_NEAR(v.param, 0.360046, 1e-5);
}

TEST(Route, TestAssignRoute)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/fabriksgatan.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 16);

    const int nrWaypoints = 6;
    Route    *route       = new Route;
    Position  routepos[nrWaypoints];
    routepos[0].SetLanePos(0, 1, 10.0, 0);
    routepos[0].SetHeadingRelative(M_PI);
    routepos[1].SetLanePos(0, 1, 7.0, 0);  // Add extra waypoint on first road - should be removed
    routepos[0].SetHeadingRelative(M_PI);
    routepos[2].SetLanePos(8, -1, 2.0, 0);
    routepos[3].SetLanePos(8, -1, 4.0, 0);  // Add extra waypoint on same road - previous should be ignored
    routepos[4].SetLanePos(1, -1, 2.0, 0);
    routepos[5].SetLanePos(1, -1, 1.0, 0);  // Add extra waypoint on same road - previous should be ignored
    for (int i = 0; i < nrWaypoints; i++)
    {
        route->AddWaypoint(routepos[i]);
    }

    EXPECT_EQ(route->minimal_waypoints_.size(), 3);
    EXPECT_DOUBLE_EQ(route->minimal_waypoints_[0].GetTrackId(), 0);
    EXPECT_DOUBLE_EQ(route->minimal_waypoints_[0].GetS(), 10.0);
    EXPECT_DOUBLE_EQ(route->minimal_waypoints_[1].GetTrackId(), 8);
    EXPECT_DOUBLE_EQ(route->minimal_waypoints_[1].GetS(), 4.0);
    EXPECT_DOUBLE_EQ(route->minimal_waypoints_[2].GetTrackId(), 1);
    EXPECT_DOUBLE_EQ(route->minimal_waypoints_[2].GetS(), 1.0);

    Position pos0 = Position(0, 1, 9.0, 0.5);
    EXPECT_EQ(pos0.SetRoute(route), 0);
    EXPECT_DOUBLE_EQ(pos0.GetRouteS(), 1.0);

    // Set a position in intersection, near route
    pos0.SetLanePos(8, -1, 1.5, -0.5);
    EXPECT_EQ(pos0.SetRoute(route), 0);
    EXPECT_DOUBLE_EQ(pos0.GetRouteS(), 11.5);

    // Set a position in intersection, at a lane not part of the route
    pos0.SetLanePos(16, -1, 1.0, 0.0);
    EXPECT_EQ(pos0.SetRoute(route), -1);  // pos not along the route
}

TEST(GeoReferenceTest, TestNoGeoReferenceSimpleRoad)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/straight_500m_signs.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    GeoReference *geo_ref = odr->GetGeoReference();
    EXPECT_EQ(std::isnan(geo_ref->lat_0_), true);
    EXPECT_EQ(std::isnan(geo_ref->lon_0_), true);
    EXPECT_EQ(geo_ref->proj_, "");
    EXPECT_EQ(std::isnan(geo_ref->k_0_), true);
    EXPECT_EQ(std::isnan(geo_ref->x_0_), true);
    EXPECT_EQ(std::isnan(geo_ref->y_0_), true);
    EXPECT_EQ(geo_ref->datum_, "");
    EXPECT_EQ(geo_ref->geo_id_grids_, "");
    EXPECT_EQ(geo_ref->vunits_, "");
    EXPECT_EQ(geo_ref->units_, "");
    EXPECT_EQ(geo_ref->ellps_, "");
    EXPECT_EQ(std::isnan(geo_ref->zone_), true);

    std::string geo_ref_str = odr->GetGeoReferenceAsString();
    EXPECT_EQ(geo_ref_str, "");
}

TEST(GeoReferenceTest, TestGeoReferenceSimpleRoad)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/curve_r100.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    GeoReference *geo_ref = odr->GetGeoReference();
    EXPECT_EQ(geo_ref->lat_0_, 37.35429341239328);
    EXPECT_EQ(geo_ref->lon_0_, -122.0859797650754);
    EXPECT_EQ(geo_ref->proj_, "utm");
    EXPECT_EQ(geo_ref->k_0_, 1);
    EXPECT_EQ(geo_ref->x_0_, 0);
    EXPECT_EQ(geo_ref->y_0_, 0);
    EXPECT_EQ(geo_ref->datum_, "WGS84");
    EXPECT_EQ(geo_ref->geo_id_grids_, "egm96_15.gtx");
    EXPECT_EQ(geo_ref->vunits_, "m");
    EXPECT_EQ(geo_ref->units_, "m");
    EXPECT_EQ(geo_ref->ellps_, "GRS80");
    EXPECT_EQ(geo_ref->zone_, 32);

    std::string geo_ref_str = odr->GetGeoReferenceAsString();
    EXPECT_EQ(geo_ref_str, "+proj=utm +lat_0=37.3542934123933 +lon_0=-122.0859797650754");
}

TEST(ProbeTest, TestProbeSimpleRoad)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/curve_r100.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    roadmanager::RoadProbeInfo probe_data;
    Position                   pos_pivot = Position(0, -1, 5.0, 0.0);

    pos_pivot.GetProbeInfo(5.0, &probe_data, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER);
    EXPECT_EQ(probe_data.road_lane_info.roadId, 0);
    EXPECT_EQ(probe_data.road_lane_info.laneId, -1);
    EXPECT_DOUBLE_EQ(probe_data.road_lane_info.heading, 0.0);

    pos_pivot.GetProbeInfo(695.0, &probe_data, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER);
    EXPECT_EQ(probe_data.road_lane_info.roadId, 0);
    EXPECT_EQ(probe_data.road_lane_info.laneId, -1);
    EXPECT_DOUBLE_EQ(probe_data.road_lane_info.s, 700.0);
    EXPECT_NEAR(probe_data.relative_h, 0.23758, 1E-5);
    EXPECT_NEAR(probe_data.relative_pos[0], 596.53500, 1E-5);
    EXPECT_NEAR(probe_data.relative_pos[1], 144.45537, 1E-5);

    pos_pivot.GetProbeInfo(695.0, &probe_data, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_ROAD_CENTER);
    EXPECT_EQ(probe_data.road_lane_info.roadId, 0);
    EXPECT_EQ(probe_data.road_lane_info.laneId, -1);
    EXPECT_NEAR(probe_data.relative_h, 0.2381740, 1E-5);
    EXPECT_NEAR(probe_data.relative_pos[0], 595.00000, 1E-5);
    EXPECT_NEAR(probe_data.relative_pos[1], 144.45537, 1E-5);

    // Turn around - position on right side
    pos_pivot.SetLanePos(0, 1, 5.0, 0.0);
    pos_pivot.SetHeadingRelative(M_PI);
    pos_pivot.GetProbeInfo(4.0, &probe_data, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_CURRENT_LATERAL_OFFSET);
    EXPECT_EQ(probe_data.road_lane_info.roadId, 0);
    EXPECT_EQ(probe_data.road_lane_info.laneId, 1);
    EXPECT_DOUBLE_EQ(probe_data.road_lane_info.s, 1.0);
    EXPECT_DOUBLE_EQ(probe_data.road_lane_info.heading, 0.0);
    EXPECT_NEAR(probe_data.relative_h, 0.0, 1E-5);
    EXPECT_NEAR(probe_data.relative_pos[0], 4.0, 1E-5);
    EXPECT_NEAR(probe_data.relative_pos[1], 0.0, 1E-5);

    // Exact at start of road
    pos_pivot.GetProbeInfo(5.0, &probe_data, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_CURRENT_LATERAL_OFFSET);
    EXPECT_EQ(probe_data.road_lane_info.roadId, 0);
    EXPECT_EQ(probe_data.road_lane_info.laneId, 1);
    EXPECT_DOUBLE_EQ(probe_data.road_lane_info.s, 0.0);
    EXPECT_DOUBLE_EQ(probe_data.road_lane_info.heading, 0.0);
    EXPECT_NEAR(probe_data.relative_h, 0.0, 1E-5);
    EXPECT_NEAR(probe_data.relative_pos[0], 5.0, 1E-5);
    EXPECT_NEAR(probe_data.relative_pos[1], 0.0, 1E-5);

    // Go beyond start of road
    pos_pivot.GetProbeInfo(6.0, &probe_data, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_CURRENT_LATERAL_OFFSET);
    EXPECT_EQ(probe_data.road_lane_info.roadId, 0);
    EXPECT_EQ(probe_data.road_lane_info.laneId, 1);
    EXPECT_DOUBLE_EQ(probe_data.road_lane_info.s, 0.0);
    EXPECT_NEAR(probe_data.relative_h, 0.0, 1E-5);
    EXPECT_NEAR(probe_data.relative_pos[0], 5.0, 1E-5);
    EXPECT_NEAR(probe_data.relative_pos[1], 0.0, 1E-5);
}

TEST(ProbeTest, TestProbeComplexRoad)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/fabriksgatan.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 16);

    roadmanager::RoadProbeInfo probe_data;

    // Position on left side, looking beyond road start point.
    Position pos_pivot = Position(3, 1, 5.0, 0.0);
    pos_pivot.SetHeadingRelative(M_PI);

    EXPECT_EQ(pos_pivot.GetProbeInfo(20.0, &probe_data, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER),
              Position::ReturnCode::ERROR_END_OF_ROAD);
    EXPECT_EQ(probe_data.road_lane_info.roadId, 3);
    EXPECT_EQ(probe_data.road_lane_info.laneId, 1);
    EXPECT_NEAR(probe_data.road_lane_info.heading, GetAngleSum(pos_pivot.GetH(), M_PI), 1E-5);
    EXPECT_DOUBLE_EQ(probe_data.road_lane_info.s, 0.0);

    // Position on right side, looking through the intersection
    pos_pivot.SetLanePos(3, -1, 5.0, 0.0);
    pos_pivot.SetHeadingRelative(0.0);
    EXPECT_EQ(pos_pivot.GetProbeInfo(130.0, &probe_data, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER),
              Position::ReturnCode::ENTERED_NEW_ROAD);
    EXPECT_EQ(probe_data.road_lane_info.roadId, 1);
    EXPECT_EQ(probe_data.road_lane_info.laneId, -1);
    EXPECT_NEAR(probe_data.road_lane_info.heading, 0.192980, 1E-5);
    EXPECT_NEAR(probe_data.road_lane_info.s, 5.23650, 1E-5);
}

TEST(DeltaTest, TestDelta)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/fabriksgatan.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 16);

    roadmanager::PositionDiff pos_diff;

    // Position on right side of road 0, looking through intersection into road 2
    Position pos_pivot = Position(0, 1, 5.0, 0.0);
    pos_pivot.SetHeadingRelative(M_PI);
    Position pos_target = Position(2, 1, 250.0, 0.0);
    pos_target.SetHeadingRelative(M_PI);
    EXPECT_EQ(pos_pivot.Delta(&pos_target, pos_diff), true);
    EXPECT_NEAR(pos_diff.ds, 74.56580, 1E-5);
    EXPECT_EQ(pos_diff.dLaneId, 0);
    EXPECT_EQ(pos_diff.dOppLane, false);

    pos_target.SetLanePos(2, -1, 250.0, 0.0);
    pos_target.SetHeadingRelative(M_PI);
    EXPECT_EQ(pos_pivot.Delta(&pos_target, pos_diff), true);
    EXPECT_NEAR(pos_diff.ds, 74.56580, 1E-5);
    EXPECT_EQ(pos_diff.dLaneId, -1);
    EXPECT_EQ(pos_diff.dOppLane, true);

    pos_target.SetLanePos(3, -1, 100.0, 0.0);
    pos_target.SetHeadingRelative(0.0);
    EXPECT_EQ(pos_pivot.Delta(&pos_target, pos_diff), true);
    EXPECT_NEAR(pos_diff.ds, 34.31779, 1E-5);
    EXPECT_NEAR(pos_diff.dt, -3.5, 1E-5);
    EXPECT_EQ(pos_diff.dLaneId, -1);
    EXPECT_EQ(pos_diff.dOppLane, true);

    // Now try diff two positions that are not connected
    pos_pivot.SetLanePos(11, -1, 1.0, 0.0);
    pos_pivot.SetHeadingRelative(0.0);
    pos_target.SetLanePos(6, -1, 1.0, 0.0);
    pos_target.SetHeadingRelative(0.0);
    EXPECT_EQ(pos_pivot.Delta(&pos_target, pos_diff), false);
}

TEST(PositionTest, TestJunctionId)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/fabriksgatan.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 16);

    Position pos;

    // Test some non junction locations - junction ID expected to be -1
    pos.SetLanePos(0, 1, 5.0, 0.0);
    EXPECT_EQ(pos.GetJunctionId(), -1);
    EXPECT_EQ(pos.IsInJunction(), false);
    pos.SetLanePos(1, -1, 5.0, 0.0);
    EXPECT_EQ(pos.GetJunctionId(), -1);
    EXPECT_EQ(pos.IsInJunction(), false);
    pos.SetLanePos(2, -1, 5.0, 0.0);
    EXPECT_EQ(pos.GetJunctionId(), -1);
    EXPECT_EQ(pos.IsInJunction(), false);
    pos.SetLanePos(3, 1, 5.0, 0.0);
    EXPECT_EQ(pos.GetJunctionId(), -1);
    EXPECT_EQ(pos.IsInJunction(), false);

    // Test some connecting road locations - junction ID expected to be 4
    pos.SetLanePos(6, -1, 1.0, 0.0);
    EXPECT_EQ(pos.GetJunctionId(), 4);
    EXPECT_EQ(pos.IsInJunction(), true);
    pos.SetLanePos(9, -1, 1.0, 0.0);
    EXPECT_EQ(pos.GetJunctionId(), 4);
    EXPECT_EQ(pos.IsInJunction(), true);
    pos.SetLanePos(13, -1, 1.0, 0.0);
    EXPECT_EQ(pos.GetJunctionId(), 4);
    EXPECT_EQ(pos.IsInJunction(), true);
    pos.SetLanePos(16, -1, 1.0, 0.0);
    EXPECT_EQ(pos.GetJunctionId(), 4);
    EXPECT_EQ(pos.IsInJunction(), true);
}

TEST(ControllerTest, TestControllers)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/multi_intersections.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 63);

    EXPECT_EQ(odr->GetNumberOfControllers(), 23);

    // check a few samples
    roadmanager::Controller *controller;

    controller = odr->GetControllerByIdx(0);
    EXPECT_EQ(controller->GetName(), "ctrl001");
    int signalIds1[] = {294, 295, 287, 288};
    for (int i = 0; i < controller->GetNumberOfControls(); i++)
    {
        EXPECT_EQ(controller->GetControl(i)->signalId_, signalIds1[i]);
    }

    controller = odr->GetControllerByIdx(22);
    EXPECT_EQ(controller->GetName(), "ctrl027");
    int signalIds2[] = {33617, 33618};
    for (int i = 0; i < controller->GetNumberOfControls(); i++)
    {
        EXPECT_EQ(controller->GetControl(i)->signalId_, signalIds2[i]);
    }

    JunctionController *jcontroller;
    Junction           *junction = odr->GetJunctionByIdx(1);
    EXPECT_EQ(junction->GetNumberOfControllers(), 5);
    int controllerIds[] = {7, 9, 10, 8, 6};
    for (int i = 0; i < junction->GetNumberOfControllers(); i++)
    {
        jcontroller = junction->GetJunctionControllerByIdx(i);
        EXPECT_EQ(jcontroller->id_, controllerIds[i]);
    }

    EXPECT_EQ(junction->GetJunctionControllerByIdx(2)->id_, 10);
    EXPECT_EQ(odr->GetControllerById(junction->GetJunctionControllerByIdx(2)->id_)->GetName(), "ctrl010");
    EXPECT_EQ(odr->GetControllerById(junction->GetJunctionControllerByIdx(2)->id_)->GetControl(1)->signalId_, 3318);
}

TEST(EdgeCaseTest, TestSTruncation)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/fabriksgatan.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 16);

    Road *road = odr->GetRoadById(1);
    EXPECT_NE(road, nullptr);

    Position     pos;
    double       s           = 16.0;
    LaneSection *laneSection = nullptr;

    // inside road length
    EXPECT_EQ(road->GetLaneSectionIdxByS(s), 0);
    laneSection = road->GetLaneSectionByS(s);
    EXPECT_NE(laneSection, nullptr);
    EXPECT_EQ(laneSection->GetLaneByIdx(0)->GetId(), 3);
    pos.SetLanePos(1, -1, s, 0.0);
    EXPECT_NEAR(pos.GetS(), s, 1e-4);
    EXPECT_NEAR(pos.GetX(), 49.17789, 1e-4);
    EXPECT_NEAR(pos.GetY(), 0.100734, 1e-4);

    s = 17.0;
    // outside road length
    EXPECT_EQ(road->GetLaneSectionIdxByS(s), 0);
    laneSection = road->GetLaneSectionByS(s);
    EXPECT_NE(laneSection, nullptr);
    EXPECT_EQ(laneSection->GetLaneByIdx(0)->GetId(), 3);
    pos.SetLanePos(1, -1, s, 0.0);
    EXPECT_NEAR(pos.GetS(), 16.90918, 1e-4);  // truncated
    EXPECT_NEAR(pos.GetS(), road->GetLength(), 1e-4);
    EXPECT_NEAR(pos.GetX(), 50.0702, 1e-4);
    EXPECT_NEAR(pos.GetY(), 0.2751, 1e-4);
}

TEST(LaneInfoTest, TestLaneWidthAndType)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../EnvironmentSimulator/Unittest/xodr/highway_exit.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 5);

    Road *road = odr->GetRoadById(0);
    EXPECT_NE(road, nullptr);

    EXPECT_NEAR(road->GetLaneWidthByS(80, -2), 3.000, 1e-3);
    EXPECT_EQ(road->GetLaneTypeByS(80, 0), 1);
    EXPECT_EQ(road->GetLaneTypeByS(80, -2), 2);

    EXPECT_NEAR(road->GetLaneWidthByS(100, -3), 0.000, 1e-3);
    EXPECT_NEAR(road->GetLaneWidthByS(105, -3), 0.084, 1e-3);
    EXPECT_NEAR(road->GetLaneWidthByS(140, -3), 2.688, 1e-3);
    EXPECT_NEAR(road->GetLaneWidthByS(150, -3), 3.000, 1e-3);
}

TEST(LaneInfoTest, TestDetailedLaneType)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../EnvironmentSimulator/Unittest/xodr/mw_100m.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    Road *road = odr->GetRoadById(1);
    EXPECT_NE(road, nullptr);

    Position pos(1, -3, 100.0, 0.0);
    pos.SetSnapLaneTypes(Lane::LaneType::LANE_TYPE_ANY);
    EXPECT_EQ(pos.GetInLaneType(), Lane::LaneType::LANE_TYPE_DRIVING);

    pos.SetInertiaPos(34.35, -13.40, 0.0);
    EXPECT_EQ(pos.GetLaneId(), -5);
    int lane_type = pos.GetInLaneType();
    EXPECT_EQ(lane_type, Lane::LaneType::LANE_TYPE_RESTRICTED);
    EXPECT_EQ(lane_type & Lane::LaneType::LANE_TYPE_ANY_DRIVING, 0);
    EXPECT_NE(lane_type & Lane::LaneType::LANE_TYPE_ANY_ROAD, 0);

    pos.SetInertiaPos(43.17, -14.81, 0.0);
    EXPECT_EQ(pos.GetLaneId(), -6);
    lane_type = pos.GetInLaneType();
    EXPECT_EQ(lane_type, Lane::LaneType::LANE_TYPE_STOP);
    EXPECT_EQ(lane_type & Lane::LaneType::LANE_TYPE_ANY_DRIVING, 0);
    EXPECT_NE(lane_type & Lane::LaneType::LANE_TYPE_ANY_ROAD, 0);

    pos.SetInertiaPos(49.65, -16.95, 0.0);
    EXPECT_EQ(pos.GetLaneId(), -7);
    lane_type = pos.GetInLaneType();
    EXPECT_EQ(lane_type, Lane::LaneType::LANE_TYPE_BORDER);
    EXPECT_EQ(lane_type & Lane::LaneType::LANE_TYPE_ANY_DRIVING, 0);
    EXPECT_EQ(lane_type & Lane::LaneType::LANE_TYPE_ANY_ROAD, 0);

    pos.SetInertiaPos(60.24, -20.29, 0.0);
    EXPECT_EQ(pos.GetLaneId(), -8);
    lane_type = pos.GetInLaneType();
    EXPECT_EQ(lane_type, Lane::LaneType::LANE_TYPE_BORDER);
    EXPECT_EQ(lane_type & Lane::LaneType::LANE_TYPE_ANY_DRIVING, 0);
    EXPECT_EQ(lane_type & Lane::LaneType::LANE_TYPE_ANY_ROAD, 0);

    pos.SetInertiaPos(74.24, -24.69, 0.0);
    EXPECT_EQ(pos.GetLaneId(), -8);  // not in it, but it's the outermost and closest lane
    lane_type = pos.GetInLaneType();
    EXPECT_EQ(lane_type, Lane::LaneType::LANE_TYPE_NONE);
    EXPECT_EQ(lane_type & Lane::LaneType::LANE_TYPE_ANY_DRIVING, 0);
    EXPECT_EQ(lane_type & Lane::LaneType::LANE_TYPE_ANY_ROAD, 0);
}

TEST(RoadInfoTest, TestGetNrRoadsOverlappingPos)
{
    int road_id_0[] = {5, 9, 10, 12, 15};
    int road_id_1[] = {16, 7, 10};
    int road_id_2[] = {2};
    int n           = 0;

    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/fabriksgatan.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 16);

    Position pos;

    pos.SetLanePos(5, -1, 7.2, -0.2);
    n = pos.GetNumberOfRoadsOverlapping();
    EXPECT_EQ(n, sizeof(road_id_0) / sizeof(int));
    for (int i = 0; i < n; i++)
    {
        EXPECT_EQ(pos.GetOverlappingRoadId(i), road_id_0[i]);
    }

    pos.SetLanePos(16, -1, 9.0, -0.2);
    n = pos.GetNumberOfRoadsOverlapping();
    EXPECT_EQ(n, sizeof(road_id_1) / sizeof(int));
    for (int i = 0; i < n; i++)
    {
        EXPECT_EQ(pos.GetOverlappingRoadId(i), road_id_1[i]);
    }

    pos.SetLanePos(2, 1, 50.0, 0.5);
    n = pos.GetNumberOfRoadsOverlapping();
    EXPECT_EQ(n, sizeof(road_id_2) / sizeof(int));
    for (int i = 0; i < n; i++)
    {
        EXPECT_EQ(pos.GetOverlappingRoadId(i), road_id_2[i]);
    }

    pos.SetLanePos(2, 3, 50.0, 0.95);
    n = pos.GetNumberOfRoadsOverlapping();
    EXPECT_EQ(n, sizeof(road_id_2) / sizeof(int));
    for (int i = 0; i < n; i++)
    {
        EXPECT_EQ(pos.GetOverlappingRoadId(i), road_id_2[i]);
    }

    double right_side_width = 5.8;
    pos.SetTrackPos(2, 50.0, right_side_width - 0.05);
    n = pos.GetNumberOfRoadsOverlapping();
    EXPECT_EQ(n, sizeof(road_id_2) / sizeof(int));
    for (int i = 0; i < n; i++)
    {
        EXPECT_EQ(pos.GetOverlappingRoadId(i), road_id_2[i]);
    }

    pos.SetTrackPos(2, 55.0, right_side_width + 0.05);
    EXPECT_EQ(pos.GetNumberOfRoadsOverlapping(), 0);

    double lane_width = 2.0;
    pos.SetLanePos(2, 3, 50.0, lane_width / 2 + 0.05);
    EXPECT_EQ(pos.GetNumberOfRoadsOverlapping(), 0);
}

TEST(RoadPosTest, TestPrioStraightRoadInJunction)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/fabriksgatan.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 16);

    Position pos;
    pos.SetMode(Position::PosModeType::UPDATE,

                roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
                    roadmanager::Position::PosMode::R_REL);
    pos.SetLanePos(0, 1, 0, 0.0);
    pos.SetHeadingRelative(3.1415);
    EXPECT_NEAR(pos.GetX(), 28.956, 1E-3);
    EXPECT_NEAR(pos.GetY(), -9.821, 1E-3);
    EXPECT_NEAR(pos.GetH(), 1.783, 1E-3);

    pos.SetInertiaPos(28.55, -7.96, 1.78);
    EXPECT_EQ(pos.GetTrackId(), 9);
    EXPECT_EQ(pos.GetLaneId(), -1);
    EXPECT_NEAR(pos.GetOffset(), 0.009, 1E-3);
    EXPECT_NEAR(pos.GetH(), 1.780, 1E-3);

    pos.SetLanePos(0, 1, 1.0, 0.0);
    pos.SetHeadingRelative(3.1415);
    EXPECT_EQ(pos.MoveAlongS(0.5, 0.0, 0.0, true, roadmanager::Position::MoveDirectionMode::HEADING_DIRECTION, true),
              roadmanager::Position::ReturnCode::OK);
    EXPECT_EQ(pos.GetTrackId(), 0);
    EXPECT_EQ(pos.MoveAlongS(1.0, 0.0, 0.0, true, roadmanager::Position::MoveDirectionMode::HEADING_DIRECTION, true),
              roadmanager::Position::ReturnCode::MADE_JUNCTION_CHOICE);
    EXPECT_EQ(pos.GetTrackId(), 9);
    EXPECT_EQ(pos.MoveAlongS(0.1, 0.0, 0.0, true, roadmanager::Position::MoveDirectionMode::HEADING_DIRECTION, true),
              roadmanager::Position::ReturnCode::OK);
    EXPECT_EQ(pos.GetTrackId(), 9);
}

class StarRoadTestFixture : public testing::Test
{
public:
    StarRoadTestFixture();
    void Check(double a, double b, double c, double d, double e);

protected:
    OpenDrive *odr;
    Position   pos;
    double     lane_width;
};

StarRoadTestFixture::StarRoadTestFixture() : lane_width(3.5)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../EnvironmentSimulator/Unittest/xodr/star.xodr");
}

void StarRoadTestFixture::Check(double x, double y, double h, double p_road, double p)
{
    EXPECT_NEAR(pos.GetX(), x, 1E-3);
    EXPECT_NEAR(pos.GetY(), y, 1E-3);
    EXPECT_NEAR(pos.GetH(), h, 1E-3);
    EXPECT_NEAR(pos.GetPRoad(), p_road, 1E-3);
    EXPECT_NEAR(pos.GetP(), p, 1E-3);
    EXPECT_NEAR(fmod(pos.GetR(), 2.0 * M_PI), 0.0, 1E-3);
}

TEST_F(StarRoadTestFixture, TestRelativeRoadPos)
{
    odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 24);

    // Road heading pi/2 downhill - RHT
    // right side
    pos.SetMode(Position::PosModeType::SET, Position::PosMode::H_REL);
    pos.SetHeadingRelative(0.0);
    pos.SetTrackPos(7, 10.0, -lane_width / 2.0);
    Check(lane_width / 2.0, 20.0, M_PI / 2, 0.55, 0.55);
    pos.SetLanePos(7, -1, 10.0, 0.0);
    Check(lane_width / 2.0, 20.0, M_PI / 2, 0.55, 0.55);
    // left side (facing uphill)
    pos.SetHeadingRelative(M_PI);
    pos.SetTrackPos(7, 10.0, lane_width / 2.0);
    Check(-lane_width / 2.0, 20.0, 3 * M_PI / 2, 0.55, 2 * M_PI - 0.55);
    pos.SetLanePos(7, 1, 10.0, 0.0);
    Check(-lane_width / 2.0, 20.0, 3 * M_PI / 2, 0.55, 2 * M_PI - 0.55);

    // Road heading pi/2 downhill - LHT
    // right side
    pos.SetHeadingRelative(M_PI);
    pos.SetTrackPos(7, 10.0, -lane_width / 2.0);
    Check(lane_width / 2.0, 20.0, 3 * M_PI / 2, 0.55, 2 * M_PI - 0.55);
    // left side (facing uphill)
    pos.SetHeadingRelative(0.0);
    pos.SetTrackPos(7, 10.0, lane_width / 2.0);
    Check(-lane_width / 2.0, 20.0, M_PI / 2, 0.55, 0.55);

    // Road heading 3pi/2 uphill - LHT
    // right side
    pos.SetHeadingRelative(0.0);
    pos.SetTrackPos(20, 10.0, -lane_width / 2.0);
    Check(-lane_width / 2.0, -20.0, 3 * M_PI / 2, -0.55, 2 * M_PI - 0.55);
    // left side (facing uphill)
    pos.SetHeadingRelative(M_PI);
    pos.SetTrackPos(20, 10.0, lane_width / 2.0);
    Check(lane_width / 2.0, -20.0, M_PI / 2, -0.55, 0.55);
}

TEST(OSIPointTest, MixedRoads)
{
    ASSERT_EQ(roadmanager::Position::LoadOpenDrive("../../../EnvironmentSimulator/Unittest/xodr/mixed_roads.xodr"), true);
    roadmanager::OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    EXPECT_EQ(odr->GetNumOfRoads(), 4);

    Road *road = odr->GetRoadByIdx(0);
    EXPECT_EQ(road->GetId(), 0);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(0).x, 0.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(0).y, 1.5, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(0).z, 0.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(4).x, 18.119, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(4).y, 3.181, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(4).z, 0.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(10).x, 40.423, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(10).y, 11.275, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(10).z, 0.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(12).x, 46.744, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(12).y, 14.436, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(12).z, 0.0, 1e-3);

    road = odr->GetRoadByIdx(1);
    EXPECT_EQ(road->GetId(), 1);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(0).x, 0.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(0).y, -8.5, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(0).z, 0.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(1).x, 19.875, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(1).y, -8.5, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(1).z, 0.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(2).x, 20.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(2).y, -8.5, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(2).z, 2.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(3).x, 50.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(3).y, -8.5, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(3).z, 2.0, 1e-3);

    road = odr->GetRoadByIdx(2);
    EXPECT_EQ(road->GetId(), 2);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(0).x, 0.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(0).y, -18.5, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(0).z, 0.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(4).x, 20.116, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(4).y, -19.507, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(4).z, 2.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(7).x, 39.167, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(7).y, -22.343, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(7).z, 2.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(9).x, 49.852, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(9).y, -24.764, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(0)->GetOSIPoints()->GetPoint(9).z, 2.0, 1e-3);

    road = odr->GetRoadByIdx(3);
    EXPECT_EQ(road->GetId(), 3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(2)->GetOSIPoints()->GetPoint(0).x, 0.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(2)->GetOSIPoints()->GetPoint(0).y, -41.5, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(2)->GetOSIPoints()->GetPoint(0).z, 0.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(2)->GetOSIPoints()->GetPoint(4).x, 56.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(2)->GetOSIPoints()->GetPoint(4).y, -41.5, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(2)->GetOSIPoints()->GetPoint(4).z, 0.397, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(2)->GetOSIPoints()->GetPoint(9).x, 69.875, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(2)->GetOSIPoints()->GetPoint(9).y, -41.5, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(2)->GetOSIPoints()->GetPoint(9).z, 3.484, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(2)->GetOSIPoints()->GetPoint(21).x, 150.0, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(2)->GetOSIPoints()->GetPoint(21).y, -41.5, 1e-3);
    EXPECT_NEAR(road->GetLaneSectionByIdx(0)->GetLaneByIdx(2)->GetOSIPoints()->GetPoint(21).z, 10.0, 1e-3);

    odr->Clear();
}

TEST(RoadEdgeTest, TestRoadEdge)
{
    ASSERT_EQ(roadmanager::Position::LoadOpenDrive("../../../EnvironmentSimulator/Unittest/xodr/highway_example_with_merge_and_split.xodr"), true);
    roadmanager::OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    EXPECT_EQ(odr->GetNumOfRoads(), 9);

    Road *road = odr->GetRoadById(0);
    EXPECT_EQ(road->GetId(), 0);
    LaneSection *lane_section = road->GetLaneSectionByIdx(2);
    EXPECT_EQ(lane_section->GetLaneById(2)->IsRoadEdge(), true);
    EXPECT_EQ(lane_section->GetLaneById(1)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(0)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(-1)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(-2)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(-3)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(-4)->IsRoadEdge(), true);

    road = odr->GetRoadById(3);
    EXPECT_EQ(road->GetId(), 3);
    lane_section = road->GetLaneSectionByIdx(0);
    EXPECT_EQ(lane_section->GetLaneById(2)->IsRoadEdge(), true);
    EXPECT_EQ(lane_section->GetLaneById(1)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(0)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(-1)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(-2)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(-3)->IsRoadEdge(), true);

    road = odr->GetRoadById(4);
    EXPECT_EQ(road->GetId(), 4);
    lane_section = road->GetLaneSectionByIdx(0);
    EXPECT_EQ(lane_section->GetLaneById(-1)->IsRoadEdge(), true);
    EXPECT_EQ(lane_section->GetLaneById(0)->IsRoadEdge(), true);

    road = odr->GetRoadById(1);
    EXPECT_EQ(road->GetId(), 1);
    lane_section = road->GetLaneSectionByIdx(1);
    EXPECT_EQ(lane_section->GetLaneById(3)->IsRoadEdge(), true);
    EXPECT_EQ(lane_section->GetLaneById(2)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(1)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(0)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(-1)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(-2)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(-3)->IsRoadEdge(), true);

    odr->Clear();

    ASSERT_EQ(roadmanager::Position::LoadOpenDrive("../../../resources/xodr/jolengatan.xodr"), true);
    odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    road = odr->GetRoadById(1);
    EXPECT_EQ(road->GetId(), 1);
    lane_section = road->GetLaneSectionByIdx(0);
    EXPECT_EQ(lane_section->GetLaneById(3)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(2)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(1)->IsRoadEdge(), true);
    EXPECT_EQ(lane_section->GetLaneById(0)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(-1)->IsRoadEdge(), true);
    EXPECT_EQ(lane_section->GetLaneById(-2)->IsRoadEdge(), false);
    EXPECT_EQ(lane_section->GetLaneById(-3)->IsRoadEdge(), false);

    odr->Clear();
}

TEST(ExplicitLineTest, TestExplicitRoadMark)
{
    ASSERT_EQ(roadmanager::Position::LoadOpenDrive("../../../EnvironmentSimulator/Unittest/xodr/explicit_line.xodr"), true);
    roadmanager::OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    EXPECT_EQ(odr->GetNumOfRoads(), 1);
    Road *road = odr->GetRoadById(1);
    EXPECT_EQ(road->GetNumberOfLaneSections(), 2);

    LaneSection  *lane_section = road->GetLaneSectionByIdx(0);
    Lane         *lane         = lane_section->GetLaneById(0);
    LaneRoadMark *roadmark     = lane->GetLaneRoadMarkByIdx(0);
    EXPECT_EQ(roadmark->GetNumberOfRoadMarkExplicit(), 0);

    lane_section = road->GetLaneSectionByIdx(1);

    // Check centerlane (double line)
    lane     = lane_section->GetLaneById(0);
    roadmark = lane->GetLaneRoadMarkByIdx(0);
    EXPECT_EQ(roadmark->GetNumberOfRoadMarkExplicit(), 1);
    LaneRoadMarkExplicit *lane_road_mark_explicit = roadmark->GetLaneRoadMarkExplicitByIdx(0);

    EXPECT_EQ(lane_road_mark_explicit->GetNumberOfLaneRoadMarkExplicitLines(), 2);
    LaneRoadMarkExplicitLine *line = lane_road_mark_explicit->GetLaneRoadMarkExplicitLineByIdx(0);
    EXPECT_NEAR(line->GetLength(), 1.0, 1e-3);
    EXPECT_NEAR(line->GetTOffset(), 0.2, 1e-3);
    EXPECT_NEAR(line->GetWidth(), 0.2, 1e-3);
    EXPECT_NEAR(line->GetSOffset(), 0.0, 1e-3);
    EXPECT_EQ(line->GetOSIPoints()->GetNumOfOSIPoints(), 2);
    EXPECT_NEAR(line->GetOSIPoints()->GetXfromIdx(0), 2, 1e-3);
    EXPECT_NEAR(line->GetOSIPoints()->GetXfromIdx(1), 3, 1e-3);

    line = lane_road_mark_explicit->GetLaneRoadMarkExplicitLineByIdx(1);
    EXPECT_NEAR(line->GetLength(), 1.0, 1e-3);
    EXPECT_NEAR(line->GetTOffset(), -0.2, 1e-3);
    EXPECT_NEAR(line->GetWidth(), 0.2, 1e-3);
    EXPECT_NEAR(line->GetSOffset(), 0.0, 1e-3);
    EXPECT_EQ(line->GetOSIPoints()->GetNumOfOSIPoints(), 2);
    EXPECT_NEAR(line->GetOSIPoints()->GetXfromIdx(0), 2, 1e-3);
    EXPECT_NEAR(line->GetOSIPoints()->GetXfromIdx(1), 3, 1e-3);

    // Check left lane (single line)
    lane     = lane_section->GetLaneById(1);
    roadmark = lane->GetLaneRoadMarkByIdx(0);
    EXPECT_EQ(roadmark->GetNumberOfRoadMarkExplicit(), 1);
    lane_road_mark_explicit = roadmark->GetLaneRoadMarkExplicitByIdx(0);

    EXPECT_EQ(lane_road_mark_explicit->GetNumberOfLaneRoadMarkExplicitLines(), 1);
    line = lane_road_mark_explicit->GetLaneRoadMarkExplicitLineByIdx(0);
    EXPECT_NEAR(line->GetLength(), 1.0, 1e-3);
    EXPECT_NEAR(line->GetTOffset(), 0.0, 1e-3);
    EXPECT_NEAR(line->GetWidth(), 0.15, 1e-3);
    EXPECT_NEAR(line->GetSOffset(), 0.0, 1e-3);

    EXPECT_EQ(line->GetOSIPoints()->GetNumOfOSIPoints(), 2);
    EXPECT_NEAR(line->GetOSIPoints()->GetXfromIdx(0), 2, 1e-3);
    EXPECT_NEAR(line->GetOSIPoints()->GetXfromIdx(1), 3, 1e-3);
    odr->Clear();
}

TEST(PositionModeTest, TestModeBitmasks)
{
    ASSERT_EQ(roadmanager::Position::LoadOpenDrive("../../../EnvironmentSimulator/Unittest/xodr/straight_500_superelevation_elevation_curve.xodr"),
              true);
    roadmanager::OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    Road *road = odr->GetRoadByIdx(0);
    EXPECT_EQ(road->GetId(), 1);

    // Verify default modes
    EXPECT_EQ(Position::GetModeDefault(Position::PosModeType::SET),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);
    EXPECT_EQ(Position::GetModeDefault(Position::PosModeType::UPDATE),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);

    Position pos;
    EXPECT_EQ(pos.GetMode(Position::PosModeType::SET), Position::GetModeDefault(Position::PosModeType::SET));
    EXPECT_EQ(pos.GetMode(Position::PosModeType::UPDATE), Position::GetModeDefault(Position::PosModeType::UPDATE));

    // Test some operations
    pos.SetLanePos(road->GetId(), -1, 140.0, 0.0);
    EXPECT_NEAR(pos.GetH(), 1.4, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.0, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.486, 1e-3);

    pos.SetMode(Position::PosModeType::UPDATE, Position::PosMode::R_REL);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::SET),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::UPDATE),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);
    pos.SetRollRelative(0.1);
    pos.SetLanePos(road->GetId(), -1, 150.0, 0.0);
    EXPECT_NEAR(pos.GetH(), 1.5, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.0, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.6, 1e-3);
    EXPECT_NEAR(pos.GetRRoad(), 0.6 - 0.1, 1e-3);
    pos.SetLanePos(road->GetId(), -1, 140.0, 0.0);
    EXPECT_NEAR(pos.GetH(), 1.4, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.0, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.486 + 0.1, 1e-3);

    pos.SetMode(Position::PosModeType::UPDATE, Position::PosMode::R_ABS);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::SET),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::UPDATE),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_ABS);
    pos.SetRoll(0.1);
    pos.SetLanePos(road->GetId(), -1, 150.0, 0.0);
    EXPECT_NEAR(pos.GetH(), 1.5, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetP(), 0.0), 0.0, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.1, 1e-3);

    pos.SetLanePos(road->GetId(), -1, 140.0, 0.0);
    EXPECT_NEAR(pos.GetH(), 1.4, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetP(), 0.0), 0.0, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.1, 1e-3);

    pos.SetLanePos(road->GetId(), -1, 300.0, 0.0);
    EXPECT_NEAR(pos.GetH(), 3.0, 1e-3);
    EXPECT_NEAR(pos.GetP(), 5.991, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.1, 1e-3);

    pos.SetMode(Position::PosModeType::UPDATE, Position::PosMode::R_REL);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::SET),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::UPDATE),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);
    pos.SetRollRelative(0.0);
    pos.SetLanePos(road->GetId(), -1, 300.0, 0.0);
    EXPECT_NEAR(pos.GetH(), 3.0, 1e-3);
    EXPECT_NEAR(pos.GetP(), 5.991, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetR(), 0.0), 0.0, 1e-3);

    pos.SetInertiaPos(0.0, 200.0, 0.5);
    EXPECT_NEAR(pos.GetH(), 0.5156, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.2356, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.1315, 1e-3);

    pos.SetInertiaPos(-100.0, 83.0, 0.5);
    EXPECT_NEAR(pos.GetH(), 0.5, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetP(), 0.0), 0.0, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetR(), 0.0), 0.0, 1e-3);

    pos.SetModeDefault(Position::PosModeType::SET);
    pos.SetInertiaPos(100.0, 85.0, -10.0, 0.5, 0.0, 0.3);
    EXPECT_NEAR(pos.GetH(), 0.5615, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.3853, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.6127, 1e-3);

    pos.SetMode(Position::PosModeType::SET, Position::PosMode::R_ABS);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::SET),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_ABS);
    pos.SetInertiaPos(100.0, 85.0, -10.0, 0.5, 0.0, 0.3);
    EXPECT_NEAR(pos.GetH(), 0.5, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetP(), 0.0), 0.0, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.3, 1e-3);

    // Test some settings
    pos.SetMode(Position::PosModeType::UPDATE, Position::PosMode::H_REL | Position::PosMode::Z_ABS);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::UPDATE),
              roadmanager::Position::PosMode::Z_ABS | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);

    pos.SetMode(Position::PosModeType::SET, Position::PosMode::H_REL | Position::PosMode::Z_ABS | Position::PosMode::P_ABS);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::SET),
              roadmanager::Position::PosMode::Z_ABS | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_ABS |
                  roadmanager::Position::PosMode::R_ABS);
    pos.SetMode(Position::PosModeType::SET, Position::PosMode::Z_MASK & Position::PosMode::Z_DEFAULT);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::SET),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_ABS |
                  roadmanager::Position::PosMode::R_ABS);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::UPDATE),
              roadmanager::Position::PosMode::Z_ABS | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);

    odr->Clear();
}

TEST(PositionModeTest, TestPositionTypes)
{
    ASSERT_EQ(roadmanager::Position::LoadOpenDrive("../../../EnvironmentSimulator/Unittest/xodr/straight_500_superelevation_elevation_curve.xodr"),
              true);
    roadmanager::OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    Road *road = odr->GetRoadByIdx(0);
    EXPECT_EQ(road->GetId(), 1);

    Position pos0;
    Position pos1;
    // Place car
    pos0.SetLanePos(1, -1, 300.0, 0.0);
    pos1.SetHeadingRelative(M_PI);
    pos1.SetLanePos(1, 1, 300.0, 0.0);

    EXPECT_NEAR(pos0.GetX(), 14.329, 1e-3);
    EXPECT_NEAR(pos0.GetY(), 200.519, 1e-3);
    EXPECT_NEAR(pos0.GetZ(), 10.0, 1e-3);
    EXPECT_NEAR(pos0.GetH(), 3.0, 1e-3);
    EXPECT_NEAR(pos0.GetP(), 5.992, 1e-3);
    EXPECT_NEAR(pos0.GetR(), 0.0, 1e-3);

    EXPECT_NEAR(pos1.GetX(), 13.895, 1e-3);
    EXPECT_NEAR(pos1.GetY(), 197.480, 1e-3);
    EXPECT_NEAR(pos1.GetZ(), 10.0, 1e-3);
    EXPECT_NEAR(pos1.GetH(), 6.142, 1e-3);
    EXPECT_NEAR(pos1.GetP(), 0.291, 1e-3);
    EXPECT_NEAR(pos1.GetR(), 6.283, 1e-3);

    odr->Clear();
}

TEST(LaneId, TestRelativeLaneIdCalculation)
{
    EXPECT_EQ(GetRelativeLaneId(0, 0), 0);
    EXPECT_EQ(GetRelativeLaneId(0, 1), 1);
    EXPECT_EQ(GetRelativeLaneId(0, 5), 5);
    EXPECT_EQ(GetRelativeLaneId(1, 0), 1);
    EXPECT_EQ(GetRelativeLaneId(4, 0), 4);
    EXPECT_EQ(GetRelativeLaneId(-1, 0), -1);
    EXPECT_EQ(GetRelativeLaneId(-3, 0), -3);
    EXPECT_EQ(GetRelativeLaneId(0, -1), -1);
    EXPECT_EQ(GetRelativeLaneId(0, -6), -6);
    EXPECT_EQ(GetRelativeLaneId(-1, -1), -2);
    EXPECT_EQ(GetRelativeLaneId(-5, -5), -10);
    EXPECT_EQ(GetRelativeLaneId(1, 1), 2);
    EXPECT_EQ(GetRelativeLaneId(5, 3), 8);
    EXPECT_EQ(GetRelativeLaneId(-1, 2), 2);
    EXPECT_EQ(GetRelativeLaneId(-3, 6), 4);
    EXPECT_EQ(GetRelativeLaneId(1, -5), -5);
    EXPECT_EQ(GetRelativeLaneId(3, -5), -3);
}

TEST(LaneId, TestLaneIdDeltaCalculation)
{
    EXPECT_EQ(GetLaneIdDelta(0, 0), 0);
    EXPECT_EQ(GetLaneIdDelta(0, 1), 0);
    EXPECT_EQ(GetLaneIdDelta(0, 5), 4);
    EXPECT_EQ(GetLaneIdDelta(1, 0), 0);
    EXPECT_EQ(GetLaneIdDelta(4, 0), -3);
    EXPECT_EQ(GetLaneIdDelta(-1, 0), 0);
    EXPECT_EQ(GetLaneIdDelta(-3, 0), 2);
    EXPECT_EQ(GetLaneIdDelta(0, -1), 0);
    EXPECT_EQ(GetLaneIdDelta(0, -6), -5);
    EXPECT_EQ(GetLaneIdDelta(-1, -1), 0);
    EXPECT_EQ(GetLaneIdDelta(-5, -5), 0);
    EXPECT_EQ(GetLaneIdDelta(-1, -5), -4);
    EXPECT_EQ(GetLaneIdDelta(-2, -5), -3);
    EXPECT_EQ(GetLaneIdDelta(1, 1), 0);
    EXPECT_EQ(GetLaneIdDelta(5, 3), -2);
    EXPECT_EQ(GetLaneIdDelta(-1, 2), 2);
    EXPECT_EQ(GetLaneIdDelta(-3, 6), 8);
    EXPECT_EQ(GetLaneIdDelta(1, -5), -5);
    EXPECT_EQ(GetLaneIdDelta(3, -5), -7);
}

// Check that orientation (pitch) is correctly adjusted when moving along s after absolute orientation has been specified.
// Also after changing side of road and pitch is inverted.
// Verifies that relative orientation are correctly calculated from the absolute values
TEST(RotationTest, TestFindOutRelativeOrientation)
{
    ASSERT_EQ(roadmanager::Position::LoadOpenDrive("../../../EnvironmentSimulator/Unittest/xodr/slope_up_slope_down.xodr"), true);
    roadmanager::OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    Position pos;
    double   start_pos_orig[3] = {25.0, 1.5, 12.5};
    double   start_pos_xform[3];
    RotateVec2D(start_pos_orig[0], start_pos_orig[1], M_PI_4, start_pos_xform[0], start_pos_xform[1]);
    start_pos_xform[2] = start_pos_orig[2];

    pos.SetMode(roadmanager::Position::PosModeType::UPDATE,
                roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_REL |
                    roadmanager::Position::PosMode::R_REL);

    // first put car with all absolute values on right side
    pos.SetInertiaPosMode(start_pos_xform[0],
                          start_pos_xform[1],
                          start_pos_xform[2],
                          M_PI_4,
                          -0.463647609,  // atan(25/50)
                          0.0,
                          roadmanager::Position::PosMode::Z_ABS | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_ABS |
                              roadmanager::Position::PosMode::R_ABS);

    EXPECT_NEAR(pos.GetX(), 16.617, 1e-3);
    EXPECT_NEAR(pos.GetY(), 18.738, 1e-3);
    EXPECT_NEAR(pos.GetZ(), 12.5, 1e-3);
    EXPECT_NEAR(pos.GetH(), 0.785, 1e-3);
    EXPECT_NEAR(pos.GetP(), -0.464, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetR(), 0.0), 0.0, 1e-3);
    // move slightly forward
    pos.MoveAlongS(0.1);
    EXPECT_NEAR(pos.GetX(), 16.688, 1e-3);
    EXPECT_NEAR(pos.GetY(), 18.809, 1e-3);
    EXPECT_NEAR(pos.GetZ(), 12.550, 1e-3);
    EXPECT_NEAR(pos.GetH(), 0.785, 1e-3);
    EXPECT_NEAR(pos.GetP(), 5.820, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetR(), 0.0), 0.0, 1e-3);

    // then move car to other side, turned around.
    pos.SetInertiaPosMode(start_pos_xform[0],
                          start_pos_xform[1],
                          start_pos_xform[2],
                          M_PI + M_PI_4,
                          0.463647609,  // atan(25/50)
                          0.0,
                          roadmanager::Position::PosMode::Z_ABS | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_ABS |
                              roadmanager::Position::PosMode::R_ABS);

    EXPECT_NEAR(pos.GetX(), 16.617, 1e-3);
    EXPECT_NEAR(pos.GetY(), 18.738, 1e-3);
    EXPECT_NEAR(pos.GetZ(), 12.5, 1e-3);
    EXPECT_NEAR(pos.GetH(), 3.927, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.464, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetR(), 0.0), 0.0, 1e-3);
    // move slightly forward
    pos.MoveAlongS(0.1);
    EXPECT_NEAR(pos.GetX(), 16.546, 1e-3);
    EXPECT_NEAR(pos.GetY(), 18.668, 1e-3);
    EXPECT_NEAR(pos.GetZ(), 12.450, 1e-3);
    EXPECT_NEAR(pos.GetH(), 3.927, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.464, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetR(), 0.0), 0.0, 1e-3);

    // Same as above, but now with relative roll
    pos.SetInertiaPosMode(start_pos_xform[0],
                          start_pos_xform[1],
                          start_pos_xform[2],
                          M_PI + M_PI_4,
                          0.463647609,  // atan(25/50)
                          0.0,
                          roadmanager::Position::PosMode::Z_ABS | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_ABS |
                              roadmanager::Position::PosMode::R_REL);

    EXPECT_NEAR(pos.GetX(), 16.617, 1e-3);
    EXPECT_NEAR(pos.GetY(), 18.738, 1e-3);
    EXPECT_NEAR(pos.GetZ(), 12.5, 1e-3);
    EXPECT_NEAR(pos.GetH(), 3.927, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.464, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetR(), 0.0), 0.0, 1e-3);
    pos.MoveAlongS(0.1);
    EXPECT_NEAR(pos.GetX(), 16.546, 1e-3);
    EXPECT_NEAR(pos.GetY(), 18.668, 1e-3);
    EXPECT_NEAR(pos.GetZ(), 12.450, 1e-3);
    EXPECT_NEAR(pos.GetH(), 3.927, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.464, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetR(), 0.0), 0.0, 1e-3);

    // relative pitch
    pos.SetInertiaPosMode(start_pos_xform[0],
                          start_pos_xform[1],
                          start_pos_xform[2],
                          M_PI + M_PI_4,
                          0.463647609,  // atan(25/50)
                          0.0,
                          roadmanager::Position::PosMode::Z_ABS | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
                              roadmanager::Position::PosMode::R_REL);

    EXPECT_NEAR(pos.GetX(), 16.617, 1e-3);
    EXPECT_NEAR(pos.GetY(), 18.738, 1e-3);
    EXPECT_NEAR(pos.GetZ(), 12.5, 1e-3);
    EXPECT_NEAR(pos.GetH(), 3.927, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.927, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetR(), 0.0), 0.0, 1e-3);
    pos.MoveAlongS(0.1);
    EXPECT_NEAR(pos.GetX(), 16.546, 1e-3);
    EXPECT_NEAR(pos.GetY(), 18.668, 1e-3);
    EXPECT_NEAR(pos.GetZ(), 12.450, 1e-3);
    EXPECT_NEAR(pos.GetH(), 3.927, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.927, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetR(), 0.0), 0.0, 1e-3);
}

TEST(LaneMaterialTest, TestGlobalFriction)
{
    ASSERT_EQ(roadmanager::Position::LoadOpenDrive("../../../EnvironmentSimulator/Unittest/xodr/straight_road.xodr"), true);
    roadmanager::OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    EXPECT_EQ(odr->GetNumOfRoads(), 1);
    EXPECT_EQ(odr->GetFriction(), 0.5);  // multiple friction values

    ASSERT_EQ(roadmanager::Position::LoadOpenDrive("../../../EnvironmentSimulator/Unittest/xodr/straight_2x100m_opposite.xodr"), true);
    odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    EXPECT_EQ(odr->GetNumOfRoads(), 2);
    EXPECT_EQ(odr->GetFriction(), FRICTION_DEFAULT);  // no friction values, fall back to default
}

TEST(LaneMaterialTest, TestLaneFriction)
{
    ASSERT_EQ(roadmanager::Position::LoadOpenDrive("../../../EnvironmentSimulator/Unittest/xodr/straight_highway_500m.xodr"), true);
    roadmanager::OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    EXPECT_EQ(std::isnan(odr->GetFriction()), true);  // multiple friction values

    Road *road = odr->GetRoadById(0);
    EXPECT_EQ(road->GetId(), 0);
    LaneSection *lane_section = road->GetLaneSectionByIdx(0);
    EXPECT_EQ(lane_section->GetNumberOfLanes(), 7);

    Lane *lane = lane_section->GetLaneById(2);
    EXPECT_EQ(lane->GetNumberOfMaterials(), 3);
    EXPECT_NEAR(lane->GetMaterialByIdx(0)->s_offset, 0.0, 1e-3);
    EXPECT_NEAR(lane->GetMaterialByIdx(0)->friction, 1.0, 1e-3);
    EXPECT_NEAR(lane->GetMaterialByIdx(1)->s_offset, 100.0, 1e-3);
    EXPECT_NEAR(lane->GetMaterialByIdx(1)->friction, 0.4, 1e-3);
    EXPECT_NEAR(lane->GetMaterialByIdx(2)->s_offset, 120.0, 1e-3);
    EXPECT_NEAR(lane->GetMaterialByIdx(2)->friction, 1.0, 1e-3);
    EXPECT_EQ(lane->GetMaterialByIdx(3), nullptr);

    lane = lane_section->GetLaneById(-3);
    EXPECT_EQ(lane->GetNumberOfMaterials(), 5);
    EXPECT_NEAR(lane->GetMaterialByIdx(0)->s_offset, 0.0, 1e-3);
    EXPECT_NEAR(lane->GetMaterialByIdx(0)->friction, 1.0, 1e-3);
    EXPECT_NEAR(lane->GetMaterialByIdx(1)->s_offset, 180.0, 1e-3);
    EXPECT_NEAR(lane->GetMaterialByIdx(1)->friction, 0.1, 1e-3);
    EXPECT_NEAR(lane->GetMaterialByIdx(2)->s_offset, 190.0, 1e-3);
    EXPECT_NEAR(lane->GetMaterialByIdx(2)->friction, 0.5, 1e-3);
    EXPECT_NEAR(lane->GetMaterialByIdx(3)->s_offset, 200.0, 1e-3);
    EXPECT_NEAR(lane->GetMaterialByIdx(3)->friction, 5.5, 1e-3);
    EXPECT_EQ(lane->GetMaterialByIdx(5), nullptr);
}

TEST(RoadId, TestStringRoadId)
{
    ASSERT_EQ(roadmanager::Position::LoadOpenDrive("../../../EnvironmentSimulator/Unittest/xodr/fabriksgatan_mixed_id_types.xodr"), true);
    roadmanager::OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    EXPECT_EQ(odr->GetNumOfRoads(), 16);

    EXPECT_EQ(odr->GetRoadByIdStr("Kalle")->GetId(), 3);
    EXPECT_EQ(odr->GetRoadByIdStr("1")->GetId(), 1);
    EXPECT_EQ(odr->GetRoadByIdStr("2Kalle3")->GetId(), 128);
    EXPECT_EQ(odr->GetJunctionByIdStr("Junction4")->GetId(), 0);

    roadmanager::Position pos;

    pos.SetLanePos(3, -1, 20.0, 0.0);
    EXPECT_EQ(pos.GetTrackId(), 3);

    pos.SetLanePos(0, 1, 10.0, 0.0);
    EXPECT_EQ(pos.GetTrackId(), 0);

    pos.SetLanePos(odr->GetRoadByIdStr("2")->GetId(), 1, 10.0, 0.0);
    EXPECT_EQ(pos.GetTrackId(), 2);

    pos.SetLanePos(odr->GetRoadByIdStr("Kalle")->GetId(), 1, 10.0, 0.0);
    EXPECT_EQ(pos.GetTrackId(), 3);
    EXPECT_EQ(pos.GetJunctionId(), -1);

    pos.SetLanePos(odr->GetRoadByIdStr("2Kalle3")->GetId(), 1, 10.0, 0.0);
    EXPECT_EQ(pos.GetTrackId(), 128);
    EXPECT_EQ(pos.GetJunctionId(), 0);

    odr->Clear();
}

// Uncomment to print log output to console
// #define LOG_TO_CONSOLE

#ifdef LOG_TO_CONSOLE
static void log_callback(const char *str)
{
    printf("%s\n", str);
}
#endif

int main(int argc, char **argv)
{
#ifdef LOG_TO_CONSOLE
    if (!(Logger::Inst().IsCallbackSet()))
    {
        Logger::Inst().SetCallback(log_callback);
    }
#endif

    // testing::GTEST_FLAG(filter) = "*RoadWidthAllLanes*";
    ParseAndSetLoggerOptions(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
