#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <stdexcept>

#include "RoadManager.hpp"

using namespace roadmanager;

#define TRIG_ERR_MARGIN 0.001

//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> Polynomial //////////
//////////////////////////////////////////////////////////////////////

class PolynomialTestFixture: public testing::Test
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

PolynomialTestFixture::PolynomialTestFixture(double a, double b, double c, double d, double p_scale)
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
    polynomial = Polynomial(1,-2,3,-4);
    ASSERT_EQ(1, polynomial.GetA());
    ASSERT_EQ(-2, polynomial.GetB());
    ASSERT_EQ(3, polynomial.GetC());
    ASSERT_EQ(-4, polynomial.GetD());
    ASSERT_EQ(1, polynomial.GetPscale());
}

TEST_F(PolynomialTestFixture, TestConstructorArgumentPscale)
{
    polynomial = Polynomial(1,-2,3,-4,2);
    ASSERT_EQ(1, polynomial.GetA());
    ASSERT_EQ(-2, polynomial.GetB());
    ASSERT_EQ(3, polynomial.GetC());
    ASSERT_EQ(-4, polynomial.GetD());
    ASSERT_EQ(2, polynomial.GetPscale());
}

TEST_F(PolynomialTestFixture, TestSetGet)
{
    polynomial.Set(1,-2,3,-4);
    ASSERT_EQ(1, polynomial.GetA());
    ASSERT_EQ(-2, polynomial.GetB());
    ASSERT_EQ(3, polynomial.GetC());
    ASSERT_EQ(-4, polynomial.GetD());
    ASSERT_EQ(1, polynomial.GetPscale());
}

TEST_F(PolynomialTestFixture, TestSetGetPscale)
{
    polynomial.Set(1,-2,3,-4, 2);
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

class PolynomialTestEvaluateEmptyParametrized: public testing::TestWithParam<std::tuple<double, double>>
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

INSTANTIATE_TEST_SUITE_P(TestEvaluateParamEmpty, PolynomialTestEvaluateEmptyParametrized, testing::Values(
                                                std::make_tuple(2,-23),
                                                std::make_tuple(0, 1)));

class PolynomialTestEvaluateArgumentParametrized: public testing::TestWithParam<std::tuple<double, double>>
{
    public:
    protected:
        Polynomial polynomial_argument{1,-2,3,-4,2};
};

TEST_P(PolynomialTestEvaluateArgumentParametrized, TestEvaluateArgumentConstructor)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    ASSERT_EQ(polynomial_argument.Evaluate(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluateParamArgument, PolynomialTestEvaluateArgumentParametrized, testing::Values(
                                                std::make_tuple(2,-215),
                                                std::make_tuple(0, 1)));


class PolynomialTestEvaluatePrimEmptyParametrized: public testing::TestWithParam<std::tuple<double, double>>
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

INSTANTIATE_TEST_SUITE_P(TestEvaluatePrimParamEmpty, PolynomialTestEvaluatePrimEmptyParametrized, testing::Values(
                                                std::make_tuple(2,-38),
                                                std::make_tuple(0, -2)));

class PolynomialTestEvaluatePrimArgumentParametrized: public testing::TestWithParam<std::tuple<double, double>>
{
    public:
    protected:
        Polynomial polynomial_argument{1,-2,3,-4,2};
};

TEST_P(PolynomialTestEvaluatePrimArgumentParametrized, TestEvaluatePrimArgumentConstructor)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    ASSERT_EQ(polynomial_argument.EvaluatePrim(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluatePrimParamArgument, PolynomialTestEvaluatePrimArgumentParametrized, testing::Values(
                                                std::make_tuple(2,-170),
                                                std::make_tuple(0, -2)));

class PolynomialTestEvaluatePrimPrimEmptyParametrized: public testing::TestWithParam<std::tuple<double, double>>
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

INSTANTIATE_TEST_SUITE_P(TestEvaluatePrimPrimParamEmpty, PolynomialTestEvaluatePrimPrimEmptyParametrized, testing::Values(
                                                std::make_tuple(2,-42),
                                                std::make_tuple(0, 6)));

class PolynomialTestEvaluatePrimPrimArgumentParametrized: public testing::TestWithParam<std::tuple<double, double>>
{
    public:
    protected:
        Polynomial polynomial_argument{1,-2,3,-4,2};
};

TEST_P(PolynomialTestEvaluatePrimPrimArgumentParametrized, TestEvaluatePrimPrimArgumentConstructor)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    ASSERT_EQ(polynomial_argument.EvaluatePrimPrim(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluatePrimPrimParamArgument, PolynomialTestEvaluatePrimPrimArgumentParametrized, testing::Values(
                                                std::make_tuple(2,-90),
                                                std::make_tuple(0, 6)));

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> OSIPoints //////////
//////////////////////////////////////////////////////////////////////

class OSIPointsTestFixture: public testing::Test
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

OSIPointsTestFixture::OSIPointsTestFixture(std::vector<double> s, std::vector<double> x, std::vector<double> y, std::vector<double> z, std::vector<double> h)
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
    std::vector<PointStruct> osi_points_test_set =
    {
        {0, 0, 0, 0, 0},
        {-1, -1, -1, -1, -1},
        {2, 2, 2, 2, 2}
    };

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
    std::vector<PointStruct> osi_points_test_set =
    {
        {0, 0, 0, 0, 0},
        {-1, -1, -1, -1, -1},
        {2, 2, 2, 2, 2}
    };

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
    std::vector<PointStruct> osi_points_test_set =
    {
        {0, 0, 0, 0, 0},
        {-1, -1, -1, -1, -1},
        {2, 2, 2, 2, 2}
    };

    OSIPoints osi_points_test_object = OSIPoints(osi_points_test_set);

    ASSERT_EQ(osi_points_test_object.GetXfromIdx(0), (double)0);
    ASSERT_EQ(osi_points_test_object.GetXfromIdx(1), (double)-1);
    ASSERT_EQ(osi_points_test_object.GetXfromIdx(2), (double)2);

    ASSERT_EQ(osi_points_test_object.GetYfromIdx(0), (double)0);
    ASSERT_EQ(osi_points_test_object.GetYfromIdx(1), (double)-1);
    ASSERT_EQ(osi_points_test_object.GetYfromIdx(2), (double)2);

    ASSERT_EQ(osi_points_test_object.GetZfromIdx(0), (double)0);
    ASSERT_EQ(osi_points_test_object.GetZfromIdx(1), (double)-1);
    ASSERT_EQ(osi_points_test_object.GetZfromIdx(2), (double)2);
}

TEST_F(OSIPointsTestFixture, TestGetNumOfOSIPoints)
{
    ASSERT_EQ(osi_points.GetNumOfOSIPoints(), 0);

    std::vector<PointStruct> osi_points_test_set =
    {
        {0, 0, 0, 0, 0},
        {-1, -1, -1, -1, -1},
        {2, 2, 2, 2, 2}
    };
    std::vector<double> s {0, -1, 2};
    std::vector<double> x {0, -1, 2};
    std::vector<double> y {0, -1, 2};
    std::vector<double> z {0, -1, 2};
    std::vector<double> h {0, -1, 2};

    OSIPoints osi_points_second = OSIPoints(osi_points_test_set);
    ASSERT_EQ(osi_points_second.GetNumOfOSIPoints(), 3);
}



class OSIPointsCloseCheck :public ::testing::TestWithParam<std::tuple<float,float,float,float,float,float,float,float,int>> {};
// eight points (start and end for two lines)
// l1_p1_x, l1_p1_y, l1_p2_x, l1_p2_y, l2_p1_x, l2_p1_y, l2_p2_x, l2_p2_y, expected amount points same
TEST_P(OSIPointsCloseCheck, ClosenessChecker) {

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

    int num_intersecting_points = CheckOverlapingOSIPoints(&line1, &line2,0.001);
    ASSERT_EQ(num_intersecting_points,std::get<8>(GetParam()));
}

INSTANTIATE_TEST_SUITE_P(OSI_Points_check_test,OSIPointsCloseCheck,::testing::Values(
    std::make_tuple(0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1),
    std::make_tuple(0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 2),
    std::make_tuple(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 2.0f, 1.0f, 1.0f, 0),
    std::make_tuple(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 2.0f, 0.0f, 0.0f, 1),
    std::make_tuple(0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 2)
)
);
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> Line (Geometry) //////////
//////////////////////////////////////////////////////////////////////

class LineGeomTestFixture: public testing::Test
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

LineGeomTestFixture::LineGeomTestFixture(double s, double x, double y, double hdg, double length)
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

    Line line_second = Line(2, -1, 1, 5*M_PI, 4);
    ASSERT_EQ(2, line_second.GetS());
    ASSERT_EQ(-1, line_second.GetX());
    ASSERT_EQ(1, line_second.GetY());
    ASSERT_EQ(M_PI, line_second.GetHdg());
    ASSERT_EQ(4, line_second.GetLength());
    EXPECT_EQ(line_second.GetType(), Geometry::GEOMETRY_TYPE_LINE);

    Line line_third = Line(2, -1, 1, -5*M_PI, 4);
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


class LineGeomTestEvaluateDsEmptyConstructor: public testing::TestWithParam<std::tuple<double, double, double, double>>
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
    double my_x = line.GetX();
    double my_y = line.GetY();
    double my_h = line.GetHdg();
    x = &my_x;
    y = &my_y;
    h = &my_h;
    line.EvaluateDS(std::get<0>(tuple), x, y, h);
    ASSERT_EQ(*x, std::get<1>(tuple));
    ASSERT_EQ(*y, std::get<2>(tuple));
    ASSERT_EQ(*h, std::get<3>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluateDsEmptyParam, LineGeomTestEvaluateDsEmptyConstructor, testing::Values(
                                                std::make_tuple(0, 0, 0, 0),
                                                std::make_tuple(1, 1, 0, 0),
                                                std::make_tuple(100, 100, 0, 0)));

class LineGeomTestEvaluateDsArgumentConstructor: public testing::TestWithParam<std::tuple<double, double, double, double>>
{
    public:
    protected:
        Line line{2.0, -1.0, 1.0, 5*M_PI, 4.0};
};

TEST_P(LineGeomTestEvaluateDsArgumentConstructor, TestLineGeomEvaluateDsArgument)
{
    std::tuple<double, double, double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    double *x, *y, *h;
    double my_x = line.GetX();
    double my_y = line.GetY();
    double my_h = line.GetHdg();
    x = &my_x;
    y = &my_y;
    h = &my_h;
    line.EvaluateDS(std::get<0>(tuple), x, y, h);
    ASSERT_EQ(*x, std::get<1>(tuple));
    ASSERT_EQ(*y, std::get<2>(tuple));
    ASSERT_EQ(*h, std::get<3>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluateLineDsArgumentParam, LineGeomTestEvaluateDsArgumentConstructor, testing::Values(
                                                std::make_tuple(0.0, -1.0, 1.0, M_PI),
                                                std::make_tuple(1.0, -2.0, 1.0+sin(M_PI), M_PI),
                                                std::make_tuple(100.0, -101.0, 1.0+100*sin(M_PI), M_PI)));

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> Arc (Geometry) //////////
//////////////////////////////////////////////////////////////////////

class ArcGeomTestFixture: public testing::Test
{
    public:
        ArcGeomTestFixture();
        ArcGeomTestFixture(double s, double x, double y, double hdg, double length, double curvature);
        virtual ~ArcGeomTestFixture();
    protected:
        Arc arc;
};

ArcGeomTestFixture::ArcGeomTestFixture()
{
}

ArcGeomTestFixture::ArcGeomTestFixture(double s, double x, double y, double hdg, double length, double curvature)
{
}

ArcGeomTestFixture::~ArcGeomTestFixture()
{
}

TEST_F(ArcGeomTestFixture, TestConstructorArgument)
{
    ASSERT_EQ(0.0, arc.GetCurvature());
    EXPECT_EQ(arc.GetType(), Geometry::GEOMETRY_TYPE_UNKNOWN);

    Arc arc_second = Arc(2, -1, 1, 5*M_PI, 4, 5);
    ASSERT_EQ(5.0, arc_second.GetCurvature());
    EXPECT_EQ(arc_second.GetType(), Geometry::GEOMETRY_TYPE_ARC);
}

TEST_F(ArcGeomTestFixture, TestEvaluateCurvatureDS)
{
    ASSERT_EQ(arc.EvaluateCurvatureDS(0), 0.0);
    ASSERT_EQ(arc.EvaluateCurvatureDS(10), 0.0);
    ASSERT_EQ(arc.EvaluateCurvatureDS(100), 0.0);
    ASSERT_EQ(arc.EvaluateCurvatureDS(1000), 0.0);

    Arc arc_second = Arc(2, -1, 1, 5*M_PI, 4, 5);

    ASSERT_EQ(arc_second.EvaluateCurvatureDS(0), 5.0);
    ASSERT_EQ(arc_second.EvaluateCurvatureDS(10), 5.0);
    ASSERT_EQ(arc_second.EvaluateCurvatureDS(100), 5.0);
    ASSERT_EQ(arc_second.EvaluateCurvatureDS(1000), 5.0);
}

TEST_F(ArcGeomTestFixture, TestGetRadius)
{
    Arc arc_second = Arc(2, -1, 1, 5*M_PI, 4, 5);
    ASSERT_EQ(arc_second.GetRadius(), 0.2);

    Arc arc_third = Arc(2, -1, 1, 5*M_PI, 4, -10);
    ASSERT_EQ(arc_third.GetRadius(), 0.1);
}

class ArcGeomTestEvaluateDsCurvPositive: public testing::TestWithParam<std::tuple<double, double, double, double>>
{
    public:
    protected:
        Arc arc{2.0, -1.0, 1.0, 5*M_PI, 4.0, 1.0};
};

TEST_P(ArcGeomTestEvaluateDsCurvPositive, TestArcGeomEvaluateDsArgument)
{
    std::tuple<double, double, double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    double *x, *y, *h;
    double my_x = arc.GetX();
    double my_y = arc.GetY();
    double my_h = arc.GetHdg();
    x = &my_x;
    y = &my_y;
    h = &my_h;
    arc.EvaluateDS(std::get<0>(tuple), x, y, h);
    ASSERT_THAT(*x, testing::AllOf(testing::Gt(std::get<1>(tuple)-TRIG_ERR_MARGIN), testing::Lt(std::get<1>(tuple)+TRIG_ERR_MARGIN)));
    ASSERT_THAT(*y,  testing::AllOf(testing::Gt(std::get<2>(tuple)-TRIG_ERR_MARGIN), testing::Lt(std::get<2>(tuple)+TRIG_ERR_MARGIN)));
    ASSERT_EQ(*h, std::get<3>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluateArcDsArgumentParam, ArcGeomTestEvaluateDsCurvPositive, testing::Values(
                                                std::make_tuple(0.0, -1.0, 1.0, M_PI),
                                                std::make_tuple(M_PI/2, -2.0, 0.0, M_PI+M_PI/2),
                                                std::make_tuple(M_PI, -1.0, -1.0, 2*M_PI)));

class ArcGeomTestEvaluateDsCurvNegative: public testing::TestWithParam<std::tuple<double, double, double, double>>
{
    public:
    protected:
        Arc arc{2.0, -1.0, 1.0, 5*M_PI, 4.0, -1.0};
};

TEST_P(ArcGeomTestEvaluateDsCurvNegative, TestArcGeomEvaluateDsArgument)
{
    std::tuple<double, double, double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    double *x, *y, *h;
    double my_x = arc.GetX();
    double my_y = arc.GetY();
    double my_h = arc.GetHdg();
    x = &my_x;
    y = &my_y;
    h = &my_h;
    arc.EvaluateDS(std::get<0>(tuple), x, y, h);
    ASSERT_THAT(*x, testing::AllOf(testing::Gt(std::get<1>(tuple)-TRIG_ERR_MARGIN), testing::Lt(std::get<1>(tuple)+TRIG_ERR_MARGIN)));
    ASSERT_THAT(*y,  testing::AllOf(testing::Gt(std::get<2>(tuple)-TRIG_ERR_MARGIN), testing::Lt(std::get<2>(tuple)+TRIG_ERR_MARGIN)));
    ASSERT_EQ(*h, std::get<3>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluateArcDsArgumentParam, ArcGeomTestEvaluateDsCurvNegative, testing::Values(
                                                std::make_tuple(0.0, -1.0, 1.0, M_PI),
                                                std::make_tuple(M_PI/2, -2.0, 2.0, M_PI-M_PI/2),
                                                std::make_tuple(M_PI, -1.0, 3.0, 0)));

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> Spiral (Geometry) //////////
//////////////////////////////////////////////////////////////////////

class SpiralGeomTestFixture: public testing::Test
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

SpiralGeomTestFixture::SpiralGeomTestFixture(double s, double x, double y, double hdg, double length, double curv_start, double curv_end)
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

    Spiral spiral_second = Spiral(2, -1, 1, 5*M_PI, 4, 2, 10);
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
    Spiral spiral_second = Spiral(2, -1, 1, 5*M_PI, 4, 2, 10);

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

class Poly3GeomTestFixture: public testing::Test
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

Poly3GeomTestFixture::Poly3GeomTestFixture(double s, double x, double y, double hdg, double length, double a, double b, double c, double d)
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

class Poly3GeomTestEvaluateDsCurvUmaxZero: public testing::TestWithParam<std::tuple<double, double, double, double>>
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
    double my_x = poly3.GetX();
    double my_y = poly3.GetY();
    double my_h = poly3.GetHdg();
    x = &my_x;
    y = &my_y;
    h = &my_h;
    poly3.EvaluateDS(std::get<0>(tuple), x, y, h);
    ASSERT_THAT(*x, testing::AllOf(testing::Gt(std::get<1>(tuple)-TRIG_ERR_MARGIN), testing::Lt(std::get<1>(tuple)+TRIG_ERR_MARGIN)));
    ASSERT_THAT(*y,  testing::AllOf(testing::Gt(std::get<2>(tuple)-TRIG_ERR_MARGIN), testing::Lt(std::get<2>(tuple)+TRIG_ERR_MARGIN)));
    ASSERT_THAT(*h, testing::AllOf(testing::Gt(std::get<3>(tuple) - TRIG_ERR_MARGIN), testing::Lt(std::get<3>(tuple) + TRIG_ERR_MARGIN)));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluatePoly3DsArgumentParam, Poly3GeomTestEvaluateDsCurvUmaxZero, testing::Values(
                                                std::make_tuple(0.1, 0.1, 0.008, 0.139),
                                                std::make_tuple(0.2, 0.199, 0.024, 0.159),
                                                std::make_tuple(0.8, 0.721, -0.230, -1.033),
                                                std::make_tuple(1.0, 0.707, -0.207, -1.008)));

class Poly3GeomTestEvaluateDsCurvUmaxNonZero: public testing::TestWithParam<std::tuple<double, double, double, double>>
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
    double my_x = poly3.GetX();
    double my_y = poly3.GetY();
    double my_h = poly3.GetHdg();
    x = &my_x;
    y = &my_y;
    h = &my_h;
    poly3.SetUMax(1.0);
    poly3.EvaluateDS(std::get<0>(tuple), x, y, h);
    ASSERT_THAT(*x, testing::AllOf(testing::Gt(std::get<1>(tuple)-TRIG_ERR_MARGIN), testing::Lt(std::get<1>(tuple)+TRIG_ERR_MARGIN)));
    ASSERT_THAT(*y,  testing::AllOf(testing::Gt(std::get<2>(tuple)-TRIG_ERR_MARGIN), testing::Lt(std::get<2>(tuple)+TRIG_ERR_MARGIN)));
    ASSERT_THAT(*h, testing::AllOf(testing::Gt(std::get<3>(tuple) - TRIG_ERR_MARGIN), testing::Lt(std::get<3>(tuple) + TRIG_ERR_MARGIN)));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluatePoly3DsArgumentParam, Poly3GeomTestEvaluateDsCurvUmaxNonZero, testing::Values(
                                                std::make_tuple(0.0, 0.0, 0.0, 0.0),
                                                std::make_tuple(2.0, 1.0, -1.0, -1.326)));

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> ParamPoly3 (Geometry) //////////
//////////////////////////////////////////////////////////////////////

class ParamPoly3GeomTestFixture: public testing::Test
{
    public:
        ParamPoly3GeomTestFixture();
        ParamPoly3GeomTestFixture(double s, double x, double y, double hdg, double length,
        double aU, double bU, double cU, double dU, double aV, double bV, double cV, double dV, ParamPoly3::PRangeType p_range);
        virtual ~ParamPoly3GeomTestFixture();
    protected:
        ParamPoly3 parampoly3;
};

ParamPoly3GeomTestFixture::ParamPoly3GeomTestFixture()
{
}

ParamPoly3GeomTestFixture::ParamPoly3GeomTestFixture(double s, double x, double y, double hdg, double length,
        double aU, double bU, double cU, double dU, double aV, double bV, double cV, double dV, ParamPoly3::PRangeType p_range)
{
}

ParamPoly3GeomTestFixture::~ParamPoly3GeomTestFixture()
{
}

TEST_F(ParamPoly3GeomTestFixture, TestParamPoly3ArgumentConstructor)
{

    EXPECT_EQ(parampoly3.GetType(), Geometry::GEOMETRY_TYPE_UNKNOWN);

    ParamPoly3 parampoly3_second = ParamPoly3(2, -1, 1, 5*M_PI, 4, 1, -2, 3, -4, 1, -2, 3, -4, ParamPoly3::PRangeType::P_RANGE_NORMALIZED);
    EXPECT_EQ(parampoly3_second.GetType(), Geometry::GEOMETRY_TYPE_PARAM_POLY3);
    ASSERT_EQ(parampoly3_second.poly3U_.GetA(), 1.0);
    ASSERT_EQ(parampoly3_second.poly3U_.GetB(), -2.0);
    ASSERT_EQ(parampoly3_second.poly3U_.GetC(), 3.0);
    ASSERT_EQ(parampoly3_second.poly3U_.GetD(), -4.0);
    ASSERT_EQ(parampoly3_second.poly3U_.GetPscale(), 1.0/4.0);
    ASSERT_EQ(parampoly3_second.poly3V_.GetPscale(), 1.0/4.0);

    ParamPoly3 parampoly3_third = ParamPoly3(2, -1, 1, 5*M_PI, 4, 1, -2, 3, -4, 1, -2, 3, -4, ParamPoly3::PRangeType::P_RANGE_UNKNOWN);
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
    ParamPoly3 parampoly3_second = ParamPoly3(2, -1, 1, 5*M_PI, 4, 1, -2, 3, -4, 1, -2, 3, -4, ParamPoly3::PRangeType::P_RANGE_UNKNOWN);

    ASSERT_EQ(parampoly3_second.EvaluateCurvatureDS(0), -3.0);
    ASSERT_EQ(parampoly3_second.EvaluateCurvatureDS(10), 234.0/1142.0);
    ASSERT_EQ(parampoly3_second.EvaluateCurvatureDS(100), 2394.0/119402.0);
    ASSERT_EQ(parampoly3_second.EvaluateCurvatureDS(1000), 23994.0/11994002.0);
}

class ParamPoly3GeomTestEvaluateDsCurv: public testing::TestWithParam<std::tuple<double, double, double, double>>
{
    public:
    protected:
        ParamPoly3 parampoly3 = ParamPoly3(2, -1, 1, 5*M_PI, 4, 1, -2, 3, -4, 1, -2, 3, -4, ParamPoly3::PRangeType::P_RANGE_UNKNOWN);
};

TEST_P(ParamPoly3GeomTestEvaluateDsCurv, TestParamPoly3GeomEvaluateDsArgument)
{
    std::tuple<double, double, double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    double *x, *y, *h;
    double my_x = parampoly3.GetX();
    double my_y = parampoly3.GetY();
    double my_h = parampoly3.GetHdg();
    x = &my_x;
    y = &my_y;
    h = &my_h;
    parampoly3.EvaluateDS(std::get<0>(tuple), x, y, h);
    ASSERT_THAT(*x, testing::AllOf(testing::Gt(std::get<1>(tuple)-TRIG_ERR_MARGIN), testing::Lt(std::get<1>(tuple)+TRIG_ERR_MARGIN)));
    ASSERT_THAT(*y,  testing::AllOf(testing::Gt(std::get<2>(tuple)-TRIG_ERR_MARGIN), testing::Lt(std::get<2>(tuple)+TRIG_ERR_MARGIN)));
    ASSERT_EQ(*h, std::get<3>(tuple));
}

INSTANTIATE_TEST_SUITE_P(TestEvaluateParamPoly3DsArgumentParam, ParamPoly3GeomTestEvaluateDsCurv, testing::Values(
                                                std::make_tuple(0.0, -2.0, 0.0, M_PI+atan2(-2.0,-2.0)),
                                                std::make_tuple(10.0, 214.0, 216.0, M_PI+atan2(-2.0,-2.0))));

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> Elevation /////////////////////////////
//////////////////////////////////////////////////////////////////////

class ElevationTestFixture: public testing::Test
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

ElevationTestFixture::ElevationTestFixture(double s, double a, double b, double c, double d)
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

TEST(LaneLinkTest, DefaultConstructor) {
    LaneLink lane_link(UNKNOWN, 0);
    EXPECT_EQ(UNKNOWN, lane_link.GetType());
    EXPECT_EQ(0,lane_link.GetId());
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> LaneWidth /////////////////////////////
//////////////////////////////////////////////////////////////////////

TEST(LaneWidthTest, DefaultConstructor) {
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

class LaneRoadMarkTypeLineTest :public ::testing::TestWithParam<std::tuple<double,double,double,double,LaneRoadMarkTypeLine::RoadMarkTypeLineRule,double>> {};
// inp: length,space,t_offset,s_offset,rule,width

TEST_P(LaneRoadMarkTypeLineTest, DefaultConstructor) {
    LaneRoadMarkTypeLine lane_roadmarking = LaneRoadMarkTypeLine(
        std::get<0>(GetParam()),
        std::get<1>(GetParam()),
        std::get<2>(GetParam()),
        std::get<3>(GetParam()),
        std::get<4>(GetParam()),
        std::get<5>(GetParam()));

    EXPECT_EQ(lane_roadmarking.GetLength(),std::get<0>(GetParam()));
    EXPECT_EQ(lane_roadmarking.GetSpace(),std::get<1>(GetParam()));
    EXPECT_EQ(lane_roadmarking.GetTOffset(),std::get<2>(GetParam()));
    EXPECT_EQ(lane_roadmarking.GetSOffset(),std::get<3>(GetParam()));

    // Test SetGlobalId method
    // OSI related stuff not implemented yet
    //lane_roadmarking.SetGlobalId();
    //EXPECT_EQ(lane_roadmarking.GetGlobalId(), 0);
}

INSTANTIATE_TEST_SUITE_P(LaneRoadMarkTypeLineTests,LaneRoadMarkTypeLineTest,::testing::Values(
    std::make_tuple(100,100,0,0,LaneRoadMarkTypeLine::NO_PASSING,2),
    std::make_tuple(10,10,-1,1,LaneRoadMarkTypeLine::CAUTION,6)));

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> LaneRoadMarkType ///////////////////////
//////////////////////////////////////////////////////////////////////

class LaneRoadMarkTypeTest : public ::testing::Test {
    protected:
    void SetUp() override { lane_test_0 = new LaneRoadMarkType("test", 0.2); }
    LaneRoadMarkType* lane_test_0;
};

TEST_F(LaneRoadMarkTypeTest, DefaultConstructor) {
    EXPECT_EQ(lane_test_0->GetName(),"test");
    EXPECT_EQ(lane_test_0->GetWidth(),0.2);
}

TEST_F(LaneRoadMarkTypeTest,AddLine) {
    LaneRoadMarkTypeLine * line_0 = new LaneRoadMarkTypeLine(100,100,0,0,LaneRoadMarkTypeLine::NO_PASSING,2);
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

class LaneRoadMarkTest :public ::testing::TestWithParam<std::tuple<double,LaneRoadMark::RoadMarkType,LaneRoadMark::RoadMarkWeight,LaneRoadMark::RoadMarkColor,LaneRoadMark::RoadMarkMaterial,LaneRoadMark::RoadMarkLaneChange,double,double>> {};
// inp: s_offset,type,weight,color,material,lane_change,width,height

TEST_P(LaneRoadMarkTest, DefaultConstructor) {
    LaneRoadMark lane_test_0 = LaneRoadMark(
        std::get<0>(GetParam()),
        std::get<1>(GetParam()),
        std::get<2>(GetParam()),
        std::get<3>(GetParam()),
        std::get<4>(GetParam()),
        std::get<5>(GetParam()),
        std::get<6>(GetParam()),
        std::get<7>(GetParam()));

    EXPECT_EQ(lane_test_0.GetSOffset(),std::get<0>(GetParam()));
    EXPECT_EQ(lane_test_0.GetType(),std::get<1>(GetParam()));
    EXPECT_EQ(lane_test_0.GetColor(),std::get<3>(GetParam()));
    EXPECT_EQ(lane_test_0.GetWidth(),std::get<6>(GetParam()));
    EXPECT_EQ(lane_test_0.GetHeight(),std::get<7>(GetParam()));

    LaneRoadMarkType * type_test_0 = new LaneRoadMarkType("test", 0.2);
    lane_test_0.AddType(type_test_0);
    EXPECT_EQ(lane_test_0.GetNumberOfRoadMarkTypes(),1);
    EXPECT_EQ(lane_test_0.GetLaneRoadMarkTypeByIdx(0)->GetName(),type_test_0->GetName());
}

INSTANTIATE_TEST_SUITE_P(LaneRoadMarkTests,LaneRoadMarkTest,::testing::Values(
    std::make_tuple(0,LaneRoadMark::NONE_TYPE,
        LaneRoadMark::STANDARD,
        LaneRoadMark::STANDARD_COLOR,
        LaneRoadMark::STANDARD_MATERIAL,
        LaneRoadMark::INCREASE,0.2,0),
    std::make_tuple(100,LaneRoadMark::SOLID,
        LaneRoadMark::BOLD,
        LaneRoadMark::BLUE,
        LaneRoadMark::STANDARD_MATERIAL,
        LaneRoadMark::DECREASE,0.2,-1)));

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> LaneOffSet ///////////////////////////
//////////////////////////////////////////////////////////////////////

class LaneOffsetTestFixture: public testing::Test
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

LaneOffsetTestFixture::LaneOffsetTestFixture(double s, double a, double b, double c, double d)
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


class LaneOffsetGetLaneOffsetTest: public testing::TestWithParam<std::tuple<double, double>>
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

INSTANTIATE_TEST_SUITE_P(TestGetLaneOffsetParam, LaneOffsetGetLaneOffsetTest, testing::Values(
                                                std::make_tuple(0.0, 49.0),
                                                std::make_tuple(10.0, -1871.0)));


class LaneOffsetGetLaneOffsetPrimTest: public testing::TestWithParam<std::tuple<double, double>>
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

INSTANTIATE_TEST_SUITE_P(TestGetLaneOffsetPrimParam, LaneOffsetGetLaneOffsetPrimTest, testing::Values(
                                                std::make_tuple(0.0, -62),
                                                std::make_tuple(10.0, -722.0)));

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
////////// TESTS FOR CLASS -> Lane ///////////////////////////////////
//////////////////////////////////////////////////////////////////////

class LaneTestFixture: public testing::Test
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

LaneTestFixture::LaneTestFixture(double s, double a, double b, double c, double d)
{
}

LaneTestFixture::~LaneTestFixture()
{
}

TEST_F(LaneTestFixture, TestLaneBaseGetConstructor)
{
    ASSERT_EQ(lane.GetId(), 0);
    ASSERT_EQ(lane.GetOffsetFromRef(), 0.0);
    ASSERT_EQ(lane.GetLaneType(), Lane::LaneType::LANE_TYPE_NONE);
    ASSERT_EQ(lane.GetGlobalId(), 0.0);

    Lane lane_second = Lane(1, Lane::LaneType::LANE_TYPE_DRIVING);
    ASSERT_EQ(lane_second.GetId(), 1);
    ASSERT_EQ(lane_second.GetOffsetFromRef(), 0.0);
    ASSERT_EQ(lane_second.GetLaneType(), Lane::LaneType::LANE_TYPE_DRIVING);
    ASSERT_EQ(lane_second.GetGlobalId(), 0.0);
}

TEST_F(LaneTestFixture, TestLaneAddFunctions)
{
    ASSERT_EQ(lane.GetNumberOfLinks(), 0);
    ASSERT_EQ(lane.GetNumberOfRoadMarks(), 0);
    ASSERT_EQ(lane.GetNumberOfLaneWidths(), 0);

    LaneLink *lanelink = new LaneLink(LinkType::SUCCESSOR, 3);
    LaneWidth *lanewidth = new LaneWidth(2.0, 1.0, -2.0, -3.0, 4.0);
    LaneRoadMark *laneroadmark = new LaneRoadMark(2.0, LaneRoadMark::RoadMarkType::BROKEN, LaneRoadMark::RoadMarkWeight::STANDARD,
    LaneRoadMark::RoadMarkColor::RED,LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL, LaneRoadMark::RoadMarkLaneChange::BOTH, 4.0, 2.0);

    lane.AddLink(lanelink);
    lane.AddLaneWidth(lanewidth);
    lane.AddLaneRoadMark(laneroadmark);

    ASSERT_EQ(lane.GetNumberOfLinks(), 1);
    ASSERT_EQ(lane.GetNumberOfLaneWidths(), 1);
    ASSERT_EQ(lane.GetNumberOfRoadMarks(), 1);

    delete lanelink;
    delete lanewidth;
    delete laneroadmark;
}

TEST_F(LaneTestFixture, TestLaneGetLink)
{
    LaneLink *lanelink = new LaneLink(LinkType::SUCCESSOR, 3);
    lane.AddLink(lanelink);
    LaneLink *mylanelink = lane.GetLink(LinkType::PREDECESSOR);
    LaneLink *dummylink = 0;
    ASSERT_EQ(mylanelink, dummylink);
    LaneLink *mylanelink_second = lane.GetLink(LinkType::SUCCESSOR);
    ASSERT_EQ(mylanelink_second->GetType(), LinkType::SUCCESSOR);
    ASSERT_EQ(mylanelink_second->GetId(), 3);

    delete lanelink;
}

TEST_F(LaneTestFixture, TestLaneGetWidth)
{
    LaneWidth *lanewidth = new LaneWidth(2.0, 1.0, -2.0, 3.0, -4.0);
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

    LaneWidth *mylanewidth = lane.GetWidthByIndex(0);
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

    LaneWidth *mylanewidth_second = lane.GetWidthByIndex(1);
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

    delete lanewidth;
    delete lanewidth_second;
}

TEST_F(LaneTestFixture, TestLaneGetRoadMark)
{
    LaneRoadMark *laneroadmark = new LaneRoadMark(2.0, LaneRoadMark::RoadMarkType::BROKEN, LaneRoadMark::RoadMarkWeight::STANDARD,
    LaneRoadMark::RoadMarkColor::RED,LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL, LaneRoadMark::RoadMarkLaneChange::BOTH, 4.0, 2.0);
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
    ASSERT_EQ(mylaneroadmark->GetColor(), LaneRoadMark::RoadMarkColor::RED);
    ASSERT_EQ(mylaneroadmark->GetMaterial(), LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL);
    ASSERT_EQ(mylaneroadmark->GetLaneChange(), LaneRoadMark::RoadMarkLaneChange::BOTH);

    delete laneroadmark;
}

TEST_F(LaneTestFixture, TestLaneGetRoadMark2)
{
    LaneRoadMark *laneroadmark = new LaneRoadMark(2.0, LaneRoadMark::RoadMarkType::BROKEN_BROKEN, LaneRoadMark::RoadMarkWeight::STANDARD,
    LaneRoadMark::RoadMarkColor::RED,LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL, LaneRoadMark::RoadMarkLaneChange::BOTH, 4.0, 2.0);
    lane.AddLaneRoadMark(laneroadmark);

    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(-1), std::runtime_error);
    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(1), std::runtime_error);
    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(2), std::runtime_error);

    LaneRoadMark *mylaneroadmark = lane.GetLaneRoadMarkByIdx(0);
    ASSERT_EQ(mylaneroadmark->GetType(), LaneRoadMark::RoadMarkType::BROKEN_BROKEN);

    delete laneroadmark;
}

TEST_F(LaneTestFixture, TestLaneGetRoadMark3)
{
    LaneRoadMark *laneroadmark = new LaneRoadMark(2.0, LaneRoadMark::RoadMarkType::SOLID_SOLID, LaneRoadMark::RoadMarkWeight::STANDARD,
    LaneRoadMark::RoadMarkColor::RED,LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL, LaneRoadMark::RoadMarkLaneChange::BOTH, 4.0, 2.0);
    lane.AddLaneRoadMark(laneroadmark);

    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(-1), std::runtime_error);
    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(1), std::runtime_error);
    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(2), std::runtime_error);

    LaneRoadMark *mylaneroadmark = lane.GetLaneRoadMarkByIdx(0);
    ASSERT_EQ(mylaneroadmark->GetType(), LaneRoadMark::RoadMarkType::SOLID_SOLID);

    delete laneroadmark;
}

TEST_F(LaneTestFixture, TestLaneGetRoadMark4)
{
    LaneRoadMark *laneroadmark = new LaneRoadMark(2.0, LaneRoadMark::RoadMarkType::BROKEN_SOLID, LaneRoadMark::RoadMarkWeight::STANDARD,
    LaneRoadMark::RoadMarkColor::RED,LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL, LaneRoadMark::RoadMarkLaneChange::BOTH, 4.0, 2.0);
    lane.AddLaneRoadMark(laneroadmark);

    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(-1), std::runtime_error);
    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(1), std::runtime_error);
    ASSERT_THROW(lane.GetLaneRoadMarkByIdx(2), std::runtime_error);

    LaneRoadMark *mylaneroadmark = lane.GetLaneRoadMarkByIdx(0);
    ASSERT_EQ(mylaneroadmark->GetType(), LaneRoadMark::RoadMarkType::BROKEN_SOLID);

    delete laneroadmark;
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
    LaneRoadMark *laneroadmark = new LaneRoadMark(0, LaneRoadMark::RoadMarkType::BROKEN, LaneRoadMark::RoadMarkWeight::STANDARD,
    LaneRoadMark::RoadMarkColor::STANDARD_COLOR, LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL, LaneRoadMark::RoadMarkLaneChange::BOTH,
    1.0, 1.0);
    LaneRoadMark *laneroadmark_second = new LaneRoadMark(50, LaneRoadMark::RoadMarkType::BROKEN, LaneRoadMark::RoadMarkWeight::STANDARD,
    LaneRoadMark::RoadMarkColor::STANDARD_COLOR, LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL, LaneRoadMark::RoadMarkLaneChange::BOTH,
    2.0, 2.0);

    LaneRoadMarkType *laneroadmarktype = new LaneRoadMarkType("type1", 1.0);
    LaneRoadMarkType *laneroadmarktype_second = new LaneRoadMarkType("type2", 1.0);

    LaneRoadMarkTypeLine *laneRoadMarktypeline = new LaneRoadMarkTypeLine(3.0, 1.0, 0.5, 0.0, LaneRoadMarkTypeLine::RoadMarkTypeLineRule::CAUTION, 1.0);
    LaneRoadMarkTypeLine *laneRoadMarktypeline_second = new LaneRoadMarkTypeLine(3.0, 1.0, 0.5, 50.0, LaneRoadMarkTypeLine::RoadMarkTypeLineRule::CAUTION, 1.0);

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

    ASSERT_THAT(all_glob_ids.size(),3);
    ASSERT_THAT(laneroadmarktype->GetLaneRoadMarkTypeLineByIdx(0)->GetGlobalId(), 0);
    ASSERT_THAT(laneroadmarktype->GetLaneRoadMarkTypeLineByIdx(1)->GetGlobalId(), 1);
    ASSERT_THAT(laneroadmarktype_second->GetLaneRoadMarkTypeLineByIdx(0)->GetGlobalId(), 1);

    delete odr;
    delete laneroadmark;
    delete laneroadmark_second;

    delete laneroadmarktype;
    delete laneroadmarktype_second;

    delete laneRoadMarktypeline;
    delete laneRoadMarktypeline_second;
}

TEST(RoadTest, RoadWidthAllLanes)
{
    roadmanager::OpenDrive* odr = new OpenDrive("../../../resources/xodr/soderleden.xodr");

    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 5);

    Road* road = odr->GetRoadByIdx(1);
    EXPECT_EQ(road->GetId(), 1);

    EXPECT_NEAR(road->GetWidth(0, -1), 5.8, 1e-5);
    EXPECT_NEAR(road->GetWidth(0, 1), 2.3, 1e-5);
    EXPECT_NEAR(road->GetWidth(0, 0), 8.1, 1e-5);

    delete odr;
}

TEST(RoadTest, RoadWidthDrivingLanes)
{
    roadmanager::OpenDrive* odr = new OpenDrive("../../../resources/xodr/soderleden.xodr");

    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 5);

    Road* road = odr->GetRoadByIdx(1);
    EXPECT_EQ(road->GetId(), 1);

    EXPECT_DOUBLE_EQ(road->GetWidth(0, -1, Lane::LaneType::LANE_TYPE_ANY_DRIVING), 3.5);
    EXPECT_DOUBLE_EQ(road->GetWidth(0, 1, Lane::LaneType::LANE_TYPE_ANY_DRIVING), 0.0);
    EXPECT_DOUBLE_EQ(road->GetWidth(0, 0, Lane::LaneType::LANE_TYPE_ANY_DRIVING), 3.5);

    delete odr;
}

TEST(TrajectoryTest, PolyLineBase_YawInterpolation)
{
    PolyLineBase pline;
    TrajVertex v = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false };

    // Simple case
    pline.AddVertex(0.0, 0.0, 0.0, 0.0);
    pline.AddVertex(1.0, 0.0, 0.0, 1.0);
    pline.Evaluate(0.5, v);

    EXPECT_NEAR(v.x, 0.5, 1e-5);
    EXPECT_NEAR(v.y, 0.0, 1e-5);
    EXPECT_NEAR(v.h, 0.5, 1e-5);

    // Wrap around case 1
    pline.Reset();
    pline.AddVertex(0.0, 0.0, 0.0, 2 * M_PI - 0.01);
    pline.AddVertex(1.0, 0.0, 0.0, 0.09);
    pline.Evaluate(0.5, v);

    EXPECT_NEAR(v.x, 0.5, 1e-5);
    EXPECT_NEAR(v.y, 0.0, 1e-5);
    EXPECT_NEAR(v.h, 0.04, 1e-5);

    // Wrap around case 2
    pline.Reset();
    pline.AddVertex(10.0, 10.0, 0.0, 0.1);
    pline.AddVertex(10.0, 0.0, 0.0, -0.5);
    pline.Evaluate(5.0, v);

    EXPECT_NEAR(v.x, 10.0, 1e-5);
    EXPECT_NEAR(v.y, 5.0, 1e-5);
    EXPECT_NEAR(v.h, 6.0831853, 1e-5);

    // Wrap around case 3
    pline.Reset();
    pline.AddVertex(0.0, 0.0, 0.0, 0.1);
    pline.AddVertex(0.0, 10.0, 0.0, 8.1);
    pline.Evaluate(5.0, v);

    EXPECT_NEAR(v.x, 0.0, 1e-5);
    EXPECT_NEAR(v.y, 5.0, 1e-5);
    EXPECT_NEAR(v.h, 0.958407, 1e-5);
}

TEST(DistanceTest, CalcDistanceLong)
{
    double dist = 0.0;
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/fabriksgatan.xodr");
    OpenDrive* odr = Position::GetOpenDrive();

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
    pos0.SetLanePos(3, 1, 5.0, -0.25);
    pos1.SetLanePos(0, -1, 15.0, 0.15);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, dist), 0);
    EXPECT_NEAR(dist, 3.1, 1e-5);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, dist), 0);
    EXPECT_NEAR(dist, 139.317791, 1e-5);

    // Route is found, but two lanes delta = 3.6 m
    pos0.SetLanePos(3, -1, 5.0, -0.25);
    pos1.SetLanePos(2, -1, 1.0, 0.15);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, dist), 0);
    EXPECT_NEAR(dist, 3.6, 1e-5);

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
    pos0.SetHeading(1.57);
    pos1.SetLanePos(1, -1, 1.0, 0.15);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ENTITY, RelativeDistanceType::REL_DIST_LATERAL, dist), 0);
    EXPECT_NEAR(dist, -7.163675, 1e-5);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ENTITY, RelativeDistanceType::REL_DIST_LONGITUDINAL, dist), 0);
    EXPECT_NEAR(dist, -13.481041, 1e-5);

    // Target position in front, to the left
    pos0.SetHeading(0.0);
    pos1.SetLanePos(1, -1, 1.0, 0.15);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ENTITY, RelativeDistanceType::REL_DIST_LATERAL, dist), 0);
    EXPECT_NEAR(dist, 7.163675, 1e-5);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ENTITY, RelativeDistanceType::REL_DIST_LONGITUDINAL, dist), 0);
    EXPECT_NEAR(dist, 13.481041, 1e-5);
}

TEST(NurbsTest, TestNurbsPosition)
{
    NurbsShape n(4);

    n.AddControlPoint(Position(-4.0, -4.0, 0.0, 0.0, 0.0, 0.0), 0.0, 1.0, true);
    n.AddControlPoint(Position(-2.0, 4.0, 0.0, 0.0, 0.0, 0.0), 0.0, 1.0, true);
    n.AddControlPoint(Position(2.0, -4.0, 0.0, 0.0, 0.0, 0.0), 0.0, 1.0, true);
    n.AddControlPoint(Position(4.0, 4.0, 0.0, 0.0, 0.0, 0.0), 0.0, 1.0, true);

    std::vector<double> knots = { 0, 0, 0, 0, 1, 1, 1, 1 };
    n.AddKnots(knots);
    n.CalculatePolyLine();

    TrajVertex v;

    n.Evaluate(0, Shape::TrajectoryParamType::TRAJ_PARAM_TYPE_S, v);
    EXPECT_DOUBLE_EQ(v.x, -4.0);
    EXPECT_DOUBLE_EQ(v.y, -4.0);

    n.Evaluate(0.5 * n.GetLength(), Shape::TrajectoryParamType::TRAJ_PARAM_TYPE_S, v);
    EXPECT_NEAR(v.x, 0.0, 1e-5);
    EXPECT_NEAR(v.y, 0.0, 1e-5);

    n.Evaluate(1.0 * n.GetLength(), Shape::TrajectoryParamType::TRAJ_PARAM_TYPE_S, v);
    EXPECT_NEAR(v.x, 4.0, 1e-5);
    EXPECT_NEAR(v.y, 4.0, 1e-5);

    n.Evaluate(0.40045 * n.GetLength(), Shape::TrajectoryParamType::TRAJ_PARAM_TYPE_S, v);
    EXPECT_NEAR(v.x, -1.248623, 1e-5);
    EXPECT_NEAR(v.y, -0.087722, 1e-5);
    EXPECT_NEAR(v.p, 0.360046, 1e-5);
}

TEST(Route, TestAssignRoute)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/fabriksgatan.xodr");
    OpenDrive* odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 16);

    const int nrWaypoints = 6;
    Route route;
    Position routepos[nrWaypoints];
    routepos[0].SetLanePos(0, 1, 10.0, 0);
    routepos[1].SetLanePos(0, 1, 7.0, 0);   // Add extra waypoint on first road - should be removed
    routepos[2].SetLanePos(8, -1, 2.0, 0);
    routepos[3].SetLanePos(8, -1, 4.0, 0);  // Add extra waypoint on same road - previous should be ignored
    routepos[4].SetLanePos(1, -1, 2.0, 0);
    routepos[5].SetLanePos(1, -1, 1.0, 0);  // Add extra waypoint on same road - previous should be ignored
    for (int i = 0; i < nrWaypoints; i++)
    {
        route.AddWaypoint(&routepos[i]);
    }

    EXPECT_EQ(route.minimal_waypoints_.size(), 3);
    EXPECT_DOUBLE_EQ(route.minimal_waypoints_[0].GetTrackId(), 0);
    EXPECT_DOUBLE_EQ(route.minimal_waypoints_[0].GetS(), 10.0);
    EXPECT_DOUBLE_EQ(route.minimal_waypoints_[1].GetTrackId(), 8);
    EXPECT_DOUBLE_EQ(route.minimal_waypoints_[1].GetS(), 4.0);
    EXPECT_DOUBLE_EQ(route.minimal_waypoints_[2].GetTrackId(), 1);
    EXPECT_DOUBLE_EQ(route.minimal_waypoints_[2].GetS(), 1.0);

    Position pos0 = Position(0, 1, 9.0, 0.5);
    pos0.SetRoute(&route);
    EXPECT_DOUBLE_EQ(pos0.GetRouteS(), 1.0);

    // Set a position in intersection, near route
    pos0.SetLanePos(8, -1, 1.5, -0.5);
    EXPECT_EQ(pos0.SetRoute(&route), 0);
    EXPECT_DOUBLE_EQ(pos0.GetRouteS(), 11.5);

    // Set a position in intersection, at a lane not part of the route
    pos0.SetLanePos(16, -1, 1.0, 0.0);
    EXPECT_EQ(pos0.SetRoute(&route), -1);  // pos not along the route
}

TEST(GeoReferenceTest, TestNoGeoReferenceSimpleRoad)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/straight_500m_signs.xodr");
    OpenDrive* odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    GeoReference* geo_ref = odr->GetGeoReference();
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
    OpenDrive* odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    GeoReference* geo_ref = odr->GetGeoReference();
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
    OpenDrive* odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    roadmanager::RoadProbeInfo probe_data;
    Position pos_pivot = Position(0, -1, 5.0, 0.0);

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
    OpenDrive* odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 16);

    roadmanager::RoadProbeInfo probe_data;

    // Position on left side, looking beyond road start point.
    Position pos_pivot = Position(3, 1, 5.0, 0.0);
    pos_pivot.SetHeadingRelative(M_PI);

    EXPECT_EQ(pos_pivot.GetProbeInfo(20.0, &probe_data, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER), Position::ErrorCode::ERROR_END_OF_ROAD);
    EXPECT_EQ(probe_data.road_lane_info.roadId, 3);
    EXPECT_EQ(probe_data.road_lane_info.laneId, 1);
    EXPECT_NEAR(probe_data.road_lane_info.heading, GetAngleSum(pos_pivot.GetH(), M_PI), 1E-5);
    EXPECT_DOUBLE_EQ(probe_data.road_lane_info.s, 0.0);

    // Position on right side, looking through the intersection
    pos_pivot.SetLanePos(3, -1, 5.0, 0.0);
    pos_pivot.SetHeadingRelative(0.0);
    EXPECT_EQ(pos_pivot.GetProbeInfo(130.0, &probe_data, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER), Position::ErrorCode::ERROR_NO_ERROR);
    EXPECT_EQ(probe_data.road_lane_info.roadId, 1);
    EXPECT_EQ(probe_data.road_lane_info.laneId, -1);
    EXPECT_NEAR(probe_data.road_lane_info.heading, 0.192980, 1E-5);
    EXPECT_NEAR(probe_data.road_lane_info.s, 5.23650, 1E-5);
}

TEST(DeltaTest, TestDelta)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/fabriksgatan.xodr");
    OpenDrive* odr = Position::GetOpenDrive();
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

    pos_target.SetLanePos(2, -1, 250.0, 0.0);
    pos_target.SetHeadingRelative(M_PI);
    EXPECT_EQ(pos_pivot.Delta(&pos_target, pos_diff), true);
    EXPECT_NEAR(pos_diff.ds, 74.56580, 1E-5);
    EXPECT_EQ(pos_diff.dLaneId, 2);

    pos_target.SetLanePos(3, -1, 100.0, 0.0);
    pos_target.SetHeadingRelative(0.0);
    EXPECT_EQ(pos_pivot.Delta(&pos_target, pos_diff), true);
    EXPECT_NEAR(pos_diff.ds, 34.31779, 1E-5);
    EXPECT_NEAR(pos_diff.dt, 3.5, 1E-5);
    EXPECT_EQ(pos_diff.dLaneId, 2);

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
    OpenDrive* odr = Position::GetOpenDrive();
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
    OpenDrive* odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 63);

    EXPECT_EQ(odr->GetNumberOfControllers(), 23);

    // check a few samples
    roadmanager::Controller* controller;

    controller = odr->GetControllerByIdx(0);
    EXPECT_EQ(controller->GetName(), "ctrl001");
    int signalIds1[] = { 294, 295, 287, 288 };
    for (int i = 0; i < controller->GetNumberOfControls(); i++)
    {
        EXPECT_EQ(controller->GetControl(i)->signalId_, signalIds1[i]);
    }

    controller = odr->GetControllerByIdx(22);
    EXPECT_EQ(controller->GetName(), "ctrl027");
    int signalIds2[] = { 33617, 33618 };
    for (int i = 0; i < controller->GetNumberOfControls(); i++)
    {
        EXPECT_EQ(controller->GetControl(i)->signalId_, signalIds2[i]);
    }

    JunctionController* jcontroller;
    Junction* junction = odr->GetJunctionByIdx(1);
    EXPECT_EQ(junction->GetNumberOfControllers(), 5);
    int controllerIds[] = { 7, 9, 10, 8, 6 };
    for (int i = 0; i < (int)junction->GetNumberOfControllers(); i++)
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
    OpenDrive* odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 16);

    Road* road = odr->GetRoadById(1);
    EXPECT_NE(road, nullptr);

    Position pos;
    double s = 16.0;
    LaneSection* laneSection = nullptr;

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

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}