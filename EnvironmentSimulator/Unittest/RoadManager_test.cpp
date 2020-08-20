#include <iostream>
#include <gtest/gtest.h>
#include "RoadManager.hpp"
#include <vector>
#include <stdexcept>

using namespace roadmanager;


////////// TESTS FOR CLASS -> Polynomial //////////

class PolynomialTestFixture: public testing::Test
{
    public:
        PolynomialTestFixture();
        PolynomialTestFixture(double a, double b, double c, double d, double p_scale);
        virtual ~PolynomialTestFixture();
    protected:
        Polynomial poly;
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
    ASSERT_EQ(0, poly.GetA());
    ASSERT_EQ(0, poly.GetB());
    ASSERT_EQ(0, poly.GetC());
    ASSERT_EQ(0, poly.GetD());
    ASSERT_EQ(1, poly.GetPscale());
}

TEST_F(PolynomialTestFixture, TestConstructorArgument)
{
    Polynomial poly = Polynomial(1,-2,3,-4);
    ASSERT_EQ(1, poly.GetA());
    ASSERT_EQ(-2, poly.GetB());
    ASSERT_EQ(3, poly.GetC());
    ASSERT_EQ(-4, poly.GetD());
    ASSERT_EQ(1, poly.GetPscale());
}

TEST_F(PolynomialTestFixture, TestConstructorArgumentPscale)
{
    Polynomial poly = Polynomial(1,-2,3,-4,2);
    ASSERT_EQ(1, poly.GetA());
    ASSERT_EQ(-2, poly.GetB());
    ASSERT_EQ(3, poly.GetC());
    ASSERT_EQ(-4, poly.GetD());
    ASSERT_EQ(2, poly.GetPscale());
}

TEST_F(PolynomialTestFixture, TestSetGet)
{
    poly.Set(1,-2,3,-4);
    ASSERT_EQ(1, poly.GetA());
    ASSERT_EQ(-2, poly.GetB());
    ASSERT_EQ(3, poly.GetC());
    ASSERT_EQ(-4, poly.GetD());
    ASSERT_EQ(1, poly.GetPscale());
}

TEST_F(PolynomialTestFixture, TestSetGetPscale)
{
    poly.Set(1,-2,3,-4, 2);
    ASSERT_EQ(1, poly.GetA());
    ASSERT_EQ(-2, poly.GetB());
    ASSERT_EQ(3, poly.GetC());
    ASSERT_EQ(-4, poly.GetD());
    ASSERT_EQ(2, poly.GetPscale());
}

TEST_F(PolynomialTestFixture, TestSetGetIndividual)
{
    poly.SetA(1);
    poly.SetB(-2);
    poly.SetC(3);
    poly.SetD(-4);
    ASSERT_EQ(1, poly.GetA());
    ASSERT_EQ(-2, poly.GetB());
    ASSERT_EQ(3, poly.GetC());
    ASSERT_EQ(-4, poly.GetD());
}

class PolynomialTestEvaluateEmptyParametrized: public testing::TestWithParam<std::tuple<double, double>>
{
    public:
    protected:
        Polynomial poly_empty;
};

TEST_P(PolynomialTestEvaluateEmptyParametrized, TestEvaluateEmptyConstructor)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    poly_empty.SetA(1);
    poly_empty.SetB(-2);
    poly_empty.SetC(3);
    poly_empty.SetD(-4);
    ASSERT_EQ(poly_empty.Evaluate(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_CASE_P(TestEvaluateParamEmpty, PolynomialTestEvaluateEmptyParametrized, testing::Values(
                                                std::make_tuple(2,-23),
                                                std::make_tuple(0, 1)));

class PolynomialTestEvaluateArgumentParametrized: public testing::TestWithParam<std::tuple<double, double>>
{
    public:
    protected:
        Polynomial poly_argument{1,-2,3,-4,2};
};

TEST_P(PolynomialTestEvaluateArgumentParametrized, TestEvaluateArgumentConstructor)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    ASSERT_EQ(poly_argument.Evaluate(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_CASE_P(TestEvaluateParamArgument, PolynomialTestEvaluateArgumentParametrized, testing::Values(
                                                std::make_tuple(2,-215),
                                                std::make_tuple(0, 1)));


class PolynomialTestEvaluatePrimEmptyParametrized: public testing::TestWithParam<std::tuple<double, double>>
{
    public:
    protected:
        Polynomial poly_empty;
};

TEST_P(PolynomialTestEvaluatePrimEmptyParametrized, TestEvaluatePrimEmptyConstructor)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    poly_empty.SetA(1);
    poly_empty.SetB(-2);
    poly_empty.SetC(3);
    poly_empty.SetD(-4);
    ASSERT_EQ(poly_empty.EvaluatePrim(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_CASE_P(TestEvaluatePrimParamEmpty, PolynomialTestEvaluatePrimEmptyParametrized, testing::Values(
                                                std::make_tuple(2,-38),
                                                std::make_tuple(0, -2)));

class PolynomialTestEvaluatePrimArgumentParametrized: public testing::TestWithParam<std::tuple<double, double>>
{
    public:
    protected:
        Polynomial poly_argument{1,-2,3,-4,2};
};

TEST_P(PolynomialTestEvaluatePrimArgumentParametrized, TestEvaluatePrimArgumentConstructor)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    ASSERT_EQ(poly_argument.EvaluatePrim(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_CASE_P(TestEvaluatePrimParamArgument, PolynomialTestEvaluatePrimArgumentParametrized, testing::Values(
                                                std::make_tuple(2,-170),
                                                std::make_tuple(0, -2)));

class PolynomialTestEvaluatePrimPrimEmptyParametrized: public testing::TestWithParam<std::tuple<double, double>>
{
    public:
    protected:
        Polynomial poly_empty;
};

TEST_P(PolynomialTestEvaluatePrimPrimEmptyParametrized, TestEvaluatePrimPrimEmptyConstructor)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    poly_empty.SetA(1);
    poly_empty.SetB(-2);
    poly_empty.SetC(3);
    poly_empty.SetD(-4);
    ASSERT_EQ(poly_empty.EvaluatePrimPrim(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_CASE_P(TestEvaluatePrimPrimParamEmpty, PolynomialTestEvaluatePrimPrimEmptyParametrized, testing::Values(
                                                std::make_tuple(2,-42),
                                                std::make_tuple(0, 6)));

class PolynomialTestEvaluatePrimPrimArgumentParametrized: public testing::TestWithParam<std::tuple<double, double>>
{
    public:
    protected:
        Polynomial poly_argument{1,-2,3,-4,2};
};

TEST_P(PolynomialTestEvaluatePrimPrimArgumentParametrized, TestEvaluatePrimPrimArgumentConstructor)
{
    std::tuple<double, double> tuple = GetParam();
    ASSERT_GE(std::get<0>(tuple), 0);
    ASSERT_EQ(poly_argument.EvaluatePrimPrim(std::get<0>(tuple)), std::get<1>(tuple));
}

INSTANTIATE_TEST_CASE_P(TestEvaluatePrimPrimParamArgument, PolynomialTestEvaluatePrimPrimArgumentParametrized, testing::Values(
                                                std::make_tuple(2,-90),
                                                std::make_tuple(0, 6)));

//////////////////////////////////////////////////////////////////////

////////// TESTS FOR CLASS -> OSIPoints //////////

//////////////////////////////////////////////////////////////////////


/******************************************
* Test the LineLink class
******************************************/
// Tests the class constructor.
TEST(LaneLinkTest, DefaultConstructor) {
    LaneLink lane_link(UNKNOWN, 0);
    EXPECT_EQ(UNKNOWN, lane_link.GetType());
    EXPECT_EQ(0,lane_link.GetId());
}

/******************************************
* Test the LaneWidth class
******************************************/
// Tests the class constructor.
TEST(LaneWidthTest, DefaultConstructor) {
    LaneWidth lane_width(1, 1, 1, 1, 1);
    EXPECT_EQ(1, lane_width.GetSOffset());
}

/******************************************
* Test the LaneBoundaryOSI class
******************************************/

/******************************************
* Test the LaneRoadMarkTypeLine class
******************************************/
class LaneRoadMarkTypeLineTest :public ::testing::TestWithParam<std::tuple<double,double,double,double,LaneRoadMarkTypeLine::RoadMarkTypeLineRule,double>> {};
// inp: length,space,t_offset,s_offset,rule,width
// expected: 

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
}

INSTANTIATE_TEST_CASE_P(LaneRoadMarkTypeLineTests,LaneRoadMarkTypeLineTest,::testing::Values(
    std::make_tuple(100,100,0,0,LaneRoadMarkTypeLine::NO_PASSING,2),
    std::make_tuple(10,10,-1,1,LaneRoadMarkTypeLine::CAUTION,6)));

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}