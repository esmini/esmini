#include <iostream>
#include <gtest/gtest.h>
#include "RoadManager.hpp"
#include <vector>
#include <stdexcept>

using namespace roadmanager;

/////////////////////////////////////////////////////////////////////////////////////////////////////////

class PolynomialTestFixture: public testing::Test
{
    public:
        PolynomialTestFixture();
        PolynomialTestFixture(double a, double b, double c, double d, double p_scale);
        virtual ~PolynomialTestFixture();
        //void SetUp() override;
        //void TearDown() override;
        //static void SetUpTestCase();
        //static void TearDownTestCase();
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

TEST_F(PolynomialTestFixture, asd)
{
    ASSERT_EQ(0,poly.GetA());
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}