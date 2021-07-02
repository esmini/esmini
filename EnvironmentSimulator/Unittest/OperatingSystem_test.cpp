#include <iostream>
#include <gtest/gtest.h>

TEST(UnixSuite, CheckUnix)
{
    std::cout << "Check Operating System -> Unix \n";
    #ifdef __WIN32
    FAIL();
    #else
    SUCCEED();
    #endif
}

TEST(WindowsSuite, CheckWindows)
{
    std::cout << "Check Operating System - Windows \n";
    #ifdef __WIN32
    SUCCEED();
    #else
    FAIL();
    #endif
}

int main(int argc, char **argv)
{
    #ifdef __WIN32
        ::testing::GTEST_FLAG(filter) = "WindowsSuite.*";
    #else
        ::testing::GTEST_FLAG(filter) = "UnixSuite.*";
    #endif

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}