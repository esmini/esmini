#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "scenarioenginedll.hpp"
#include <vector>
#include <stdexcept>

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}