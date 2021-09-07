#include <gtest/gtest.h>

#include "CommonMini.hpp"
#include "esminiLib.hpp"

struct Coordinate2D
{
  double x;
  double y;
};

class Local2Global : public ::testing::TestWithParam<std::tuple<Coordinate2D,
  Coordinate2D, double, Coordinate2D>>
{
 public:
  Coordinate2D global_host, target_host, target_global, result;
  double theta; // angle between the coordinate systems
};

TEST_P(Local2Global, RotationAndTranslation)
{
  target_host = std::get<0>(GetParam());
  global_host = std::get<1>(GetParam());
  theta = std::get<2>(GetParam());
  result = std::get<3>(GetParam());

  Local2GlobalCoordinates(target_global.x, target_global.y,
      global_host.x, global_host.y, theta, target_host.x, target_host.y);

  EXPECT_DOUBLE_EQ(result.x, target_global.x);
  EXPECT_DOUBLE_EQ(result.y, target_global.y);
}

INSTANTIATE_TEST_SUITE_P(CommonMini, Local2Global,
    ::testing::Values(std::make_tuple(Coordinate2D{0, 1}, Coordinate2D{1, 1},
                                      -M_PI / 2, Coordinate2D{2, 1}),

                      std::make_tuple(Coordinate2D{1, 0}, Coordinate2D{1, 1},
                                      -M_PI / 2, Coordinate2D{1, 0})
                     ));

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
