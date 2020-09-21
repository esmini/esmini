#include <gmock/gmock.h>
#include "RoadManager.hpp"

using namespace roadmanager;

class LaneMock: public Lane
{
    public:
        LaneMock(int id, Lane::LaneType type);

        MOCK_METHOD(RoadMarkInfo, GetRoadMarkInfoByS, (int, int, double));
};