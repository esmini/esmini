#include <gmock/gmock.h>
#include "RoadManager.hpp"

using namespace roadmanager;

class LaneMock: public Lane
{
    public:
        LaneMock(int id, Lane::LaneType type);

        MOCK_METHOD(RoadMarkInfo, GetRoadMarkInfoByS, (int, int, double));
        MOCK_METHOD(int, GetNumberOfRoadMarks, ());
};

class LaneSectionMock: public LaneSection
{
    public:
        LaneSectionMock(double s);

        MOCK_METHOD(Lane*, GetLaneById, (double));
};

class RoadMock: public Road
{
    public:
        RoadMock(int id, std::string name);

        MOCK_METHOD(int, GetLaneSectionIdxByS, (double, int));
        MOCK_METHOD(LaneSection*, GetLaneSectionByIdx, (int));
};

class PositionMock: public Position
{
    public:
        PositionMock(int track_id, double s, double t);

        MOCK_METHOD(Road*, GetRoadById, (int));
};

