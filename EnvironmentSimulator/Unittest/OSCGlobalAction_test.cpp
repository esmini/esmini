#include "OSCGlobalAction.hpp"
#include "playerbase.cpp"
#include <gtest/gtest.h>

using namespace scenarioengine;
using namespace roadmanager;

class TrafficAreaActionTestBase
{
protected:
    TrafficAreaAction* trafficAreaAction = nullptr;

    void SetUpBase()
    {
        roadmanager::Position::GetOpenDrive()->LoadOpenDriveFile("../../../EnvironmentSimulator/Unittest/xodr/trafficarea.xodr");
        // ScenarioEngine* scenarioEngine;// = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/four_roads.xosc");
        // ScenarioGateway* gateway;// = new ScenarioGateway();
        // // Entities* entities;
        // // Catalogs* catalogs;
        // // OSCEnvironment* environment;
        // ScenarioReader* reader; // = new ScenarioReader(entities, catalogs,environment);

        // class DummyScenarioEngine : public ScenarioEngine {};
        // class DummyGateway : public ScenarioGateway { };
        // class DummyReader : public ScenarioReader {};

        // auto scenarioEngine = std::make_unique<DummyScenarioEngine>();
        // auto gateway = std::make_unique<DummyGateway>();
        // auto reader = std::make_unique<DummyReader>();

        std::shared_ptr<TrafficActionContext> trafficActionContext = std::make_shared<TrafficActionContext>(*roadmanager::Position::GetOpenDrive());
        // std::shared_ptr<TrafficActionContext> trafficActionContext =
        // std::make_shared<TrafficActionContext>(nullptr, nullptr, nullptr, *roadmanager::Position::GetOpenDrive());

        trafficAreaAction = new TrafficAreaAction(nullptr, trafficActionContext);
        trafficAreaAction->SetNumberOfEntities(10);

        // TODO [hlindst9] How to get rid of this dummy output?
        LOG_INFO("Dummy logging to avoid first log being captured in tests");
    }

    void TearDownBase()
    {
        delete trafficAreaAction;
    }
};

struct RoadCursorTestParam
{
    RoadCursor  inputCursor;
    RoadCursor  expectedCursor;
    std::string log_output;
};

class TrafficAreaActionRoadCursorInfoTest : public ::testing::TestWithParam<RoadCursorTestParam>, public TrafficAreaActionTestBase
{
protected:
    void SetUp() override
    {
        SetUpBase();
    }
    void TearDown() override
    {
        TearDownBase();
    }
};

TEST_P(TrafficAreaActionRoadCursorInfoTest, SetAdditionalRoadCursorInfoTest)
{
    RoadCursor road_cursor          = GetParam().inputCursor;
    RoadCursor expected_road_cursor = GetParam().expectedCursor;

    testing::internal::CaptureStdout();
    trafficAreaAction->SetAdditionalRoadCursorInfo(road_cursor);
    std::string output = testing::internal::GetCapturedStdout();

    ASSERT_TRUE(road_cursor == expected_road_cursor);
    ASSERT_EQ(output, GetParam().log_output);
}

// Example parameters
INSTANTIATE_TEST_SUITE_P(
    RoadCursorInfoTests,
    TrafficAreaActionRoadCursorInfoTest,
    ::testing::Values(
        RoadCursorTestParam{RoadCursor{0}, RoadCursor{0, 0.0, {2, 1, -1, -2}, false, 100}, ""},
        RoadCursorTestParam{RoadCursor{1}, RoadCursor{1, 0.0, {2, 1, -1, -2}, false, 150}, ""},
        RoadCursorTestParam{RoadCursor{2}, RoadCursor{2, 0.0, {2, 1, -1, -2}, false, 200}, ""},
        RoadCursorTestParam{RoadCursor{0, 10}, RoadCursor{0, 10.0, {2, 1, -1, -2}, false, 100}, ""},
        RoadCursorTestParam{RoadCursor{0, 0, {1, -1}}, RoadCursor{0, 0.0, {1, -1}, false, 100}, ""},
        RoadCursorTestParam{RoadCursor{1, 10, {2, -2}}, RoadCursor{1, 10.0, {2, -2}, false, 150}, ""},
        RoadCursorTestParam{RoadCursor{3}, RoadCursor{3}, "[] [error] TrafficAreaAction: Road ID '3' not found in OpenDRIVE data\n"},
        RoadCursorTestParam{RoadCursor{0, 150},
                            RoadCursor{0, 100, {2, 1, -1, -2}, false, 100},
                            "[] [warn] TrafficAreaAction: RoadCursor s value 150 was clamped to 100 for road ID 0.\n"},
        RoadCursorTestParam{
            RoadCursor{0, 50, {3, 2, 1, -1, -2}},
            RoadCursor{0, 50.0, {2, 1, -1, -2}, false, 100},
            "[] [warn] TrafficAreaAction: Some lane IDs in RoadCursor not found in road ID 0 at s=50. Updating lane IDs to match road data.\n"},
        RoadCursorTestParam{
            RoadCursor{0, 50, {3, 2, 1, -1, -2}},
            RoadCursor{0, 50.0, {2, 1, -1, -2}, false, 100},
            "[] [warn] TrafficAreaAction: Some lane IDs in RoadCursor not found in road ID 0 at s=50. Updating lane IDs to match road data.\n"}));

struct RoadRangeTestParam
{
    RoadRange inputRange;
    RoadRange expectedRange;
};

// class TrafficAreaActionAdditionalRoadCursorTest : public ::testing::TestWithParam<RoadRangeTestParam>, public TrafficAreaActionTestBase
// {
// protected:
//     void SetUp() override
//     {
//         SetUpBase();
//     }
//     void TearDown() override
//     {
//         TearDownBase();
//     }
// };

// TEST_P(TrafficAreaActionAdditionalRoadCursorTest, AddComplementaryRoadCursorsTest)
// {
//     RoadRange road_range          = GetParam().inputRange;
//     RoadRange expected_road_range = GetParam().expectedRange;

//     trafficAreaAction->SetRoadRanges({road_range});

//     trafficAreaAction->AddComplementaryRoadCursors();

//     road_range = trafficAreaAction->GetRoadRanges()[0];

//     for (auto& rc : road_range.roadCursors)
//     {
//         std::cout << rc.roadId << std::endl;
//         std::cout << rc.s << std::endl;
//         std::cout << rc.road_length << std::endl;
//         for (const auto& laneId : rc.laneIds)
//         {
//             std::cout << laneId << " ";
//         }
//         std::cout << std::endl << std::endl;
//     }
//     for (auto& rc : expected_road_range.roadCursors)
//     {
//         std::cout << rc.roadId << std::endl;
//         std::cout << rc.s << std::endl;
//         std::cout << rc.road_length << std::endl;
//         for (const auto& laneId : rc.laneIds)
//         {
//             std::cout << laneId << " ";
//         }
//         std::cout << std::endl << std::endl;
//     }

//     ASSERT_EQ(road_range.roadCursors.size(), expected_road_range.roadCursors.size());

//     for (const auto& rc : road_range.roadCursors)
//     {
//         bool found = std::any_of(expected_road_range.roadCursors.begin(),
//                                  expected_road_range.roadCursors.end(),
//                                  [&](const RoadCursor& expected_rc) { return rc == expected_rc; });
//         ASSERT_TRUE(found) << "RoadCursor not found in expected_road_range.roadCursors";
//     }

//     for (const auto& expected_rc : expected_road_range.roadCursors)
//     {
//         bool found =
//             std::any_of(road_range.roadCursors.begin(), road_range.roadCursors.end(), [&](const RoadCursor& rc) { return rc == expected_rc; });
//         ASSERT_TRUE(found) << "Expected RoadCursor not found in road_range.roadCursors";
//     }
// };

// Example parameters
// INSTANTIATE_TEST_SUITE_P(
//     RoadCursorInfoTests,
//     TrafficAreaActionAdditionalRoadCursorTest,
//     ::testing::Values(
//         RoadRangeTestParam{
//             RoadRange{0, {RoadCursor{0, 0, {2, 1, -1, -2}, false, 100}, RoadCursor{1, 50, {2, 1, -1, -2}, true, 150}}},
//             RoadRange{0,
//                       {RoadCursor{0, 0, {2, 1, -1, -2}, false, 100},
//                        RoadCursor{1, 0, {2, 1, -1, -2}, false, 150},
//                        RoadCursor{1, 50, {2, 1, -1, -2}, true, 150}}},
//         },
//         RoadRangeTestParam{
//             RoadRange{0, {RoadCursor{0, 0, {2, 1, -1, -2}, false, 100}, RoadCursor{2, 0, {2, 1, -1, -2}, true, 200}}},
//             RoadRange{0,
//                       {RoadCursor{0, 0, {2, 1, -1, -2}, false, 100},
//                        RoadCursor{1, 0, {2, 1, -1, -2}, false, 150},
//                        RoadCursor{2, 0, {2, 1, -1, -2}, false, 200},
//                        RoadCursor{2, 200, {2, 1, -1, -2}, true, 200}}},
//         },
//         RoadRangeTestParam{
//             RoadRange{0, {RoadCursor{0, 50, {-1, -2}, false, 100}, RoadCursor{1, 10, {1, -1, -2}, false, 150}, RoadCursor{1, 20, {-1}, true,
//             150}}}, RoadRange{0,
//                       {RoadCursor{0, 50, {-1, -2}, false, 100},
//                        RoadCursor{1, 0, {-1, -2}, false, 150},
//                        RoadCursor{1, 10, {1, -1, -2}, false, 150},
//                        RoadCursor{1, 20, {-1}, true, 150}}},
//         },
//         RoadRangeTestParam{
//             RoadRange{100, {RoadCursor{1, 50, {2, 1}, false, 150}, RoadCursor{1, 100, {2, 1}, true, 150}}},
//             RoadRange{100, {RoadCursor{1, 50, {2, 1}, false, 150}, RoadCursor{1, 100, {2, 1}, true, 150}}},
//         }));

struct RoadRangeLengthTestParam
{
    RoadRange inputRange;
    double    expectedLength;
};

class TrafficAreaActionSetRoadRangeLengthTest : public ::testing::TestWithParam<RoadRangeLengthTestParam>, public TrafficAreaActionTestBase
{
protected:
    void SetUp() override
    {
        SetUpBase();
    }
    void TearDown() override
    {
        TearDownBase();
    }
};

TEST_P(TrafficAreaActionSetRoadRangeLengthTest, SetRoadRangeLengthTest)
{
    RoadRange road_range                 = GetParam().inputRange;
    double    expected_road_range_length = GetParam().expectedLength;

    trafficAreaAction->SetRoadRanges({road_range});

    trafficAreaAction->SetRoadRangeLength(road_range);

    // road_range = trafficAreaAction->GetRoadRanges()[0];

    ASSERT_DOUBLE_EQ(road_range.length, expected_road_range_length);
}

// Example parameters
INSTANTIATE_TEST_SUITE_P(
    RoadCursorInfoTests,
    TrafficAreaActionSetRoadRangeLengthTest,
    ::testing::Values(
        RoadRangeLengthTestParam{
            RoadRange{0, {RoadCursor{1, 50, {-1, -2}, false, 150}, RoadCursor{1, 75, {-1, -2}, true, 150}}},
            25,
        },
        RoadRangeLengthTestParam{
            RoadRange{20, {RoadCursor{1, 50, {-1, -2}, false, 150}, RoadCursor{1, 75, {-1, -2}, true, 150}}},
            20,
        },
        RoadRangeLengthTestParam{
            RoadRange{50, {RoadCursor{1, 50, {-1, -2}, false, 150}, RoadCursor{1, 75, {-1, -2}, true, 150}}},
            25,
        },

        RoadRangeLengthTestParam{
            RoadRange{0, {RoadCursor{0, 50, {-1, -2}, false, 100}, RoadCursor{1, 0, {-1, -2}, false, 150}, RoadCursor{1, 75, {-1, -2}, true, 150}}},
            125,
        },
        RoadRangeLengthTestParam{
            RoadRange{100, {RoadCursor{0, 50, {-1, -2}, false, 100}, RoadCursor{1, 0, {-1, -2}, false, 150}, RoadCursor{1, 75, {-1, -2}, true, 150}}},
            100,
        },
        RoadRangeLengthTestParam{
            RoadRange{200, {RoadCursor{0, 50, {-1, -2}, false, 100}, RoadCursor{1, 0, {-1, -2}, false, 150}, RoadCursor{1, 75, {-1, -2}, true, 150}}},
            125,
        },

        RoadRangeLengthTestParam{
            RoadRange{0,
                      {RoadCursor{0, 0, {-1, -2}, false, 100},
                       RoadCursor{1, 0, {-1, -2}, false, 150},
                       RoadCursor{2, 0, {-1, -2}, false, 200},
                       RoadCursor{2, 200, {-1, -2}, true, 200}}},
            450,
        },
        RoadRangeLengthTestParam{
            RoadRange{300,
                      {RoadCursor{0, 0, {-1, -2}, false, 100},
                       RoadCursor{1, 0, {-1, -2}, false, 150},
                       RoadCursor{2, 0, {-1, -2}, false, 200},
                       RoadCursor{2, 200, {-1, -2}, true, 200}}},
            300,
        },
        RoadRangeLengthTestParam{
            RoadRange{500,
                      {RoadCursor{0, 0, {-1, -2}, false, 100},
                       RoadCursor{1, 0, {-1, -2}, false, 150},
                       RoadCursor{2, 0, {-1, -2}, false, 200},
                       RoadCursor{2, 200, {-1, -2}, true, 200}}},
            450,
        },

        RoadRangeLengthTestParam{
            RoadRange{0,
                      {RoadCursor{0, 0, {-1, -2}, false, 100},
                       RoadCursor{1, 0, {-1, -2}, false, 150},
                       RoadCursor{2, 0, {-1, -2}, false, 200},
                       RoadCursor{2, 100, {-1, -2}, true, 200}}},
            350,
        },
        RoadRangeLengthTestParam{
            RoadRange{450,
                      {RoadCursor{0, 0, {-1, -2}, false, 100},
                       RoadCursor{1, 0, {-1, -2}, false, 150},
                       RoadCursor{2, 0, {-1, -2}, false, 200},
                       RoadCursor{2, 100, {-1, -2}, true, 200}}},
            350,
        },
        RoadRangeLengthTestParam{
            RoadRange{0,
                      {RoadCursor{0, 50, {-1, -2}, false, 100},
                       RoadCursor{1, 0, {-1, -2}, false, 150},
                       RoadCursor{2, 0, {-1, -2}, false, 200},
                       RoadCursor{2, 200, {-1, -2}, true, 200}}},
            400,
        },
        RoadRangeLengthTestParam{
            RoadRange{200,
                      {RoadCursor{0, 50, {-1, -2}, false, 100},
                       RoadCursor{1, 0, {-1, -2}, false, 150},
                       RoadCursor{2, 0, {-1, -2}, false, 200},
                       RoadCursor{2, 200, {-1, -2}, true, 200}}},
            200,
        }));

struct LaneSegmentsTestParam
{
    RoadRange                inputRange;
    std::vector<LaneSegment> expectedLaneSegments;
};

class TrafficAreaActionSetLaneSegmentsTest : public ::testing::TestWithParam<LaneSegmentsTestParam>, public TrafficAreaActionTestBase
{
protected:
    void SetUp() override
    {
        SetUpBase();
    }
    void TearDown() override
    {
        TearDownBase();
    }
};

TEST_P(TrafficAreaActionSetLaneSegmentsTest, SetLaneSegmentsTest)
{
    RoadRange                road_range             = GetParam().inputRange;
    std::vector<LaneSegment> expected_lane_segments = GetParam().expectedLaneSegments;

    trafficAreaAction->SetRoadRanges({road_range});

    trafficAreaAction->SetLaneSegments(road_range);

    std::vector<LaneSegment> generated_lane_segments = trafficAreaAction->GetLaneSegments();

    ASSERT_EQ(generated_lane_segments.size(), expected_lane_segments.size());

    for (const auto& ls : generated_lane_segments)
    {
        bool found = std::any_of(expected_lane_segments.begin(),
                                 expected_lane_segments.end(),
                                 [&](const LaneSegment& expected_ls) { return ls == expected_ls; });
        ASSERT_TRUE(found) << "LaneSegment not found in expected_lane_segments";
    }

    for (const auto& expected_ls : expected_lane_segments)
    {
        bool found =
            std::any_of(generated_lane_segments.begin(), generated_lane_segments.end(), [&](const LaneSegment& ls) { return ls == expected_ls; });
        ASSERT_TRUE(found) << "Expected RoadCursor not found in road_range.roadCursors";
    }
}

// Example parameters
INSTANTIATE_TEST_SUITE_P(
    SetLaneSegmentsTest,
    TrafficAreaActionSetLaneSegmentsTest,
    ::testing::Values(
        LaneSegmentsTestParam{RoadRange{25, {RoadCursor{1, 50, {-1, -2}, false, 150}, RoadCursor{1, 75, {-1, -2}, true, 150}}},
                              {LaneSegment{1, -1, 50, 75, 25}, LaneSegment{1, -2, 50, 75, 25}}},
        LaneSegmentsTestParam{RoadRange{20, {RoadCursor{1, 50, {-1, -2}, false, 150}, RoadCursor{1, 75, {-1, -2}, true, 150}}},
                              {LaneSegment{1, -1, 50, 70, 20}, LaneSegment{1, -2, 50, 70, 20}}},

        LaneSegmentsTestParam{
            RoadRange{125, {RoadCursor{0, 50, {-1, -2}, false, 100}, RoadCursor{1, 0, {-1, -2}, false, 150}, RoadCursor{1, 75, {-1, -2}, true, 150}}},
            {LaneSegment{0, -1, 50, 100, 50}, LaneSegment{0, -2, 50, 100, 50}, LaneSegment{1, -1, 0, 75, 75}, LaneSegment{1, -2, 0, 75, 75}}},
        LaneSegmentsTestParam{
            RoadRange{
                125,
                {RoadCursor{0, 50, {2, 1, -1, -2}, false, 100}, RoadCursor{1, 0, {-1, -2}, false, 150}, RoadCursor{1, 75, {-1, -2}, true, 150}}},
            {LaneSegment{0, 2, 50, 100, 50},
             LaneSegment{0, 1, 50, 100, 50},
             LaneSegment{0, -1, 50, 100, 50},
             LaneSegment{0, -2, 50, 100, 50},
             LaneSegment{1, -1, 0, 75, 75},
             LaneSegment{1, -2, 0, 75, 75}}},
        LaneSegmentsTestParam{
            RoadRange{100, {RoadCursor{0, 50, {-1, -2}, false, 100}, RoadCursor{1, 0, {-1, -2}, false, 150}, RoadCursor{1, 75, {-1, -2}, true, 150}}},
            {LaneSegment{0, -1, 50, 100, 50}, LaneSegment{0, -2, 50, 100, 50}, LaneSegment{1, -1, 0, 50, 50}, LaneSegment{1, -2, 0, 50, 50}}},

        LaneSegmentsTestParam{RoadRange{450,
                                        {RoadCursor{0, 0, {-1}, false, 100},
                                         RoadCursor{1, 0, {-1, -2}, false, 150},
                                         RoadCursor{2, 0, {-1, -2}, false, 200},
                                         RoadCursor{2, 200, {-1, -2}, true, 200}}},
                              {LaneSegment{0, -1, 0, 100, 100},
                               LaneSegment{1, -1, 0, 150, 150},
                               LaneSegment{1, -2, 0, 150, 150},
                               LaneSegment{2, -1, 0, 200, 200},
                               LaneSegment{2, -2, 0, 200, 200}}},
        LaneSegmentsTestParam{RoadRange{300,
                                        {RoadCursor{0, 50, {-1, -2}, false, 100},
                                         RoadCursor{1, 0, {-1, -2}, false, 150},
                                         RoadCursor{2, 0, {-1, -2}, false, 200},
                                         RoadCursor{2, 200, {-1, -2}, true, 200}}},
                              {LaneSegment{0, -1, 50, 100, 50},
                               LaneSegment{0, -2, 50, 100, 50},
                               LaneSegment{1, -1, 0, 150, 150},
                               LaneSegment{1, -2, 0, 150, 150},
                               LaneSegment{2, -1, 0, 100, 100},
                               LaneSegment{2, -2, 0, 100, 100}}},

        LaneSegmentsTestParam{
            RoadRange{300, {RoadCursor{0, 50, {-1, -2}, false, 100}, RoadCursor{1, 50, {-1, -2}, true, 150}}},
            {LaneSegment{0, -1, 50, 100, 50}, LaneSegment{0, -2, 50, 100, 50}, LaneSegment{1, -1, 0, 50, 50}, LaneSegment{1, -2, 0, 50, 50}}},

        LaneSegmentsTestParam{RoadRange{500,
                                        {RoadCursor{0, 0, {2, 1, -1, -2}, false, 100},
                                         RoadCursor{1, 0, {2, 1, -1, -2}, false, 150},
                                         RoadCursor{2, 0, {2, 1, -1, -2}, true, 200}}},
                              {LaneSegment{0, 2, 0, 100, 100},
                               LaneSegment{0, 1, 0, 100, 100},
                               LaneSegment{0, -1, 0, 100, 100},
                               LaneSegment{0, -2, 0, 100, 100},
                               LaneSegment{1, 2, 0, 150, 150},
                               LaneSegment{1, 1, 0, 150, 150},
                               LaneSegment{1, -1, 0, 150, 150},
                               LaneSegment{1, -2, 0, 150, 150},
                               LaneSegment{2, 2, 0, 200, 200},
                               LaneSegment{2, 1, 0, 200, 200},
                               LaneSegment{2, -1, 0, 200, 200},
                               LaneSegment{2, -2, 0, 200, 200}}},

        LaneSegmentsTestParam{RoadRange{500,
                                        {RoadCursor{0, 0, {2, 1, -1, -2}, false, 100},
                                         RoadCursor{1, 75, {2, 1, -1, -2}, false, 150},
                                         RoadCursor{2, 0, {2, 1, -1, -2}, true, 200}}},
                              {LaneSegment{0, 2, 0, 100, 100},
                               LaneSegment{0, 1, 0, 100, 100},
                               LaneSegment{0, -1, 0, 100, 100},
                               LaneSegment{0, -2, 0, 100, 100},
                               LaneSegment{1, 2, 75, 150, 75},
                               LaneSegment{1, 1, 75, 150, 75},
                               LaneSegment{1, -1, 75, 150, 75},
                               LaneSegment{1, -2, 75, 150, 75},
                               LaneSegment{2, 2, 0, 200, 200},
                               LaneSegment{2, 1, 0, 200, 200},
                               LaneSegment{2, -1, 0, 200, 200},
                               LaneSegment{2, -2, 0, 200, 200}}}));