#include <gtest/gtest.h>

#include <cstddef>
#include <functional>
#include <set>
#include <vector>
#include "KdTree.h"

struct TestCircle
{
    int    id = -1;
    double x  = 0.0;
    double y  = 0.0;
    double r  = 0.0;
};

// NOLINTNEXTLINE(misc-use-internal-linkage)
bool operator==(const TestCircle& a, const TestCircle& b)
{
    return a.id == b.id;
}

namespace
{
    KdTree::BoundingBox CircleBoundingBox(const TestCircle& c)
    {
        return KdTree::BoundingBox({.x = c.x - c.r, .y = c.y - c.r}, {.x = c.x + c.r, .y = c.y + c.r});
    }

    bool IsPointInCircle(const KdTree::Point& p, const TestCircle& c)
    {
        const double dx = c.x - p.x;
        const double dy = c.y - p.y;
        return dx * dx + dy * dy <= c.r * c.r;
    }
}  // namespace

namespace std
{
    template <>
    struct hash<TestCircle>
    {
        std::size_t operator()(const TestCircle& k) const noexcept
        {
            return hash<int>()(k.id);
        }
    };
}  // namespace std

TEST(KdTree, CheckBasicBoundingBoxFunc)
{
    // check empty box
    {
        constexpr KdTree::BoundingBox boxEmpty;
        ASSERT_TRUE(boxEmpty.IsEmpty());
    }

    auto checkBox = [](const KdTree::BoundingBox& box, const double sizeX, const double sizeY, const double centerX, const double centerY)
    {
        ASSERT_DOUBLE_EQ(box.Size(0), sizeX);
        ASSERT_DOUBLE_EQ(box.Size(1), sizeY);
        ASSERT_DOUBLE_EQ(box.Center(0), centerX);
        ASSERT_DOUBLE_EQ(box.Center(1), centerY);
    };

    // test boxes created using the constructor
    {
        const KdTree::BoundingBox boxPositive({.x = 2.0, .y = 1.0}, {.x = 3.0, .y = 3.0});
        checkBox(boxPositive, 1.0, 2.0, 2.5, 2.0);

        const KdTree::BoundingBox boxNegative({.x = -3.0, .y = -3.0}, {.x = -2.0, .y = -1.0});
        checkBox(boxNegative, 1.0, 2.0, -2.5, -2.0);
    }

    // test boxes created using the |= operator
    {
        KdTree::BoundingBox boxPositive;
        boxPositive |= {.x = 2.0, .y = 1.0};
        boxPositive |= {.x = 3.0, .y = 3.0};
        checkBox(boxPositive, 1.0, 2.0, 2.5, 2.0);

        KdTree::BoundingBox boxNegative;
        boxNegative |= {.x = -2.0, .y = -1.0};
        boxNegative |= {.x = -3.0, .y = -3.0};
        checkBox(boxNegative, 1.0, 2.0, -2.5, -2.0);
    }
}

TEST(KdTree, CheckBasicKdTreeXYFunc)
{
    auto createTree = [](const std::vector<TestCircle>& circles) { return KdTree::TreeXY<TestCircle>(circles, CircleBoundingBox); };

    auto getIntersectingCircle = [](const KdTree::Point& p, const KdTree::TreeXY<TestCircle>& tree)
    {
        TestCircle out;

        auto circleCallback = [&p, &out](const TestCircle& c)
        {
            if (IsPointInCircle(p, c))
            {
                out = c;
                return true;
            }
            return false;
        };

        tree.PointIntersects(p, circleCallback);
        return out;
    };

    auto getIntersectingCircleSet = [](const KdTree::BoundingBox& box, const KdTree::TreeXY<TestCircle>& tree)
    {
        std::set<int> set;
        tree.BoxOverlaps(box,
                                [&set](const TestCircle& c)
                                {
                                    set.insert(c.id);
                                    return false;
                                });
        return set;
    };

    // a tree with no circles
    {
        auto tree = createTree({});

        // query the empty tree
        {
            ASSERT_EQ(tree.GetElementsCount(), 0);
            ASSERT_EQ(getIntersectingCircle({0.0, 0.0}, tree).id, -1);
            ASSERT_EQ(getIntersectingCircle({1.0, -1.0}, tree).id, -1);
        }
    }

    // several different circles
    {
        std::vector<TestCircle> features{
            {.id = 0, .x = 0.0, .y = 0.0, .r = 1.0},
            {.id = 1, .x = 2.0, .y = 3.0, .r = 1.0},
            {.id = 2, .x = 3.0, .y = 3.0, .r = 0.5},
            {.id = 3, .x = 10.0, .y = 12.0, .r = 1.0},
            {.id = 4, .x = -10.0, .y = -12.0, .r = 1.0},
            {.id = 5, .x = 15.0, .y = 0.0, .r = 1.0},
        };

        auto tree = createTree(features);

        // query the tree
        {
            ASSERT_EQ(tree.GetElementsCount(), features.size());
            ASSERT_EQ(getIntersectingCircle({0.0, 0.0}, tree).id, 0);
            ASSERT_EQ(getIntersectingCircle({0.0, -1.0}, tree).id, 0);
            ASSERT_EQ(getIntersectingCircle({1.0, -1.0}, tree).id, -1);
            ASSERT_NE(getIntersectingCircle({2.5, 3.0}, tree).id, -1);
            ASSERT_EQ(getIntersectingCircle({3.0, 3.5}, tree).id, 2);
            ASSERT_EQ(getIntersectingCircle({15.0, 0.0}, tree).id, 5);
            ASSERT_EQ(getIntersectingCircleSet(KdTree::BoundingBox({2.0, 2.0}, {3.0, 3.0}), tree), (std::set<int>{1, 2}));
        }
    }

    // multiple circles in the same position
    {
        auto tree = createTree({
            {.id = 0, .x = 2.0, .y = 2.0, .r = 1.0},
            {.id = 1, .x = 2.0, .y = 2.0, .r = 1.0},
            {.id = 2, .x = 2.0, .y = 2.0, .r = 1.0},
        });

        ASSERT_EQ(getIntersectingCircle({0.0, 0.0}, tree).id, -1);
        ASSERT_EQ(getIntersectingCircleSet(KdTree::BoundingBox({2.0, 2.0}, {2.0, 2.0}), tree), (std::set<int>{0, 1, 2}));
    }
}

