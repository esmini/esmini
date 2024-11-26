#pragma once
/*
* Copyright 2024, dSPACE GmbH, All rights reserved.
* SPDX-License-Identifier: MPL-2.0
* kduvnjak@dspace.hr
* mdransfeld@dspace.de
*/

#include "CommonMini.hpp"

#include <optional>

#undef min
#undef max

namespace KdTree
{
    struct Point
    {
        double x;
        double y;
        double operator[](size_t i) const { return i == 0 ? x : y; }
        Point operator+(const Point& p) const { return Point {x + p.x, y + p.y}; }
        Point operator-(const Point& p) const { return Point {x - p.x, y - p.y}; }

        bool operator==(const Point& other) const
        {
            return x == other.x && y == other.y;
        }

        bool operator<(const Point& other) const
        {
            if (x < other.x)
                return true;
            if (x > other.x)
                return false;
            return y < other.y;
        }
    };

    // 2d bounding box structure
    struct BoundingBox
    {
        BoundingBox() = default;
        BoundingBox(const Point& min, const Point& max) : min(min), max(max) {}

        BoundingBox& operator|=(const Point& p)
        {
            // because a point can be added to an empty box, we always update both min and max values
            min.x = std::min(min.x, p.x);
            min.y = std::min(min.y, p.y);
            max.x = std::max(max.x, p.x);
            max.y = std::max(max.y, p.y);
            return *this;
        }

        BoundingBox& operator|=(const BoundingBox& b)
        {
            if (!b.IsEmpty())
            {
                *this |= b.min;
                *this |= b.max;
            }
            return *this;
        }

        double Size(size_t axis) const { return max[ axis ] - min[ axis ]; }
        double Center(size_t axis) const { return min[ axis ] + Size(axis) * 0.5; }

        // empty box is one where no points have been added (min > max)
        bool IsEmpty() const { return Size(0) < 0.0 || Size(1) < 0.0; }

        // point containment test - can be used with any point type that defines operator[]
        template<class T>
        bool Contains(const T& p) const
        {
            return p[ 0 ] >= min[ 0 ] && p[ 0 ] <= max[ 0 ] && p[ 1 ] >= min[ 1 ] && p[ 1 ] <= max[ 1 ];
        }

        BoundingBox Expand(const double v) const
        {
            BoundingBox b(*this);
            b.min.x -= v;
            b.min.y -= v;
            b.max.x += v;
            b.max.y += v;
            return b;
        }

        bool Touches(const BoundingBox& other) const
        {
            if (IsEmpty() || other.IsEmpty())
            {
                return false;
            }
            return IntervalsOverlap(min[ 0 ], max[ 0 ], other.min[ 0 ], other.max[ 0 ])
            && IntervalsOverlap(min[ 1 ], max[ 1 ], other.min[ 1 ], other.max[ 1 ]);
        }

        bool operator==(const BoundingBox& other) const
        {
            return min == other.min && max == other.max;
        }

        bool operator<(const BoundingBox& other) const
        {
            if (min < other.min)
                return true;
            if (other.min < min)
                return false;
            return max < other.max;
        }

        // axis aligned boundaries: initialized so the box is empty (min > max)
        // and that any point can be added to the box
        Point min {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
        Point max {-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()};
    };

    // Implementation of AABB tree using kdtree structure
    template <typename T, int DIM = 2>
    class TreeXY
    {
        struct ElementIdWBBox
        {
            size_t idx;
            KdTree::BoundingBox bbox;
        };

        class Node
        {
        public:

            Node() = default;
            KdTree::BoundingBox aabb;
            size_t nodeIdx;
            size_t              axis;

            size_t elemIdx;

            std::optional<size_t> leftChildIdx;
            std::optional<size_t> rightChildIdx;

            bool isLeaf() const
            {
                return !leftChildIdx.has_value() && !rightChildIdx.has_value();
            }
        };

        // 'callback' should take a const reference to T and return a boolean (false - continue, true - stop search)
        template <typename F>
        void Hit(const Node& node, const KdTree::BoundingBox& bb, bool& done, F callback) const
        {
            if (done || !root_.has_value())
                return;

            if (node.isLeaf())
            {
                done = callback(data_[node.elemIdx]);
                return;
            }
            else
            {
                if (node.leftChildIdx.has_value() && nodes_[*node.leftChildIdx].aabb.Touches(bb))
                {
                    Hit(nodes_[*node.leftChildIdx], bb, done, callback);
                }
                if (node.rightChildIdx.has_value() && nodes_[*node.rightChildIdx].aabb.Touches(bb))
                {
                    Hit(nodes_[*node.rightChildIdx], bb, done, callback);
                }
            }
        }

        template<typename Iterator>
        void FillInTree(Node& currNode, Iterator  startIter, Iterator endIter)
        {
            const size_t numElements = endIter - startIter;
            if (numElements == 1)
            {
                currNode.aabb     = startIter->bbox;
                currNode.elemIdx = startIter->idx;
                return;
            }

            for_each(startIter, endIter, [&](const ElementIdWBBox& itElement) { currNode.aabb |= itElement.bbox; });
            const double splitPos = currNode.aabb.Center(currNode.axis);

            auto midIter = std::partition(startIter, endIter, [&](const ElementIdWBBox& itElement) { return itElement.bbox.Center(currNode.axis) < splitPos; });

            if (midIter == startIter || midIter == endIter)
            {
                midIter = startIter + numElements / 2;
            }

            auto GenerateNewNode = [this, &currNode]()
            {
                Node& newNode = nodes_.emplace_back(Node{});
                newNode.axis    = (currNode.axis + 1) % DIM;
                newNode.nodeIdx = nodes_.size() - 1;

                return newNode.nodeIdx;
            };

            currNode.leftChildIdx  = GenerateNewNode();
            currNode.rightChildIdx = GenerateNewNode();

            FillInTree(nodes_[*currNode.leftChildIdx], startIter, midIter);
            FillInTree(nodes_[*currNode.rightChildIdx], midIter, endIter);
        }

        std::vector<Node>         nodes_;
        std::vector<T> data_;
        std::vector<ElementIdWBBox> indexedBoundingBoxes_;

        std::optional<Node> root_;

    public:
        TreeXY() = default;
        // boundingBoxFunc should take a const reference to T and return a BoundingBox
        // Note: T must be hashable and == comparable
        template <class F>
        TreeXY(const std::vector<T>& data, F boundingBoxFunc) : data_(data)
        {
            if (data.empty())
                return;

            // preallocate memory for nodes and box cache
            const size_t numElements = data.size();
            nodes_.reserve(numElements * 2);
            indexedBoundingBoxes_.reserve(numElements);

            // store bounding boxes of elements together with index of the element
            for_each(data.begin(), data.end(), [&](const T& element){indexedBoundingBoxes_.push_back({indexedBoundingBoxes_.size(), boundingBoxFunc(element)});});

            // start creating tree
            root_ = nodes_.emplace_back(Node{});
            FillInTree(*root_, indexedBoundingBoxes_.begin(), indexedBoundingBoxes_.end());
        }
        ~TreeXY() = default;

        // F 'callback' should take a const reference to T and return a boolean (false - continue, true - stop search)
        template <typename F>
        void PointIntersects(const KdTree::Point& pp, F intersectedCallback) const
        {
            const KdTree::BoundingBox bb{pp, pp};
            BoxOverlaps(bb, intersectedCallback);
        }

        template <typename F>
        void BoxOverlaps(const KdTree::BoundingBox& bBox, F overlappedCallback) const
        {
            if (!root_.has_value())
            {
                return;
            }

            bool done = false;

            Hit(*root_, bBox, done, overlappedCallback);
        }

        size_t GetElementsCount() const 
        {
            return data_.size();
        }
    };
} // namespace KdTree

namespace std
{
    // Custom hash function for BBoxWithRoadPtr
    template <>
    struct hash<KdTree::Point>
    {
        std::size_t operator()(const KdTree::Point& p) const
        {
            std::size_t h1 = std::hash<double>()(p.x);
            std::size_t h2 = std::hash<double>()(p.y);
            return h1 ^ (h2 << 1);  // Combine the hashes
        }
    };

    template <>
    struct hash<KdTree::BoundingBox>
    {
        std::size_t operator()(const KdTree::BoundingBox& b) const
        {
            std::size_t h1 = std::hash<KdTree::Point>()(b.min);
            std::size_t h2 = std::hash<KdTree::Point>()(b.max);
            return h1 ^ (h2 << 1);  // Combine the hashes
        }
    };
}