/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#include "OSCAABBTree.hpp"
#include "OSCTriangle2D.hpp"
#include "RoadManager.hpp"
#include "OSCSwarmTrafficGeometry.hpp"
#include <cmath>
#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>

using namespace aabbTree;
using roadmanager::Geometry;
using std::make_shared;
using std::max;
using std::min;
using triangle2D::overlap2d;
using namespace STGeometry;

/******************************************
 *  _____     _                   _       *
 * |_   _| __(_) __ _ _ __   __ _| | ___  *
 *   | || '__| |/ _` | '_ \ / _` | |/ _ \ *
 *   | || |  | | (_| | | | | (_| | |  __/ *
 *   |_||_|  |_|\__,_|_| |_|\__, |_|\___| *
 *                          |___/         *
 *****************************************/

bool Triangle::collide(ptTriangle const triangle) const
{
    Triangle const &tr = *triangle;
    return overlap2d(a, b, c, tr.a, tr.b, tr.c);
}

/***************************
 *  ____  ____             *
 * | __ )| __ )  _____  __ *
 * |  _ \|  _ \ / _ \ \/ / *
 * | |_) | |_) | (_) >  <  *
 * |____/|____/ \___/_/\_\ *
 **************************/

BBox::BBox(ptTriangle triangle) : triangle_(triangle)
{
    Point a = triangle->a;
    Point b = triangle->b;
    Point c = triangle->c;
    blhc_.x = min(a.x, min(b.x, c.x));
    urhc_.x = max(a.x, max(b.x, c.x));
    blhc_.y = min(a.y, min(b.y, c.y));
    urhc_.y = max(a.y, max(b.y, c.y));
}

BBox::BBox(BBoxVec const &bboxes)
{
    merge(bboxes.begin(), bboxes.end());
}

BBox::BBox(BBoxVec::const_iterator start, BBoxVec::const_iterator end)
{
    triangle_ = nullptr;
    merge(start, end);
}

inline bool BBox::collide(ptBBox const bbox) const
{
    return collide(*bbox);
}

bool BBox::collide(BBox const &bbox) const
{
    Point const &urhc = bbox.urhCorner();
    Point const &blhc = bbox.blhCorner();

    return !((urhc_.x < blhc.x) || (blhc_.x > urhc.x) || (urhc_.y < blhc.y) || (blhc_.y > urhc.y));
}

double inline BBox::midPointX() const
{
    return (urhc_.x + blhc_.x) / 2;
}

double inline BBox::midPointY() const
{
    return (urhc_.y + blhc_.y) / 2;
}

void BBox::merge(BBoxVec::const_iterator start, BBoxVec::const_iterator end)
{
    BBoxVec::const_iterator it   = start;
    double                 &xmin = blhc_.x;
    double                 &xmax = urhc_.x;
    double                 &ymin = blhc_.y;
    double                 &ymax = urhc_.y;

    xmin = (*it)->blhCorner().x;
    ymin = (*it)->blhCorner().y;
    xmax = (*it)->urhCorner().x;
    ymax = (*it)->urhCorner().y;

    while (end > ++it)
    {
        xmin = min(xmin, (*it)->blhCorner().x);
        ymin = min(ymin, (*it)->blhCorner().y);
        xmax = max(xmax, (*it)->urhCorner().x);
        ymax = max(ymax, (*it)->urhCorner().y);
    }
}

/************************
 *  _____               *
 * |_   _| __ ___  ___  *
 *   | || '__/ _ \/ _ \ *
 *   | || | |  __/  __/ *
 *   |_||_|  \___|\___| *
 ***********************/

Tree::~Tree()
{
    childeren.clear();
    bbox.reset();
}

/*
 * Builds the AABB tree structure. Uses an external function.
 * If the number of nodes should be greater equal than the number
 * of bounding boxes.
 */
void Tree::build(BBoxVec &bboxes)
{
    childeren.clear();
    if (bboxes.empty())
        return;
    nodeCount_ = 1;
    leafCount_ = 0;
    __build(bboxes.begin(), bboxes.end());
}

bool Tree::empty()
{
    return (!bbox && childeren.empty());
}

/*
 * This implements the construction of an AABB Tree.
 * It exploits an iterative algorithm instead of a recursive one
 * to handle huge networks.
 */
void Tree::__build(BBoxVec::iterator const start, BBoxVec::iterator const end)
{
    vector<StackRecord> stack;
    stack.reserve(static_cast<unsigned int>((log(end - start) * 2)));
    stack.clear();

    BBoxVec::iterator cut;
    BBoxVec::iterator first = start;
    BBoxVec::iterator last  = end;

    ptTree currentTree_ = nullptr;

    auto currentTree = [&currentTree_, this]() -> Tree &
    {
        if (currentTree_)
            return *currentTree_;
        else
            return *this;
    };

    while (last > first)
    {
        if (last - first == 1)
        {
            currentTree().bbox = *first;
            leafCount_++;
            if (!stack.empty())
            {
                auto record = stack.back();
                stack.pop_back();
                first        = record.first;
                last         = record.last;
                currentTree_ = record.tree;
            }
            else
                break;
        }
        else
        {
            ptBBox tmpBbox     = make_shared<aabbTree::BBox>(first, last);
            currentTree().bbox = tmpBbox;
            cut                = divide(first, last, tmpBbox);
            ptTree pos         = make_shared<Tree>();
            ptTree neg         = make_shared<Tree>();

            if (cut - first > 0 && last - cut > 0)
            {
            kids:
                currentTree().childeren.push_back(pos);
                currentTree().childeren.push_back(neg);
                nodeCount_ += 2;
                StackRecord record = {cut, last, neg};
                stack.push_back(record);
                currentTree_ = pos;
                last         = cut;
            }
            else
            {
                cut = first + (last - first) / 2;
                goto kids;
            }
        }
    }
}

/*
 * It separates the array in two parts according to a partition
 * line of the current bounding box.
 * It returns a pointer to the first element of the right side of the partition
 */
BBoxVec::iterator Tree::divide(BBoxVec::iterator const start, BBoxVec::iterator const end, ptBBox bboxTmp)
{
    if (end - start == 0)
        return start;
    if (end - start == 1)
        return end;

    std::function<bool(ptBBox)> compare;
    double                      mid;
    Point                       blhc = bboxTmp->blhCorner();
    Point                       urhc = bboxTmp->urhCorner();

    if (urhc.x - blhc.x > urhc.y - blhc.y)
    {
        mid     = (urhc.x + blhc.x) / 2.0;
        compare = [mid](ptBBox bbx) { return bbx->midPointX() < mid; };
    }
    else
    {
        mid     = (urhc.y + blhc.y) / 2.0;
        compare = [mid](ptBBox bbx) { return bbx->midPointY() < mid; };
    }

    auto first = start;
    auto last  = end;
    while (true)
    {
        while (first < last && compare(*first))
        {
            first++;
        }
        last--;
        while (first < last && !compare(*last))
            last--;
        if (!(first < last))
            return first;
        std::iter_swap(first, last);
        first++;
    }
}

/*
 * It Intersects two trees and puts the possible candidate bounding boxes in
 * a vector. This function has been adapted from:
 *   https://github.com/ebertolazzi/Clothoids/blob/master/src/AABBtree.cc
 *
 */
void Tree::intersect(Tree const &tree, Candidates &candidates) const
{
    if (!bbox || !tree.BBox() || !tree.BBox()->collide(bbox))
        return;

    int case_ = (this->childeren.empty() ? 0 : 1) + (tree.Children().empty() ? 0 : 2);

    switch (case_)
    {
        case 0:
        {  // Leaf & Leaf
            candidates.push_back(Candidate(bbox, tree.BBox()));
            break;
        }
        case 1:
        {  // Tree & Leaf
            for (ptTree const &child : childeren)
            {
                child->intersect(tree, candidates);
            }
            break;
        }
        case 2:
        {  // Leaf & Tree
            for (ptTree const &child : tree.Children())
            {
                intersect(*child, candidates);
            }
            break;
        }
        case 3:
        {  // Tree & Tree
            for (ptTree const &child1 : childeren)
            {
                for (ptTree const &child2 : tree.Children())
                    child1->intersect(*child2, candidates);
            }
            break;
        }
    }
}

/***********************
 *  _   _ _   _ _      *
 * | | | | |_(_) |___  *
 * | | | | __| | / __| *
 * | |_| | |_| | \__ \ *
 *  \___/ \__|_|_|___/ *
 ***********************/

aabbTree::ptBBox aabbTree::makeTriangleAndBbx(double x0, double y0, double x1, double y1, double x2, double y2, Geometry *gm, double s0, double s1)
{
    ptTriangle triangle = make_shared<Triangle>(gm);
    triangle->a         = Point(x0, y0);
    triangle->b         = Point(x1, y1);
    triangle->c         = Point(x2, y2);
    triangle->sI        = s0;
    triangle->sF        = s1;
    return make_shared<BBox>(triangle);
}

void aabbTree::processCandidates(Candidates const &candidates, vector<ptTriangle> &solutions)
{
    for (auto const &candidate : candidates)
    {
        ptTriangle const tr1 = candidate.bbox1->triangle();
        ptTriangle const tr2 = candidate.bbox2->triangle();
        if (tr1->collide(tr2))
        {
            if (tr2->geometry())
                solutions.push_back(tr2);
            else
                solutions.push_back(tr1);
        }
    }
}

void aabbTree::findPoints(vector<ptTriangle> const &triangles, EllipseInfo &eInfo, Solutions &points)
{
    for (auto const &tr : triangles)
    {
        if (tr->geometry())
        {
            geometryIntersect(*tr, eInfo, points);
        }
        else
            LOG("Warning: triangle without a geometry found");
    }
}

void aabbTree::curve2triangles(Geometry *geometry, double segmSize, double maxAngle, BBoxVec &vec)
{
    double s0     = 0;
    double length = geometry->GetLength();
    double ds     = min(segmSize, length);
    while (s0 < length)
    {
        double x0, y0, t0, x1, y1, t1, x2, y2;
        int    count = 0;
        geometry->EvaluateDS(s0, &x0, &y0, &t0);

        double s1 = min(s0 + ds, length);

    s_1:
        geometry->EvaluateDS(s1, &x1, &y1, &t1);
        if (abs(t1 - t0) > maxAngle && count < 4)
        {
            double dt = abs(t1 - t0) / (s1 - s0);
            s1        = s0 + maxAngle / dt;
            count++;
            goto s_1;
        }

        tangentIntersection(x0, y0, s0, t0, x1, y1, s1, t1, x2, y2);

        ptBBox bbx = makeTriangleAndBbx(x0, y0, x1, y1, x2, y2, geometry, s0, s1);
        vec.push_back(bbx);
        s0 = s1;
    }
}
