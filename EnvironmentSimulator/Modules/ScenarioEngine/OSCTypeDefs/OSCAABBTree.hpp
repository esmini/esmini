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

#pragma once
#include "RoadManager.hpp"
#include <memory>
#include <vector>

namespace aabbTree
{

    using namespace roadmanager;
    using std::shared_ptr;
    using std::vector;

    class Point;
    class Triangle;
    class BBox;
    class Tree;

    typedef shared_ptr<Triangle> ptTriangle;
    typedef shared_ptr<BBox>     ptBBox;
    typedef shared_ptr<Tree>     ptTree;

    typedef vector<ptBBox> BBoxVec;
    typedef vector<Point>  Solutions;

    typedef struct
    {
        double                SMjA;
        double                SMnA;
        roadmanager::Position egoPos;
    } EllipseInfo;

    class Point
    {
    public:
        double x, y, h;
        Point(double x_, double y_) : x(x_), y(y_){};
        Point(double x_, double y_, double h_) : x(x_), y(y_), h(h_){};
        Point(Point const &pt) : x(pt.x), y(pt.y), h(pt.h){};
        Point &operator=(const Point &) = default;
        Point() : x(0), y(0), h(0){};
    };

    class Triangle
    {
    public:
        double sI, sF;
        Point  a, b, c;
        Triangle() : sI(0), sF(0), geometry_(nullptr)
        {
        }
        Triangle(Geometry *geometry) : geometry_(geometry)
        {
        }
        ~Triangle()
        {
        }
        Geometry *geometry()
        {
            return geometry_;
        }
        bool collide(ptTriangle const triangle) const;

    private:
        Geometry *geometry_;

        Triangle(Triangle const &triangle);
    };

    class BBox
    {
    public:
        BBox(ptTriangle triangle);
        BBox(BBoxVec const &bboxes);
        BBox(BBoxVec::const_iterator start, BBoxVec::const_iterator end);
        ~BBox()
        {
        }
        Point blhCorner() const
        {
            return blhc_;
        }
        Point urhCorner() const
        {
            return urhc_;
        }
        ptTriangle triangle()
        {
            return triangle_;
        }
        inline bool collide(ptBBox const bbox) const;
        bool        collide(BBox const &bbox) const;
        double inline midPointX() const;
        double inline midPointY() const;

    private:
        Point      blhc_, urhc_;
        ptTriangle triangle_;

        void merge(BBoxVec::const_iterator start, BBoxVec::const_iterator end);
    };

    struct CandidateStruct;
    typedef struct CandidateStruct
    {
        ptBBox bbox1;
        ptBBox bbox2;
        CandidateStruct(ptBBox bbx1, ptBBox bbx2) : bbox1(bbx1), bbox2(bbx2)
        {
        }
    } Candidate;

    typedef vector<Candidate> Candidates;

    class Tree
    {
    public:
        Tree() : bbox(nullptr), nodeCount_(0)
        {
        }
        ~Tree();
        void                  intersect(Tree const &tree, Candidates &candidates) const;
        void                  build(BBoxVec &bboxes);
        bool                  empty();
        vector<ptTree> const &Children() const
        {
            return childeren;
        }
        ptBBox BBox() const
        {
            return bbox;
        }
        unsigned long nodeCount() const
        {
            return nodeCount_;
        }
        unsigned long leafCount() const
        {
            return leafCount_;
        }

    private:
        ptBBox         bbox;
        vector<ptTree> childeren;
        unsigned long  nodeCount_, leafCount_;  // for debug;

        Tree(Tree const &tree);
        BBoxVec::iterator divide(BBoxVec::iterator const start, BBoxVec::iterator const end, ptBBox bbox);
        void              __build(BBoxVec::iterator const start, const BBoxVec::iterator end);
        void              __build(BBoxVec &bboxes);

        typedef struct
        {
            BBoxVec::iterator first, last;
            ptTree            tree;
        } StackRecord;
    };

    void   processCandidates(Candidates const &candidates, vector<ptTriangle> &solutions);
    void   findPoints(vector<ptTriangle> const &triangles, EllipseInfo &eInfo, Solutions &points);
    void   curve2triangles(Geometry *geometry, double segmSize, double maxAngle, BBoxVec &vec);
    ptBBox makeTriangleAndBbx(double x0, double y0, double x1, double y1, double x2, double y2, Geometry *gm = nullptr, double s0 = 0, double s1 = 0);
}  // namespace aabbTree