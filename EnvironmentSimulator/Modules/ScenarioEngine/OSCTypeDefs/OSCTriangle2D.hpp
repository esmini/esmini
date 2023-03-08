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
/*
 *  This file contains C implementation of algorithms for
 *  performing two-dimensional triangle-triangle intersection test
 *  The algorithms and underlying theory are described in
 *
 *  "Fast and Robust Triangle-Triangle Overlap Test Using Orientation Predicates"
 *  P. Guigue - O. Devillers
 *
 *  Journal of Graphics Tools, 8(1), 2003
 *
 *  A geometric predicates is defined.  Their parameters are all
 *  points.  Each point is a dedicated class containing the 2D coordinates.
 *  The geometric predicates implemented in this file are:
 *
 *    bool overlap2d(a1, b1, c1, a2, b2, c2)
 *
 *
 *    each function returns 1 if the triangles (including their
 *    boundary) intersect, otherwise 0
 *
 *
 *  Other information are available from the Web page
 *  https://www.acm.org/jgt/papers/GuigueDevillers03/
 *
 * The following algorithm has been adapted from
 *   https://github.com/ebertolazzi/Clothoids/blob/master/src/Triangle2D.cc
 *
 */

#pragma once
#include <cmath>
#include "OSCAABBTree.hpp"

namespace triangle2D
{

    using aabbTree::Point;

    static inline double orientation(Point const a, Point const b, Point const c)
    {
        return ((a.x - c.x) * (b.y - c.y)) - ((a.y - c.y) * (b.x - c.x));
    }

    static inline bool intersection_test_vertex(Point const a1,
                                                Point const b1,
                                                Point const c1,

                                                Point const a2,
                                                Point const b2,
                                                Point const c2)
    {
        if (orientation(c2, a2, b1) >= 0)
        {
            if (orientation(c2, b2, b1) <= 0)
            {
                if (orientation(a1, a2, b1) > 0)
                    return orientation(a1, b2, b1) <= 0;
                else
                    return orientation(a1, a2, c1) >= 0 && orientation(b1, c1, a2) >= 0;
            }
            else
                return orientation(a1, b2, b1) <= 0 && orientation(c2, b2, c1) <= 0 && orientation(b1, c1, b2) >= 0;
        }
        else
        {
            if (orientation(c2, a2, c1) >= 0)
            {
                if (orientation(b1, c1, c2) >= 0)
                    return orientation(a1, a2, c1) >= 0;
                else
                    return orientation(b1, c1, b2) >= 0 && orientation(c2, c1, b2) >= 0;
            }
            else
                return false;
        }
    }

    static inline bool intersection_test_edges(Point const a1,
                                               Point const b1,
                                               Point const c1,

                                               Point const a2,
                                               Point const c2)
    {
        if (orientation(c2, a2, b1) >= 0)
        {
            if (orientation(a1, a2, b1) >= 0)
                return orientation(a1, b1, c2) >= 0;
            else
                return orientation(b1, c1, a2) >= 0 && orientation(c1, a1, a2) >= 0;
        }
        else if (orientation(c2, a2, c1) >= 0)
            return orientation(a1, a2, c1) >= 0 && (orientation(a1, c1, c2) >= 0 || orientation(b1, c1, c2) >= 0);
        return false;
    }

    static inline bool intersection2d(Point const a1,
                                      Point const b1,
                                      Point const c1,

                                      Point const a2,
                                      Point const b2,
                                      Point const c2)
    {
        if (orientation(a2, b2, a1) >= 0)
        {
            if (orientation(b2, c2, a1) >= 0)
                return orientation(c2, a2, a1) >= 0 || intersection_test_edges(a1, b1, c1, a2, c2);
            else
            {
                if (orientation(c2, a2, a1) >= 0)
                    return intersection_test_edges(a1, b1, c1, c2, b2);
                else
                    return intersection_test_vertex(a1, b1, c1, a2, b2, c2);
            }
        }
        else
        {
            if (orientation(b2, c2, a1) >= 0)
            {
                if (orientation(c2, a2, a1) >= 0)
                    return intersection_test_edges(a1, b1, c1, b2, c2);
                else
                    return intersection_test_vertex(a1, b1, c1, b2, c2, a2);
            }
            else
                return intersection_test_vertex(a1, b1, c1, c2, a2, b2);
        }
    }

    static inline bool overlap2d(Point const a1,
                                 Point const b1,
                                 Point const c1,

                                 Point const a2,
                                 Point const b2,
                                 Point const c2)
    {
        if (orientation(a1, b1, c1) < 0)
        {
            if (orientation(a2, b2, c2) < 0)
                return intersection2d(a1, c1, b1, a2, c2, b2);
            else
                return intersection2d(a1, c1, b1, a2, b2, c2);
        }
        else
        {
            if (orientation(a2, b2, c2) < 0)
                return intersection2d(a1, b1, c1, a2, c2, b2);
            else
                return intersection2d(a1, b1, c1, a2, b2, c2);
        }
    }
}  // namespace triangle2D