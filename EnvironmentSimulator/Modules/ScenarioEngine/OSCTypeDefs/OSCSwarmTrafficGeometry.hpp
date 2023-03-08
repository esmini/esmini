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
#include "OSCAABBTree.hpp"
#include <functional>

namespace STGeometry
{

    using aabbTree::EllipseInfo;
    using aabbTree::Point;
    using aabbTree::Solutions;
    using aabbTree::Triangle;

    typedef std::function<double(double)> DDProc;

    /**
     * @brief It computer the left hand side in (x, y) of the ellipse equation in the form:
     *
     *   [(x - h)cos(A) + (y - k)sin(A)]²     [(x  - h)sin(A) - (y - k)cos(A)]²
     *  ---------------------------------- + ---------------------------------- - 1 = 0
     *                  a²                                   b²
     * Ideally, it returns n = 0 when (x,y) is on the curve, n > 0 if teh point is outside
     * the ellipse, n < 0 if the point is inside the ellipse.
     *
     * @param h x coordinate of the center
     * @param k y coordinate of the center
     * @param A Angle of rotation of the ellipse
     * @param SMjA Semi major axes
     * @param SMnA Semi minor axes
     * @param x
     * @param y
     * @returns double
     */
    double ellipse(double h, double k, double A, double SMjA, double SMnA, double x, double y);

    /**
     * @brief It calculates the points of a parametrized ellipse in the form:
     *   x = a cos(alpha) cos(theta) - b sin(alpha) sin(theta) + h
     *   y = a cos(alpha) sin(theta) - b sin(alpha) cos(theta) + k
     *
     * @param alpha Angle of parametrization [0;2pi]
     * @param h Center of the ellipse
     * @param k
     * @param SMjA Semi major axes
     * @param SMnA Semi minor axes
     * @param hdg rotation angle of the ellipse
     * @param sx x-coordinate where the solution will be stored
     * @param sy y-coordinate where the solution will be stored
     */
    void paramEllipse(double alpha, double h, double k, double SMjA, double SMnA, double hdg, double& sx, double& sy);

    /**
     * @brief It computes the angle of the tangent in the point of parametrization alpha
     *
     * @param SMjA Semi major axes
     * @param SMnA Semi minor axes
     * @param alpha Angle of parametrization [0;2pi]
     * @param hdg Angle of rotation of the ellipse
     * @return double
     */
    double angleTangentEllipse(double SMjA, double SMnA, double alpha, double hdg);

    /**
     * @brief It computes the intersection point between two tangents. Used
     * to construct the triangles for the AABB Tree algorithm
     *
     * @param x0
     * @param y0
     * @param s0
     * @param t0
     * @param x1
     * @param y1
     * @param s1
     * @param t1
     * @param x2
     * @param y2
     */
    void tangentIntersection(double x0, double y0, double s0, double t0, double x1, double y1, double s1, double t1, double& x2, double& y2);

    /*
     * Line considered in the form y = mx + q
     * @m: angular coefficient of the line
     * @hdg: orientation of ellipses
     * @SMjA semi-major axes
     * @SMnA semi-minor axes
     */
    double A(double m, double hdg, double SMjA, double SMnA);

    /*
     * Line considered in the form y = mx + q, where m = tan(theta)
     * @m: angular coefficient of the line
     * @q: y intercept of the line
     * @h: x-position of the center of the ellipses
     * @k: y-position of the center of the ellipses
     * @hdg: orientation of ellipses
     * @SMjA semi-major axes
     * @SMnA semi-minor axes
     */
    double B(double m, double q, double h, double k, double hdg, double SMjA, double SMnA);

    /*
     * Line considered in the form y = mx + q, where m = tan(hdg)
     * @q: y intercept of the line
     * @h: x-position of the center of the ellipses
     * @k: y-position of the center of the ellipses
     * @hdg: orientation of ellipses
     * @SMnA: semi-minor axes
     * @SMjA semi-major axes
     */
    double C(double q, double h, double k, double hdg, double SMjA, double SMnA);

    double vA(double e_hdg, double SMjA);

    double vB(double x, double h, double k, double e_hdg, double SMjA);

    double vC(double x, double h, double k, double e_hdg, double SMjA, double SMnA);

    /*
     * Checks whether the intersection points found belong to the segment of road
     */
    void checkRange(aabbTree::Triangle& triangle, Solutions& sols, size_t pos);

    /**
     * @brief Finds the zeros of a function 'f' given the guess
     * interval (a, b) and a tollerance 'delta'. The result
     * is saved in 'res' and the function returns true if a solution is found
     *
     */
    bool brent_zeros(double a, double b, double& res, double delta, DDProc f);

    /**
     * @brief It computes the intersection between a road geometry and the ellipse
     * using numerical methods.
     *
     * @param triangle Triangle containing the geometry info and segment
     * @param eInfo Struct containing the info of the ellipse
     * @param sol Vector on which the solutions will be stored
     * @return true if a solution is found
     * @return false if no solutions have been found
     */
    bool geometryIntersect(Triangle& triangle, EllipseInfo& eInfo, Solutions& sol);

}  // namespace STGeometry