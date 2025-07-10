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

#include <vector>
#include <cmath>  // For std::nan

namespace roadmanager
{
    // piecewise cubic Hermite interpolating polynomial
    // interpolating tangents calculated according to Fritsch-Carlson method
    // https://en.wikipedia.org/wiki/Monotone_cubic_interpolation

    class PCHIP
    {
    public:
        PCHIP() = default;

        // Sets the x and y values, normally time (x) and distance (y)
        void SetValues(const std::vector<double>& x_pts, const std::vector<double>& y_pts);

        // Interpolation function: evaluates the curve at some given x value
        double interpolate(double x) const;

        // Specify initial slope/speed. Optional, default is slope from first to second point
        void SetInitialSlope(double initial_slope)
        {
            initial_slope_ = initial_slope;
        }

        void SetPreventOvershoot(bool prevent_overshoot)
        {
            prevent_overshoot_ = prevent_overshoot;
        }

        bool GetPreventOvershoot() const
        {
            return prevent_overshoot_;
        }

        void SetPreviousSecantMethod(bool use_previous_secant_method)
        {
            previous_secant_method_ = use_previous_secant_method;
        }

        bool GetPreviousSecantMethod() const
        {
            return previous_secant_method_;
        }

        bool IsSet() const
        {
            if (x_pts_.empty() || y_pts_.empty())
            {
                return false;
            }

            return true;
        }

    private:
        std::vector<double> x_pts_;
        std::vector<double> y_pts_;
        std::vector<double> y_prime_;  // tangent slopes
        double              initial_slope_          = std::nan("");
        bool                prevent_overshoot_      = false;
        bool                previous_secant_method_ = false;
    };

}  // namespace roadmanager
