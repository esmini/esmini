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

#include <algorithm>

#include "pchip.hpp"
#include "CommonMini.hpp"
#include "logger.hpp"

using namespace roadmanager;

void PCHIP::SetValues(const std::vector<double>& x_pts, const std::vector<double>& y_pts)
{
    if (x_pts.size() != y_pts.size() || x_pts.size() < 2)
    {
        LOG_ERROR_AND_QUIT("PCHIP: x and y must be vectors of the same size, with at least 2 points.");
    }

    x_pts_ = x_pts;
    y_pts_ = y_pts;

    size_t              n = x_pts_.size();
    std::vector<double> h(n - 1);
    std::vector<double> m(n - 1);

    // Calculate secant slopes (m_k)
    for (size_t i = 0; i < n - 1; ++i)
    {
        h[i] = x_pts_[i + 1] - x_pts_[i];
        m[i] = (y_pts_[i + 1] - y_pts_[i]) / h[i];
    }

    // Estimate initial tangent slopes (y_prime)
    y_prime_.resize(n);
    if (n == 2)
    {
        y_prime_[0] = m[0];
        y_prime_[1] = m[0];
    }
    else
    {
        for (size_t i = 1; i < n - 1; ++i)
        {
            if (GetPreviousSecantMethod())
            {
                // Use the slope of previous secant for interior point tangents
                y_prime_[i] = m[i - 1];
            }
            else
            {
                // Fritsch-Carlson method: For interior points, use the average of adjacent secant slopes
                y_prime_[i] = (m[i - 1] + m[i]) / 2.0;
            }
        }
        // For endpoints, use one-sided differences, except when explicit initial slope is provided
        y_prime_[0]     = std::isnan(initial_slope_) ? m[0] : initial_slope_;
        y_prime_[n - 1] = m[n - 2];
    }

    // Enforce Monotonicity Constraints
    for (size_t k = 0; k < n - 1; ++k)
    {
        if (NEAR_ZERO(m[k]))
        {
            y_prime_[k]     = 0.0;
            y_prime_[k + 1] = 0.0;
        }
        else
        {
            if (SIGN(y_prime_[k]) != SIGN(m[k]))
            {
                y_prime_[k] = 0.0;
            }
            if (SIGN(y_prime_[k + 1]) != SIGN(m[k]))
            {
                y_prime_[k + 1] = 0.0;
            }

            double alpha = y_prime_[k] / m[k];
            double beta  = y_prime_[k + 1] / m[k];

            if (GetPreventOvershoot() && alpha * alpha + beta * beta > 9.0)
            {
                LOG_DEBUG("PCHIP: Preventing overshoot in interval {}: alpha = {}, beta = {}", k, alpha, beta);
                double tau      = 3.0 / std::sqrt(alpha * alpha + beta * beta);
                y_prime_[k]     = tau * alpha * m[k];
                y_prime_[k + 1] = tau * beta * m[k];
            }
        }
    }
}

double PCHIP::interpolate(double x) const
{
    // Find which interval x belongs to
    // std::upper_bound finds the first element greater than x
    auto   it    = std::upper_bound(x_pts_.begin(), x_pts_.end(), x);
    size_t index = static_cast<unsigned int>(std::distance(x_pts_.begin(), it));

    // Handle edge cases
    if (index == 0)
    {
        index = 1;
    }
    if (index == x_pts_.size())
    {
        index = x_pts_.size() - 1;
    }

    size_t k = index - 1;

    // Evaluate the polynomial for the interval
    double x_k  = x_pts_[k];
    double x_k1 = x_pts_[k + 1];

    double y_k  = y_pts_[k];
    double y_k1 = y_pts_[k + 1];

    double yp_k  = y_prime_[k];
    double yp_k1 = y_prime_[k + 1];

    double h = x_k1 - x_k;
    double s = (x - x_k) / h;

    // Hermite basis functions
    double h00 = (2 * std::pow(s, 3)) - (3 * std::pow(s, 2)) + 1;
    double h10 = std::pow(s, 3) - (2 * std::pow(s, 2)) + s;
    double h01 = (-2 * std::pow(s, 3)) + (3 * std::pow(s, 2));
    double h11 = std::pow(s, 3) - std::pow(s, 2);

    return h00 * y_k + h10 * h * yp_k + h01 * y_k1 + h11 * h * yp_k1;
}
