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

#include <random>
#include <algorithm>

namespace Utils
{

    using std::uniform_int_distribution;

    template <typename __InputIterator, typename __RandomAccessIterator, typename __Number, typename __Generator>
    inline void sample(__InputIterator first, __InputIterator last, __RandomAccessIterator out, __Number n, __Generator &g)
    {
        uniform_int_distribution<__Number> dist(0, n);
        __Number                           count = 0;
        while (first < last && count < n)
        {
            out[count++] = *first;
            ++first;
        }
        for (; first < last; first++)
        {
            __Number j = dist(g);
            if (j < n)
                out[j] = *first;
        }
    }

}  // namespace Utils