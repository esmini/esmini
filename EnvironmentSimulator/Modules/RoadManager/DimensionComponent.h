#pragma once

#include <cmath>
#include "CommonMini.hpp"

namespace esmini
{

    class DimensionComponent
    {
    public:
        // Constructor
        DimensionComponent(double value = std::nan("")) : value_(value)
        {
        }

        // Returns value if present, otherwise 0
        double Get() const
        {
            if (IsSet())
            {
                return value_;
            }
            return 0;
        }

        // Sets value
        void Set(double value)
        {
            value_ = value;
        }

        // Sets the value only if it was not already set and returns true, otherwise false
        bool SetIfNot(double value)
        {
            if (!IsSet())
            {
                value_ = value;
                return true;
            }
            return false;
        }

        // Returns true if user has set the value
        bool IsSet() const
        {
            return (!std::isnan(value_));
        }

        // private data
    private:
        double value_ = std::nan("");

    };  // class DimensionComponent

}  // namespace esmini