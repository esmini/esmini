#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace utils
{

    static std::vector<std::string> SplitString(const std::string &str, char delimiter)
    {
        if (str.empty())
        {
            return {};
        }
        std::vector<std::string> result;
        size_t                   start = 0, end = 0;

        while ((end = str.find(delimiter, start)) != std::string_view::npos)
        {
            result.emplace_back(str.substr(start, end - start));
            start = end + 1;
        }

        // Add the last segment
        result.emplace_back(str.substr(start));

        return result;
    }

}  // namespace utils
