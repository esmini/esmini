#include "EnumConfig.hpp"
#include "logger.hpp"

using namespace esmini_options;

CONFIG_ENUM esmini_options::ConvertStrKeyToEnum(const std::string& key)
{
    if (auto itr = configStrKeyEnumMap.find(key); itr == configStrKeyEnumMap.end())
    {
        // the option can be for viewer stuff
        return CONFIGS_COUNT;
    }
    else
    {
        return itr->second;
    }
}