#include "EnumConfig.hpp"
#include "logger.hpp"

CONFIG_ENUM ConvertStrKeyToEnum(const std::string& key)
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