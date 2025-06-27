#include "EnumConfig.hpp"
#include "logger.hpp"

CONFIG_ENUM ConvertStrKeyToEnum(const std::string& key)
{
    if (auto itr = configStrKeyEnumMap.find(key); itr == configStrKeyEnumMap.end())
    {
        LOG_ERROR("Unknown option:{}", key);
        throw std::invalid_argument(std::string("Unknown option: ") + key);
    }
    else
    {
        return itr->second;
    }
}