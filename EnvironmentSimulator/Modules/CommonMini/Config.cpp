#include "Config.hpp"

#include "ConfigParser.cpp"
#include "logger.hpp"

const std::string DEFAULT_CONFIG_FILE = "config.yml";

namespace esmini::common
{
    Config& Config::Inst()
    {
        static Config instance;
        return instance;
    }

    Config::Config()
    {
        configFilePaths_.push_back(DEFAULT_CONFIG_FILE);
        if (const auto envConfigFile = GetEnvironmentVariable("ESMINI_CONFIG_FILE"); envConfigFile.has_value())
        {
            LOG_INFO("Found environment variable ESMINI_CONFIG_FILE: {}", envConfigFile.value());
            const auto valueVec = SplitString(envConfigFile.value(), ',');
            configFilePaths_.insert(configFilePaths_.end(), std::make_move_iterator(valueVec.begin()), std::make_move_iterator(valueVec.end()));
        }

        esmini::common::ConfigParser parser(configFilePaths_);
        const auto                   config = parser.Parse();
    }

    std::optional<std::string> Config::GetEnvironmentVariable(const std::string& variableName)
    {
        const char* env_var = std::getenv(variableName.c_str());
        if (env_var != nullptr)
        {
            return std::string(env_var);
        }
        else
        {
            LOG_DEBUG("Environment variable: {} not set", variableName);
            return std::nullopt;
        }
    }

}  // namespace esmini::common