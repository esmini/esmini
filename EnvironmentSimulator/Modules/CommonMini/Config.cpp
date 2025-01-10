#include "Config.hpp"

#include "ConfigParser.hpp"
#include "logger.hpp"
#include "DefaultPathFinder.hpp"
#include "Defines.hpp"

namespace esmini::common
{
    Config::Config(const std::string& applicationName) : applicationName_(applicationName)
    {
        configFilePaths_.push_back(MakeDefaultConfigFilePath());

        if (const auto envConfigFile = GetEnvironmentVariable("ESMINI_CONFIG_FILE"); envConfigFile.has_value())
        {
            LOG_INFO("Found environment variable ESMINI_CONFIG_FILE: {}", envConfigFile.value());
            const auto valueVec = SplitString(envConfigFile.value(), ',');
            configFilePaths_.insert(configFilePaths_.end(), std::make_move_iterator(valueVec.begin()), std::make_move_iterator(valueVec.end()));
        }
        else
        {
            std::cout << "Environment variable ESMINI_CONFIG_FILE not set" << std::endl;
        }
    }

    std::vector<std::string> Config::GetConfig() const
    {
        esmini::common::ConfigParser parser(applicationName_, configFilePaths_);
        return parser.Parse();
    }

    std::string Config::MakeDefaultConfigFilePath() const
    {
        esmini::common::DefaultPathFinder pathFinder;
        std::string                       defaultPath = pathFinder.GetDefaultPath();
        defaultPath                                   = defaultPath.substr(0, defaultPath.find_last_of("//"));
        std::string defaultFilePath                   = fmt::format("{}/{}", defaultPath, DEFAULT_CONFIG_FILE);
        LOG_INFO("Default config file path: {}", defaultFilePath);
        return defaultFilePath;
    }

    std::optional<std::string> Config::GetEnvironmentVariable(const std::string& variableName) const
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