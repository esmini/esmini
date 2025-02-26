#include "Config.hpp"

#include "ConfigParser.hpp"
#include "logger.hpp"
#include "DefaultPathFinder.hpp"
#include "Defines.hpp"
#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#error "Missing <filesystem> header"
#endif

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
    }

    std::vector<std::string> Config::GetConfig() const
    {
        esmini::common::ConfigParser parser(applicationName_, configFilePaths_);
        return parser.Parse();
    }

    std::string Config::MakeDefaultConfigFilePath() const
    {
        esmini::common::DefaultPathFinder pathFinder;
        fs::path                          defaultPath(pathFinder.GetDefaultPath());
        std::string                       defaultFilePath = (defaultPath.parent_path() / ".." / DEFAULT_CONFIG_FILE).string();

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

    const std::vector<std::string>& Config::GetFilePaths() const
    {
        return configFilePaths_;
    }
}  // namespace esmini::common