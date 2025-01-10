#pragma once

#include <string>
#include <optional>
#include <vector>

namespace esmini::common
{
    class Config
    {
    public:
        // constructor
        Config(const std::string& applicationName);

        // Get the config for the application
        std::vector<std::string> GetConfig() const;

        Config()                      = delete;
        Config(Config const&)         = delete;
        void operator=(Config const&) = delete;

        // private interface
    private:
        //  Read environment variables
        std::optional<std::string> GetEnvironmentVariable(const std::string& variableName) const;
        // Make default config file path
        std::string MakeDefaultConfigFilePath() const;

        // private data members
    private:
        // Config file paths
        std::vector<std::string> configFilePaths_;
        // Application for which we are parsing the config file i.e. esmini, replayer etc.
        std::string applicationName_;
    };
}  // namespace esmini::common
