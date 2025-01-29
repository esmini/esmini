#pragma once

#include <string>
#include <optional>
#include <vector>

namespace esmini::common
{
    class Config
    {
    public:
        static Config& Inst();

        Config(Config const&)         = delete;
        void operator=(Config const&) = delete;

        // private interface
    private:
        // Singleton constructor
        Config();
        //  Read environment variables
        std::optional<std::string> GetEnvironmentVariable(const std::string& variableName);

        // private data members
    private:
        std::vector<std::string> configFilePaths_;
    };
}  // namespace esmini::common
