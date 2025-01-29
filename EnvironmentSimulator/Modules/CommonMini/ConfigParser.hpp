#pragma once

#include "yaml-cpp/yaml.h"

#include <unordered_map>
#include <vector>
#include <string>

namespace esmini::common
{
    using ConfigMap             = std::unordered_map<std::string, std::vector<std::string>>;
    using ApplicationsConfigMap = std::unordered_map<std::string, ConfigMap>;

    class ConfigParser
    {
    public:
        // constructor
        ConfigParser(const std::vector<std::string>& configFilePath);

        // parses the config file
        ApplicationsConfigMap Parse();

        // returns config map of a particular application
        ConfigMap GetApplicationConfig(const std::string& application) const;

        // private functions
    private:
        // Function to recursively parse the YAML node and populate the map
        void ParseNode(const YAML::Node& node, const std::string& prefix);

        // Logs all applications config map
        void LogAllAppsConfig() const;

        // Logs one application config map
        void LogOneAppConfig(const std::string& app) const;

        // Function to store the app wise value in the map
        void PutValue(const std::string& app, const std::string& key, const std::string& value);

        // private data members
    private:
        // config file path
        std::vector<std::string> configFilePaths_;

        // map to store application wise config
        ApplicationsConfigMap appsConfig_;
    };
}  // namespace esmini::common
