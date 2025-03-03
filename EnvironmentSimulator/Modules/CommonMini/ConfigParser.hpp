#pragma once

#include "yaml.hpp"
#include <vector>
#include <string>

namespace esmini::common
{
    // using ConfigMap             = std::unordered_map<std::string, std::vector<std::string>>;
    // using ApplicationsConfigMap = std::unordered_map<std::string, ConfigMap>;

    class ConfigParser
    {
    public:
        // constructor
        ConfigParser(const std::string& applicationName, const std::vector<std::string>& configFilePath);

        // parses the config file
        std::vector<std::string> Parse();

        // returns config map of a particular application
        // const ConfigMap& GetApplicationConfig(const std::string& application) const;

        // private functions
    private:
        // Function to parse the YAML file
        void ParseYamlFile(const std::string& filename);

        // Function to recursively parse the YAML node and populate the map
        // void ParseNode(const ryml::NodeRef& node, const std::string& prefix);
        void ParseNode(TINY_YAML::Node& node, std::string parent);

        // Function to store the app wise value in the map
        void PutValue(const std::string& app, const std::string& key, const std::string& value);

        // private data members
    private:
        // config file(s) path(s)
        std::vector<std::string> configFilePaths_;
        // application name for which we are parsing the config file i.e. esmini, replayer etc.
        std::string applicationName_;
        // container to store the config for the application
        std::vector<std::string> configs_;
    };
}  // namespace esmini::common
