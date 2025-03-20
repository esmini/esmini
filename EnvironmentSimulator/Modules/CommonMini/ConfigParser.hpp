#pragma once

#include "yaml.hpp"
#include <vector>
#include <string>

namespace esmini::common
{
    class ConfigParser
    {
    public:
        // constructor
        ConfigParser(const std::string&                                applicationName,
                     const std::vector<std::string>&                   configFilePath,
                     std::vector<std::pair<std::string, std::string>>& loadedConfigFiles);

        std::vector<std::string> Parse();
        bool                     IsFaulty() const;
        std::string              GetParsingErrorMsg() const;
        // private functions
    private:
        // Function to check if file exists and is not a directory. Also, check if the number of loaded files is not exceeding the limit
        // i.e. circular dependency checking
        void DoSanityChecksOnConfigFile(const std::string& filename);
        // Function to parse the YAML file
        void ParseYamlFile(const std::string& filename);
        // Function to recursively parse the YAML node and populate data from it
        void ParseNode(TINY_YAML::Node& node, std::string parent);
        // Function to store the key-value pair for the requested application only
        void PutValue(const std::string& app, const std::string& key, const std::string& value);

        // private data members
    private:
        // config file(s) path(s)
        std::vector<std::string> configFilePaths_;
        // application name for which we are parsing the config file i.e. esmini, replayer etc.
        std::string applicationName_;
        // container to store the config for the application
        std::vector<std::string> configs_;
        // canonical and relative paths of config files, which are successfully loaded
        std::vector<std::pair<std::string, std::string>>& loadedConfigFiles_;
        // parsing status
        bool faulty_ = false;
        // error message
        std::string parsingErrorMsg_;
    };
}  // namespace esmini::common
