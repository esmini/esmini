#include "ConfigParser.hpp"

#include "CommonMini.hpp"

#include "logger.hpp"
#include <iostream>

namespace esmini::common
{
    ConfigParser::ConfigParser(const std::vector<std::string>& configFilePaths) : configFilePaths_(configFilePaths)
    {
    }

    //---------------------------------------------------------------------------------------------------------------------

    void ConfigParser::ParseNode(const YAML::Node& node, const std::string& prefix)
    {
        for (auto& it : node)
        {
            std::string app       = prefix.empty() ? it.first.as<std::string>() : prefix;
            std::string key       = it.first.as<std::string>();
            ConfigMap&  configMap = appsConfig_[app];

            if (it.second.IsMap())
            {
                // Recursively parse nested maps
                ParseNode(it.second, key);
            }
            else if (it.second.IsSequence())
            {
                // Flatten sequences into a single comma-separated string
                std::string value;
                for (size_t i = 0; i < it.second.size(); ++i)
                {
                    if (i > 0)
                        value += ",";
                    value += it.second[i].as<std::string>();
                }
                PutValue(app, key, value);
                // configMap[key] = value;
            }
            else
            {
                // Store scalar values
                PutValue(app, key, it.second.as<std::string>());
                // configMap[key] = it.second.as<std::string>();
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------

    void ConfigParser::PutValue(const std::string& app, const std::string& key, const std::string& value)
    {
        ConfigMap& configMap = appsConfig_[app];
        const auto valueVec  = SplitString(value, ' ');
        auto&      configVec = configMap[key];
        configVec.push_back(fmt::format("--{}", key));
        configVec.insert(configVec.end(), std::make_move_iterator(valueVec.begin()), std::make_move_iterator(valueVec.end()));
    }

    //---------------------------------------------------------------------------------------------------------------------

    void ConfigParser::LogAllAppsConfig() const
    {
        for (const auto& [app, configMap] : appsConfig_)
        {
            std::cout << std::endl << "Application: " << app << std::endl;
            for (const auto& [key, value] : configMap)
            {
                std::cout << key << " : " << std::endl;
                for (const auto& val : value)
                {
                    std::cout << val << " " << std::endl;
                }
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------

    void ConfigParser::LogOneAppConfig(const std::string& app) const
    {
        if (const auto& it = appsConfig_.find(app); it == appsConfig_.end())
        {
            std::cout << "Config for application: " << app << " not found" << std::endl;
            return;
        }
        else
        {
            std::cout << std::endl << "Application: " << app << std::endl;
            for (const auto& [key, value] : it->second)
            {
                std::cout << key << " : " << std::endl;
                for (const auto& val : value)
                {
                    std::cout << val << " " << std::endl;
                }
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------

    ApplicationsConfigMap ConfigParser::Parse()
    {
        try
        {
            for (const auto& configFilePath : configFilePaths_)
            {
                LOG_INFO("Parsing config file: {}", configFilePath);
                YAML::Node config = YAML::LoadFile(configFilePath);
                ParseNode(config, "");
            }
        }
        catch (const YAML::Exception& ex)
        {
            std::cerr << "Error parsing YAML file: " << ex.what() << std::endl;
        }
        LogAllAppsConfig();
        return appsConfig_;
    }

    //---------------------------------------------------------------------------------------------------------------------

    ConfigMap ConfigParser::GetApplicationConfig(const std::string& application) const
    {
        auto it = appsConfig_.find(application);
        if (it != appsConfig_.end())
        {
            LogOneAppConfig(application);
            return it->second;
        }
        return ConfigMap();
    }
}  // namespace esmini::common