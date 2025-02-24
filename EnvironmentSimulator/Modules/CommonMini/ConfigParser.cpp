#include "ConfigParser.hpp"

#include "CommonMini.hpp"
#include "logger.hpp"
#include "Defines.hpp"

#include <iostream>

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
    ConfigParser::ConfigParser(const std::string& applicationName, const std::vector<std::string>& configFilePaths)
        : applicationName_(applicationName),
          configFilePaths_(configFilePaths)
    {
    }

    //---------------------------------------------------------------------------------------------------------------------

    // void ConfigParser::PutValue(const std::string& app, const std::string& key, const std::string& value)
    // {
    //     ConfigMap& configMap = appsConfig_[app];
    //     const auto valueVec  = SplitString(value, ' ');
    //     auto&      configVec = configMap[key];
    //     configVec.push_back(fmt::format("--{}", key));
    //     configVec.insert(configVec.end(), std::make_move_iterator(valueVec.begin()), std::make_move_iterator(valueVec.end()));
    // }

    //---------------------------------------------------------------------------------------------------------------------

    // void ConfigParser::LogAllAppsConfig() const
    // {
    //     for (const auto& [app, configMap] : appsConfig_)
    //     {
    //         std::cout << std::endl << "Application: " << app << std::endl;
    //         for (const auto& [key, value] : configMap)
    //         {
    //             std::cout << key << " : " << std::endl;
    //             for (const auto& val : value)
    //             {
    //                 std::cout << val << " " << std::endl;
    //             }
    //         }
    //     }
    // }

    //---------------------------------------------------------------------------------------------------------------------

    // void ConfigParser::LogOneAppConfig(const std::string& app) const
    // {
    //     if (const auto& it = appsConfig_.find(app); it == appsConfig_.end())
    //     {
    //         std::cout << "Config for application: " << app << " not found" << std::endl;
    //         return;
    //     }
    //     else
    //     {
    //         std::cout << std::endl << "Application: " << app << std::endl;
    //         for (const auto& [key, value] : it->second)
    //         {
    //             std::cout << key << " : " << std::endl;
    //             for (const auto& val : value)
    //             {
    //                 std::cout << val << " " << std::endl;
    //             }
    //         }
    //     }
    // }

    //---------------------------------------------------------------------------------------------------------------------

    void ConfigParser::LogConfig() const
    {
        for (const auto& config : configs_)
        {
            std::cout << config << std::endl;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------

    void ConfigParser::PutValue(const std::string& app, const std::string& key, const std::string& value)
    {
        if (app == applicationName_)
        {
            // There is a possibility that the config file contains another config file's reference
            // In that case, we need to parse the referenced file as well and put in the configs_
            // exactly in the place where the reference was found, so the order of the configs_ is preserved
            if (key == CONFIG_FILE_OPTION_NAME && !value.empty())
            {
                ConfigParser anotherFileParser(applicationName_, {value});
                const auto   anotherConfigs = anotherFileParser.Parse();
                configs_.insert(configs_.end(), std::make_move_iterator(anotherConfigs.begin()), std::make_move_iterator(anotherConfigs.end()));
                return;
            }

            configs_.push_back(fmt::format("--{}", key));
            const auto valueVec = SplitString(value, ' ');
            configs_.insert(configs_.end(), std::make_move_iterator(valueVec.begin()), std::make_move_iterator(valueVec.end()));
        }
    }

    //---------------------------------------------------------------------------------------------------------------------

    void ConfigParser::ParseNode(TINY_YAML::Node& node, std::string parent)
    {
        if (node.getSize() == 0)  // no children
        {
            std::string key   = node.getID();
            std::string value = node.getData<std::string>();
            PutValue(parent, key, value);
        }
        else
        {
            for (auto& child : node.getChildren())
            {
                ParseNode(*child.second, node.getID());
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------

    void ConfigParser::ParseYamlFile(const std::string& filename)
    {
        TINY_YAML::Yaml yaml = TINY_YAML::Yaml(filename);

        for (auto& node : yaml.getNodes())
        {
            ParseNode(*node.second, "");
        }
    }

    //---------------------------------------------------------------------------------------------------------------------

    std::vector<std::string> ConfigParser::Parse()
    {
        for (unsigned int i = 0; i < configFilePaths_.size(); i++)
        {
            if (fs::exists(configFilePaths_[i]))
            {
                try
                {
                    ParseYamlFile(configFilePaths_[i]);
                }
                catch (const std::exception& e)
                {
                    LOG_ERROR("Failed to load config {}: {}", configFilePaths_[i], e.what());
                }
                LOG_INFO("Loaded config: {}", configFilePaths_[i]);
            }
            else
            {
                LOG_ERROR("Failed to locate config: {}", configFilePaths_[i]);
            }
        }

        return configs_;
    }

}  // namespace esmini::common