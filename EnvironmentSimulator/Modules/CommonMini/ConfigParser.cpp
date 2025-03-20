#include "ConfigParser.hpp"

#include "CommonMini.hpp"
#include "logger.hpp"

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

static size_t loadedFilesCount   = 0;
const size_t  LOADED_FILES_LIMIT = 100;

namespace esmini::common
{
    ConfigParser::ConfigParser(const std::string&                                applicationName,
                               const std::vector<std::string>&                   configFilePaths,
                               std::vector<std::pair<std::string, std::string>>& loadedConfigFiles)
        : applicationName_(applicationName),
          configFilePaths_(configFilePaths),
          loadedConfigFiles_(loadedConfigFiles)
    {
    }

    //---------------------------------------------------------------------------------------------------------------------

    bool ConfigParser::IsFaulty() const
    {
        return faulty_;
    }

    //---------------------------------------------------------------------------------------------------------------------

    std::string ConfigParser::GetParsingErrorMsg() const
    {
        return parsingErrorMsg_;
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
                ParseYamlFile(value);
                return;
            }

            const auto valueVec = SplitQuotedString(value, ' ');
            if (valueVec.size() == 1)
            {
                // this can be boolean value
                if (auto [isBool, boolValue] = StrToBool(valueVec[0]); isBool)
                {
                    // in case of boolean value, we need to add only the key to be in sync with the command line arguments
                    if (boolValue)
                    {
                        configs_.emplace_back(fmt::format("--{}", key));
                    }
                }
                else
                {
                    configs_.emplace_back(fmt::format("--{}", key));
                    configs_.emplace_back(valueVec[0]);
                }
            }
            else
            {
                configs_.emplace_back(fmt::format("--{}", key));
                configs_.insert(configs_.end(), std::make_move_iterator(valueVec.begin()), std::make_move_iterator(valueVec.end()));
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------

    void ConfigParser::ParseNode(TINY_YAML::Node& node, std::string parent)
    {
        if (faulty_)
        {
            return;
        }

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

    void ConfigParser::DoSanityChecksOnConfigFile(const std::string& filename)
    {
        if (!fs::exists(filename))
        {
            faulty_          = true;
            parsingErrorMsg_ = fmt::format("Failed to locate config: {}", filename);
            return;
        }
        if (fs::is_directory(fs::status(filename)))
        {
            faulty_          = true;
            parsingErrorMsg_ = fmt::format("Given path {} is a directory, config file path is expected instead.", filename);
            return;
        }
        if (++loadedFilesCount > LOADED_FILES_LIMIT)
        {
            faulty_          = true;
            parsingErrorMsg_ = fmt::format("Too many config files loaded (100). Possible circular dependency.");
            return;
        }
    }

    void ConfigParser::ParseYamlFile(const std::string& filename)
    {
        DoSanityChecksOnConfigFile(filename);
        if (faulty_)
        {
            return;
        }
        std::string canonicalPath{fs::canonical(fs::path(filename)).u8string()};
        loadedConfigFiles_.emplace_back(std::make_pair(canonicalPath, filename));
        try
        {
            TINY_YAML::Yaml yaml = TINY_YAML::Yaml(filename);
            for (auto& node : yaml.getNodes())
            {
                ParseNode(*node.second, "");
            }
        }
        catch (const std::exception& e)
        {
            loadedConfigFiles_.pop_back();  // its not needed since we are exiting but just in case we decide not to exit down the road
            faulty_          = true;
            parsingErrorMsg_ = e.what();
            return;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------

    std::vector<std::string> ConfigParser::Parse()
    {
        for (const auto& file : configFilePaths_)
        {
            ParseYamlFile(file);
        }
        return configs_;
    }

}  // namespace esmini::common