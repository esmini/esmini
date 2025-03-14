#include "Config.hpp"

#include "ConfigParser.hpp"
#include "logger.hpp"
#include "CommonMini.hpp"
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
    Config::Config(const std::string& applicationName, int argc, char** argv) : applicationName_(applicationName), argc_(argc)
    {
        argv_ = new char*[argc];
        for (int i = 0; i < argc; i++)
        {
            argv_[i] = new char[strlen(argv[i]) + 1];
            StrCopy(argv_[i], argv[i], strlen(argv[i]) + 1);
        }

        LoadDefaultAndEnvironmentConfigFiles();
    }

    Config::~Config()
    {
        for (int i = 0; i < argc_; i++)
        {
            delete[] argv_[i];
        }
        delete[] argv_;
    }

    void Config::LoadDefaultAndEnvironmentConfigFiles()
    {
        auto defaultConfigPath = MakeDefaultConfigFilePath();
        if (!fs::exists(defaultConfigPath))
        {
            LOG_INFO("Ignoring missing default config: {}", defaultConfigPath);
        }
        else
        {
            configFilePaths_.push_back(defaultConfigPath);
        }

        if (const auto envConfigFile = GetEnvironmentVariableValue("ESMINI_CONFIG_FILE"); envConfigFile.has_value())
        {
            LOG_INFO("Found environment variable ESMINI_CONFIG_FILE: {}", envConfigFile.value());
#ifdef _WIN32
            const auto valueVec = SplitString(envConfigFile.value(), ';');
#else
            const auto valueVec = SplitString(envConfigFile.value(), ':');
#endif
            configFilePaths_.insert(configFilePaths_.end(), std::make_move_iterator(valueVec.begin()), std::make_move_iterator(valueVec.end()));
        }
    }

    std::pair<int&, char**&> Config::Load()
    {
        ParseAndProcessConfigFiles();
        return std::make_pair(std::ref(argc_), std::ref(argv_));
    }

    std::string Config::MakeDefaultConfigFilePath() const
    {
        fs::path    defaultPath(GetDefaultPath());
        std::string defaultFilePath = (defaultPath.parent_path() / ".." / DEFAULT_CONFIG_FILE).string();
        return defaultFilePath;
    }

    std::optional<std::string> Config::GetEnvironmentVariableValue(const std::string& variableName) const
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

    void Config::ParseAndProcessConfigFiles()
    {
        std::vector<std::string> allConfigs;

        // parse default config file and environment variable config files
        esmini::common::ConfigParser parser(applicationName_, configFilePaths_, loadedConfigFiles_);
        allConfigs = parser.Parse();
        if (parser.IsFaulty())
        {
            LOG_ERROR_AND_QUIT(parser.GetParsingErrorMsg());
        }
        // there is a possibility that the config file path is already set in options, maybe through the api call
        // so we need to parse those config files as well. Since, order of appearance matters, so we will reverse iterate
        SE_Options& opt             = SE_Env::Inst().GetOptions();
        auto        configFilePaths = opt.GetOptionArgs(CONFIG_FILE_OPTION_NAME);
        for (auto rItr = configFilePaths.rbegin(); rItr != configFilePaths.rend(); ++rItr)
        {
            esmini::common::ConfigParser configParser(applicationName_, {*rItr}, loadedConfigFiles_);
            auto                         configs = configParser.Parse();
            if (configParser.IsFaulty())
            {
                LOG_ERROR_AND_QUIT(configParser.GetParsingErrorMsg());
            }
            allConfigs.insert(allConfigs.end(), std::make_move_iterator(configs.begin()), std::make_move_iterator(configs.end()));
        }
        opt.UnsetOption(CONFIG_FILE_OPTION_NAME);
        // parse config file path(s) from the arguments, if present. And append the configs to the arguments
        std::string configFilePathOption = fmt::format("--{}", CONFIG_FILE_OPTION_NAME);
        for (int i = 1; i < argc_; ++i)
        {
            if (strcmp(configFilePathOption.c_str(), argv_[i]) == 0 && i < argc_ - 1)  // we protect against buffer overflow
            {
                // now we can parse config file here
                esmini::common::ConfigParser configParser(applicationName_, {argv_[i + 1]}, loadedConfigFiles_);
                auto                         configs = configParser.Parse();
                if (configParser.IsFaulty())
                {
                    LOG_ERROR_AND_QUIT(configParser.GetParsingErrorMsg());
                }

                // we need to wipe out the config file path from the arguments, so that they wont be consumed again
                // free memory of the two arguments and shift the rest of the arguments
                delete[] argv_[i];
                delete[] argv_[i + 1];

                for (int j = i; j < argc_ - 2; ++j)
                {
                    argv_[j] = argv_[j + 2];
                }
                // indicate not in use
                argv_[argc_ - 2] = nullptr;
                argv_[argc_ - 1] = nullptr;
                argc_ -= 2;
                AppendArgcArgv(i, configs);
                --i;  // to recheck the same index, as we have shifted the arguments (since we start i with 1 so its safe to decrement in this case)
            }
        }

        // since the config file(s) from arguments are already parsed and appended to the arguments.
        // We just want to keep the application name at first index, after it we low priority configs
        AppendArgcArgv(1, allConfigs);
        // perform final argument check, resolving conflicts and prioritization
        PostProcessArgs();
    }

    void Config::LogLoadedConfigFiles() const
    {
        for (const auto& [canonicalPath, relativePath] : loadedConfigFiles_)
        {
            LOG_INFO("Loaded config file: {} ({})", canonicalPath, relativePath);
        }
    }

    void Config::PostProcessArgs()
    {
        // ignore any window argument prior to any headless argument

        // first, find positions of last occurrence of relevant options
        int last_headless_index = -1;
        int last_window_index   = -1;
        int last_osc_index      = -1;
        int last_osc_str_index  = -1;

        for (int i = 0; i < argc_; i++)
        {
            if (strcmp(argv_[i], "--headless") == 0)
            {
                last_headless_index = i;
            }
            else if (strcmp(argv_[i], "--window") == 0)
            {
                last_window_index = i;
            }
            else if (strcmp(argv_[i], "--osc") == 0)
            {
                last_osc_index = i;
            }
            else if (strcmp(argv_[i], "--osc_str") == 0)
            {
                last_osc_str_index = i;
            }
        }

        // remove any window arguments prior to headless argument
        if (last_window_index > -1 && last_window_index < last_headless_index)
        {
            RemoveOptionAndArguments("--window", 4, 0, last_headless_index);
        }

        // remove any osc arguments prior to osc_str argument
        if (last_osc_index > -1 && last_osc_index < last_osc_str_index)
        {
            RemoveOptionAndArguments("--osc", 1, 0, last_osc_str_index);
        }

        // remove any osc_str arguments prior to osc argument
        if (last_osc_str_index > -1 && last_osc_str_index < last_osc_index)
        {
            RemoveOptionAndArguments("--osc_str", 1, 0, last_osc_index);
        }
    }

    void Config::AppendArgcArgv(int appendIndex, const std::vector<std::string>& prefixArgs)
    {
        if (prefixArgs.empty())
        {
            return;
        }
        int          previousArgc = argc_;
        unsigned int newArgc      = static_cast<unsigned int>(argc_ + prefixArgs.size());
        char**       newArgv      = new char*[newArgc];

        int i;
        // firstly, copy the original arguments before the appendIndex
        for (i = 0; i < appendIndex; ++i)
        {
            newArgv[i] = new char[std::strlen(argv_[i]) + 1];
            StrCopy(newArgv[i], argv_[i], std::strlen(argv_[i]) + 1);
        }
        // secondly, copy the prefix arguments
        for (const auto& arg : prefixArgs)
        {
            newArgv[i] = new char[arg.length() + 1];
            StrCopy(newArgv[i], arg.c_str(), arg.length() + 1);
            ++i;
        }
        // thirdly, copy the original arguments from the appendIndex
        for (int j = appendIndex; j < argc_; ++j)
        {
            newArgv[i] = new char[std::strlen(argv_[j]) + 1];
            StrCopy(newArgv[i], argv_[j], std::strlen(argv_[j]) + 1);
            ++i;
        }

        for (int j = 0; j < previousArgc; ++j)
        {
            delete[] argv_[j];
        }
        delete[] argv_;

        argc_ = newArgc;
        argv_ = newArgv;
    }

    void Config::RemoveOptionAndArguments(const char* option, unsigned int n_arguments, unsigned int start_index, unsigned int end_index)
    {
        for (unsigned int i = start_index; i < end_index; i++)
        {
            if (strcmp(argv_[i], option) == 0)
            {
                int new_argc = argc_;
                for (unsigned int j = 0; i + j < static_cast<unsigned int>(argc_) && j < n_arguments + 1; j++)  // +1 to include option itself
                {
                    delete[] argv_[i + j];
                    argv_[i + j] = nullptr;
                    new_argc--;
                    end_index--;
                }

                // and shift the remaining the arguments
                for (unsigned int k = i; k < argc_ - (n_arguments + 1); k++)
                {
                    argv_[k] = argv_[k + n_arguments + 1];
                }

                argc_ = new_argc;
            }
        }
    }
}  // namespace esmini::common