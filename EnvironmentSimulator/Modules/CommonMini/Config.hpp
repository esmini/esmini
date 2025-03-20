#pragma once

#include <string>
#include <optional>
#include <vector>
#include <unordered_map>

namespace esmini::common
{
    class Config
    {
    public:
        // constructor
        Config(const std::string& applicationName, int argc, char** argv);

        // destructor
        ~Config();

        // Will parse all the applicable config files and append them to argv_.
        // Returns the references to updated argc_ and argv_, so that user of these arguments can consume them and effectively reduce their size.
        // Remaining argv_ will be deleted by this class on destruction.
        std::pair<int&, char**&> Load();

        // Logs loaded file names, so user can see which config files are successfully loaded
        void LogLoadedConfigFiles() const;

        Config()                      = delete;
        Config(Config const&)         = delete;
        void operator=(Config const&) = delete;

        // private interface
    private:
        //  Read environment variables
        std::optional<std::string> GetEnvironmentVariableValue(const std::string& variableName) const;
        // Make default config file path
        std::string MakeDefaultConfigFilePath() const;
        // Loads default and environment config files into configFilePaths_
        void LoadDefaultAndEnvironmentConfigFiles();
        // Parses the config files and appends the arguments to argv_
        void ParseAndProcessConfigFiles();

        /**
         * Appends Argc and Argv with the arguments
         * @param appendIndex: Index until which original arguments should be kept, after which new arguments will be added. Once new arguments are
         * added, remaining original arguments will be added at the last
         * @param dataToAppend: Vector of strings to append, new arguments which needs to be added
         */
        void AppendArgcArgv(int appendIndex, const std::vector<std::string>& dataToAppend);

        /**
         * Perform final argument check, resolving conflicts and prioritization
         */
        void PostProcessArgs();
        // Some arguments can't co-exist with others, this function will be called to remove such arguments
        void RemoveOptionAndArguments(const char* option, unsigned int n_arguments, unsigned int start_index, unsigned int end_index);

        // private data members
    private:
        // Config file paths like default config file, environment variable config file etc.
        std::vector<std::string> configFilePaths_;
        // canonical and relative paths of config files, which are successfully loaded
        std::vector<std::pair<std::string, std::string>> loadedConfigFiles_;
        // Application for which we are parsing the config file i.e. esmini, replayer etc.
        std::string applicationName_;
        // Argument count
        int argc_;
        // Argument list
        char** argv_;
    };
}  // namespace esmini::common
