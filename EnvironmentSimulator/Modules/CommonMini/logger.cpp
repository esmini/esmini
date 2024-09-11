#include "logger.hpp"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/fmt/fmt.h"

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

#include <unordered_set>

std::shared_ptr<spdlog::logger> consoleLogger;
std::shared_ptr<spdlog::logger> fileLogger;
std::string                     strTime;
std::string                     currentLogFileName;

void CreateFileLogger(const std::string& path)
{
    // std::cout << "currentLogFileName: " << currentLogFileName << " new file path: " << path << std::endl;
    if (path != currentLogFileName)
    {
        bool createNewFile = !SE_Env::Inst().GetOptions().GetOptionSet("log_append");
        // std::cout << path << " ----------------------------------creating new file\n";
        fileLogger = spdlog::basic_logger_mt("file", path, createNewFile);
        InitIndivisualLogger(fileLogger);
        currentLogFileName = path;
    }
    else
    {
        std::cout << "not creating log file" << std::endl;
    }
}

bool LogConsole()
{
    bool consoleLoggerDisabled = SE_Env::Inst().GetOptions().GetOptionSet("disable_stdout");
    if (consoleLogger)
    {
        if (consoleLoggerDisabled)
        {
            spdlog::drop("console");
            consoleLogger.reset();
            return false;
        }
    }
    else  // no console logging currently available
    {
        if (consoleLoggerDisabled)
        {
            return false;
        }
        else
        {
            consoleLogger = spdlog::stdout_color_mt("console");
            InitIndivisualLogger(consoleLogger);
        }
    }
    return true;
}

bool LogFile(const std::string& providedPath)
{
    bool fileLoggerDisabled = SE_Env::Inst().GetOptions().GetOptionSet("disable_log");

    if (fileLogger)
    {
        if (fileLoggerDisabled)
        {
            // std::cout << "deleting current file : " << currentLogFileName << " as logger disabled"<< std::endl;
            spdlog::drop("file");
            fileLogger.reset();
            currentLogFileName = "";
            return false;
        }
    }
    else  // no file logging currently available
    {
        if (fileLoggerDisabled)
        {
            return false;
        }
        else
        {
            // bool createNewFile = !SE_Env::Inst().GetOptions().GetOptionSet("log_append");

            if (!providedPath.empty())
            {
                // std::cout << "currentLogFileName: " << currentLogFileName << " providedPath: " << providedPath << "
                // -------------------------------------------------77----creating new file\n";
                // fileLogger = spdlog::basic_logger_mt("file", providedPath, createNewFile);
                // InitIndivisualLogger(fileLogger);
                // currentLogFileName = providedPath;
                CreateFileLogger(providedPath);
                return true;
            }

            std::string filePath;
            if (SE_Env::Inst().GetOptions().IsOptionArgumentSet("logfile_path"))
            {
                filePath = SE_Env::Inst().GetOptions().GetOptionArg("logfile_path");

                if (filePath.empty())
                {
                    // printf("Custom logfile path empty, disable logfile\n");
                }
                else
                {
                    printf("Custom logfile path: %s\n", filePath.c_str());
                    // std::cout << "currentLogFileName: " << currentLogFileName << " filePath: " << filePath << "
                    // ------------------------------------------------95-----creating new file\n"; fileLogger = spdlog::basic_logger_mt("file",
                    // filePath, createNewFile); InitIndivisualLogger(fileLogger); currentLogFileName = filePath;
                    CreateFileLogger(filePath);
                }
            }
            else
            {
                // std::cout << "Custom logfile path: log.txt new file flag " << createNewFile << std::endl;
                // std::cout << "currentLogFileName: " << currentLogFileName << "
                // -------------------------------------------103--log.txt--------creating new file\n";
                // fileLogger = spdlog::basic_logger_mt("file", "log.txt", createNewFile);
                // InitIndivisualLogger(fileLogger);
                // currentLogFileName = "log.txt";
                CreateFileLogger("log.txt");
            }
        }
    }
    return true;
}

void CreateNewFileForLogging(const std::string& filePath)
{
    if (filePath.empty() || currentLogFileName == filePath)
    {
        // In principle this shouldn't be true but edge cases make it happen :(
        return;
    }
    if (fileLogger)
    {
        // std::cout << "deleting current file : " << currentLogFileName << " for new file : " << filePath << std::endl;
        spdlog::drop("file");
        fileLogger.reset();
        currentLogFileName = "";
    }
    LogFile(filePath);
}

void StopFileLogging()
{
    if (fileLogger && !SE_Env::Inst().GetOptions().GetOptionSet("log_append"))
    {
        // std::cout << "deleting current file : " << currentLogFileName << " as stop called" << std::endl;
        spdlog::drop("file");
        fileLogger.reset();
        currentLogFileName = "";
    }
}

void StopConsoleLogging()
{
    if (consoleLogger)
    {
        spdlog::drop("console");
        consoleLogger.reset();
    }
}

spdlog::level::level_enum GetLogLevelFromStr(const std::string& str)
{
    if ("debug" == str)
    {
        return spdlog::level::debug;
    }
    else if ("info" == str)
    {
        return spdlog::level::info;
    }
    else if ("warn" == str)
    {
        return spdlog::level::warn;
    }
    else if ("error" == str)
    {
        return spdlog::level::err;
    }
    // by default we set info
    return spdlog::level::info;
}

bool ShouldLogModule(char const* file)
{
    std::string fileName = fs::path(file).stem().string();
    // it may seem that checking emptiness is an overhead as find function does it optimmally
    // but checking emptiness helps to find if user has enabled any file or not, because if its empty
    // then user wants all logs i.e. no filtering
    if (!loggerConfig.enabledFiles_.empty())
    {
        if (loggerConfig.enabledFiles_.find(fileName) == loggerConfig.enabledFiles_.end())
        {
            // not found in the list, which means user has not enabled this file for logging
            return false;
        }
        else
        {
            // file is present in the enabled list, we log
            return true;
        }
    }
    if (!loggerConfig.disabledFiles_.empty())
    {
        if (loggerConfig.disabledFiles_.find(fileName) == loggerConfig.disabledFiles_.end())
        {
            // not found in the list, which means this file is enabled for logging
            return true;
        }
        else
        {
            // file is present in the list, which means user has categorically disabled this file from logging
            return false;
        }
    }
    // if we are here then user doesnt have any enabled/disabled files, so we log
    return true;
}

std::string AddTimeAndMetaData(char const* function, char const* file, long line, const std::string& level, const std::string& log)
{
    // std::cout << "Pointer value: " << loggerConfig.time_ << std::endl;
    if (loggerConfig.time_ != nullptr)
    {
        strTime = fmt::format("[{:.3f}]", *loggerConfig.time_);
    }
    else
    {
        strTime = "[]";
    }

    if (SE_Env::Inst().GetOptions().GetOptionSet("log_meta_data"))
    {
        std::string fileName = fs::path(file).filename().string();
        std::string logWithTimeAndMeta{fmt::format("{} [{}] [{}::{}::{}] {}", strTime, level, fileName, function, line, log)};
        return logWithTimeAndMeta;
    }
    else
    {
        std::string logWithTime{fmt::format("{} [{}] {}", strTime, level, log)};
        return logWithTime;
    }
}

void SetLoggerTime(double* ptr)
{
    loggerConfig.time_ = ptr;
}

void InitIndivisualLogger(std::shared_ptr<spdlog::logger>& logger)
{
    try
    {
        // std::cout << "initializing logger ..." << std::endl;
        logger->set_pattern("%v");
        logger->info(GetVersionInfoForLog());
        if (SE_Env::Inst().GetOptions().IsOptionArgumentSet("log_level"))
        {
            logger->set_level(GetLogLevelFromStr(SE_Env::Inst().GetOptions().GetOptionArg("log_level")));
        }
        else
        {
            logger->set_level(spdlog::level::info);  // we keep info level as default
        }
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << '\n';
    }
    catch (...)
    {
        std::cout << "generic error incurred" << std::endl;
    }
}

void LogVersion()
{
    std::string esminiVersion = GetVersionInfoForLog();
    if (LogConsole())
    {
        consoleLogger->info(esminiVersion);
    }
    if (LogFile())
    {
        fileLogger->info(esminiVersion);
    }
}

void LogTimeOnly()
{
    if (LogConsole())
    {
        consoleLogger->set_pattern("[%Y-%m-%d %H:%M:%S]");
        consoleLogger->info("");
        consoleLogger->set_pattern("%v");
    }
    if (LogFile())
    {
        fileLogger->set_pattern("[%Y-%m-%d %H:%M:%S]");
        fileLogger->info("");
        fileLogger->set_pattern("%v");
    }
}

void SetupLogger(const LoggerConfig& logConfig)
{
    loggerConfig = logConfig;
}