/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

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
static std::string              strTime;
static std::string              currentLogFileName;
static std::string              defaultLogFileName = "log.txt";

LoggerConfig& LoggerConfig::Inst()
{
    static LoggerConfig loggerConfig_;
    return loggerConfig_;
}

void SetLoggerLevel(std::shared_ptr<spdlog::logger>& logger)
{
    if (!logger)
    {
        return;
    }
    if (SE_Env::Inst().GetOptions().IsOptionArgumentSet("log_level"))
    {
        logger->set_level(GetLogLevelFromStr(SE_Env::Inst().GetOptions().GetOptionArg("log_level")));
    }
    else
    {
        logger->set_level(spdlog::level::info);  // we keep info level as default
    }
}

void CreateFileLogger(const std::string& path)
{
    if ((path.empty() && currentLogFileName.empty()) || path != currentLogFileName)
    {
        bool        appendFile = SE_Env::Inst().GetOptions().IsOptionArgumentSet("log_append");
        std::string filePath   = path.empty() ? LoggerConfig::Inst().logFilePath_ : path;
        fileLogger             = spdlog::basic_logger_mt("file", filePath, !appendFile);
        SetLoggerLevel(fileLogger);
        fileLogger->set_pattern("%v");
        fileLogger->info(GetVersionInfoForLog());
        currentLogFileName = path.empty() ? LoggerConfig::Inst().logFilePath_ : path;
    }
    else
    {
        std::cout << "not creating log file" << std::endl;
    }
}

bool LogConsole()
{
    bool stdoutDisabled = SE_Env::Inst().GetOptions().IsOptionArgumentSet("disable_stdout");
    bool shouldLog      = LoggerConfig::Inst().persistedState_ != LOG_PERSISTANCE_STATE::LPS_FALSE && !stdoutDisabled;
    if (shouldLog && !consoleLogger)
    {
        consoleLogger = spdlog::stdout_color_mt("console");
        SetLoggerLevel(consoleLogger);
        consoleLogger->set_pattern("%v");
        consoleLogger->info(GetVersionInfoForLog());
    }
    return shouldLog;
}

bool LogFile(const std::string& providedPath)
{
    bool fileLoggerDisabled = SE_Env::Inst().GetOptions().GetOptionSet("disable_log");

    if (fileLogger)
    {
        if (fileLoggerDisabled)
        {
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
            if (!providedPath.empty())
            {
                CreateFileLogger(providedPath);
                return true;
            }

            std::string filePath;
            if (SE_Env::Inst().GetOptions().IsOptionArgumentSet("logfile_path"))
            {
                filePath = SE_Env::Inst().GetOptions().GetOptionArg("logfile_path");

                if (!filePath.empty())
                {
                    printf("Custom logfile path: %s\n", filePath.c_str());
                    CreateFileLogger(filePath);
                }
            }
            else
            {
                CreateFileLogger("log.txt");
            }
        }
    }
    return true;
}

void CreateNewFileForLogging(const std::string& filePath)
{
    SetLoggerLevel(fileLogger);
    SetLoggerLevel(consoleLogger);

    if (filePath.empty() || currentLogFileName == filePath)
    {
        return;
    }
    if (fileLogger)
    {
        spdlog::drop("file");
        fileLogger.reset();
        currentLogFileName = "";
    }
    LogFile(filePath);
}

void StopFileLogging()
{
    //    if (fileLogger && !SE_Env::Inst().GetOptions().GetOptionSet("log_append"))
    {
        spdlog::drop("file");
        fileLogger.reset();
        currentLogFileName = "";
        spdlog::shutdown();
        LoggerConfig::Inst().enabledFiles_.clear();
        LoggerConfig::Inst().disabledFiles_.clear();
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
    if (!LoggerConfig::Inst().enabledFiles_.empty())
    {
        if (LoggerConfig::Inst().enabledFiles_.find(fileName) == LoggerConfig::Inst().enabledFiles_.end())
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
    if (!LoggerConfig::Inst().disabledFiles_.empty())
    {
        if (LoggerConfig::Inst().disabledFiles_.find(fileName) == LoggerConfig::Inst().disabledFiles_.end())
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
    if (LoggerConfig::Inst().time_ != nullptr)
    {
        strTime = fmt::format("[{:.3f}]", *LoggerConfig::Inst().time_);
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
    LoggerConfig::Inst().time_ = ptr;
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

// we will override program option, discuss if there can be any issue in it
void EnableConsoleLogging(bool state, bool persistant)
{
    if (persistant)
    {
        if (state == true)
        {
            LoggerConfig::Inst().persistedState_ = LOG_PERSISTANCE_STATE::LPS_TRUE;
        }
        else
        {
            LoggerConfig::Inst().persistedState_ = LOG_PERSISTANCE_STATE::LPS_FALSE;
        }
    }
    else
    {
        LoggerConfig::Inst().persistedState_ = LOG_PERSISTANCE_STATE::LPS_UNDEFINED;
    }

    if (state)
    {
        SE_Env::Inst().GetOptions().UnsetOption("disable_stdout");
    }
    else
    {
        SE_Env::Inst().GetOptions().SetOptionValue("disable_stdout", "");
    }
}