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

#include "CommonMini.hpp"

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

namespace esmini::common
{
    std::shared_ptr<spdlog::logger> consoleLogger;
    std::shared_ptr<spdlog::logger> fileLogger;

    std::string ValidateAndCreateFilePath(const std::string& path, const std::string& defaultFileName, const std::string& defaultExtension)
    {
        fs::path filePath = path;

        if (filePath.has_parent_path() && !fs::exists(filePath.parent_path()))
        {
            std::string esminiVersion = GetVersionInfoForLog();
            if (TxtLogger::Inst().ShouldLogToConsole())
            {
                consoleLogger->error("Invalid file path : {} exiting", path);
            }
            else
            {
                std::cout << esminiVersion << '\n' << "Invalid file path : " << path << " exiting" << '\n';
            }
            exit(-1);
        }
        if (!filePath.has_filename())
        {
            filePath.append(defaultFileName);
        }
        else if (!filePath.has_extension())
        {
            if (fs::exists(filePath))
            {
                filePath.append(defaultFileName);
            }
            else
            {
                filePath.replace_extension(defaultExtension);
            }
        }
        return filePath.string();
    }

    TxtLogger& TxtLogger::Inst()
    {
        static TxtLogger txtLogger_;
        return txtLogger_;
    }

    bool TxtLogger::IsMetaDataEnabled() const
    {
        return metaDataEnabled_;
    }

    const std::unordered_set<std::string>& TxtLogger::GetLogOnlyModules() const
    {
        return logOnlyModules_;
    }

    const std::unordered_set<std::string>& TxtLogger::GetLogSkipModules() const
    {
        return logSkipModules_;
    }

    void TxtLogger::SetMetaDataEnabled(bool enabled)
    {
        metaDataEnabled_ = enabled;
    }

    void TxtLogger::SetLogOnlyModules(const std::unordered_set<std::string>& logOnlyModules)
    {
        logOnlyModules_ = logOnlyModules;
    }

    void TxtLogger::SetLogSkipModules(const std::unordered_set<std::string>& logSkipModules)
    {
        logSkipModules_ = logSkipModules;
    }

    void TxtLogger::SetLoggerVerbosity(std::shared_ptr<spdlog::logger>& logger)
    {
        if (!logger)
        {
            return;
        }
        if (SE_Env::Inst().GetOptions().IsOptionArgumentSet("log_level"))
        {
            logger->set_level(GetVerbosityLevelFromStr(SE_Env::Inst().GetOptions().GetOptionArg("log_level")));
        }
        else
        {
            logger->set_level(spdlog::level::info);
        }
    }

    std::string TxtLogger::CreateLogFilePath()
    {
        if (SE_Env::Inst().GetOptions().GetOptionSet("disable_log"))
        {
            return "";
        }
        std::string filePath = SE_Env::Inst().GetOptions().GetOptionArg("logfile_path");
        if (filePath.empty())
        {
            return "";
        }
        return ValidateAndCreateFilePath(filePath, LOG_FILENAME, "txt");
    }

    bool TxtLogger::CreateFileLogger()
    {
        std::string filePath = CreateLogFilePath();
        if (filePath.empty())
        {
            return false;
        }

        bool appendFile = SE_Env::Inst().GetOptions().IsOptionArgumentSet("log_append");

        try
        {
            fileLogger = spdlog::basic_logger_mt("file", filePath, !appendFile);
        }
        catch (const spdlog::spdlog_ex& ex)
        {
            std::string esminiVersion = GetVersionInfoForLog();
            std::cout << esminiVersion << '\n';
            std::cerr << "Logger initialization failed: " << ex.what() << std::endl;
            exit(-1);
        }
        catch (...)
        {
            std::string esminiVersion = GetVersionInfoForLog();
            std::cout << esminiVersion << '\n';
            std::cerr << "Logger initialization failed: Unknown exception" << std::endl;
            exit(-1);
        }
        currentLogFileName_ = filePath;
        SetLoggerVerbosity(fileLogger);
        fileLogger->set_pattern("%v");
        fileLogger->error(GetVersionInfoForLog());
        return true;
    }

    void TxtLogger::SetLogFilePath(const std::string& path)
    {
        std::string filePath = ValidateAndCreateFilePath(path, LOG_FILENAME, "txt");
        if (path.empty() || currentLogFileName_ == filePath)
        {
            return;
        }
        if (fileLogger)
        {
            spdlog::drop("file");
            fileLogger.reset();
            currentLogFileName_ = "";
        }
        if (filePath != SE_Env::Inst().GetOptions().GetOptionArg("logfile_path"))
        {
            SE_Env::Inst().GetOptions().SetOptionValue("logfile_path", filePath);
        }
        CreateFileLogger();
    }

    spdlog::level::level_enum TxtLogger::GetVerbosityLevelFromStr(const std::string& str)
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
        return spdlog::level::info;  // by default we set INFO
    }

    void TxtLogger::Stop()
    {
        StopFileLogging();
        StopConsoleLogging();
        logOnlyModules_.clear();
        logSkipModules_.clear();
    }

    void TxtLogger::StopFileLogging()
    {
        if (fileLogger)
        {
            spdlog::drop("file");
            fileLogger.reset();
            currentLogFileName_ = "";
            spdlog::shutdown();
        }
    }

    void TxtLogger::StopConsoleLogging()
    {
        if (consoleLogger)
        {
            spdlog::drop("console");
            consoleLogger.reset();
        }
    }

    bool TxtLogger::ShouldLogModule(char const* file)
    {
        std::string fileName = fs::path(file).stem().string();
        // it may seem that checking emptiness is an overhead as find function does it optimmally
        // but checking emptiness helps to find if user has enabled any file or not, because if its empty
        // then user wants all logs i.e. no filtering
        if (!logOnlyModules_.empty())
        {
            if (logOnlyModules_.find(fileName) == logOnlyModules_.end())
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
        if (!logSkipModules_.empty())
        {
            if (logSkipModules_.find(fileName) == logSkipModules_.end())
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

    std::string TxtLogger::AddTimeAndMetaData(char const* function, char const* file, long line, const std::string& level, const std::string& log)
    {
        std::string strTime;
        if (time_ != nullptr)
        {
            strTime = fmt::format("[{:.3f}]", *time_);
        }
        else
        {
            strTime = "[]";
        }
        if (metaDataEnabled_)
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

    void TxtLogger::SetLoggerTime(double* ptr)
    {
        time_ = ptr;
    }

    void TxtLogger::LogVersion()
    {
        ShouldLogToConsole();
        ShouldLogToFile();
    }

    void TxtLogger::LogTimeOnly()
    {
        if (ShouldLogToConsole())
        {
            consoleLogger->set_pattern("[%Y-%m-%d %H:%M:%S]");
            consoleLogger->error("");
            consoleLogger->set_pattern("%v");
        }
        if (ShouldLogToFile())
        {
            fileLogger->set_pattern("[%Y-%m-%d %H:%M:%S]");
            fileLogger->error("");
            fileLogger->set_pattern("%v");
        }
    }

    bool TxtLogger::ShouldLogToConsole()
    {
        bool shouldLog = !SE_Env::Inst().GetOptions().IsOptionArgumentSet("disable_stdout");
        if (shouldLog && !consoleLogger)
        {
            consoleLogger = spdlog::stdout_color_mt("console");
            SetLoggerVerbosity(consoleLogger);
            consoleLogger->set_pattern("%v");
            consoleLogger->error(GetVersionInfoForLog());
        }
        return shouldLog;
    }

    bool TxtLogger::ShouldLogToFile()
    {
        bool fileLoggerDisabled = SE_Env::Inst().GetOptions().GetOptionSet("disable_log");

        if (fileLogger)
        {
            if (fileLoggerDisabled)
            {
                spdlog::drop("file");
                fileLogger.reset();
                currentLogFileName_ = "";
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
                return CreateFileLogger();
            }
        }
        return true;
    }

}  // namespace esmini::common
