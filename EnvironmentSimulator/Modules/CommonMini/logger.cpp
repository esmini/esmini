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
#include <iostream>

TxtLogger txtLogger;

namespace esmini::common
{
    std::string ValidateAndCreateFilePath(const std::string& path, const std::string& defaultFileName, const std::string& defaultExtension)
    {
        fs::path filePath = path;

        if (filePath.has_parent_path() && !fs::exists(filePath.parent_path()))
        {
            std::cout << GetVersionInfoForLog() << '\n' << "Invalid file path : " << path << " exiting" << '\n';
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

    TxtLogger::~TxtLogger()
    {
        Stop();
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

    void TxtLogger::SetLoggerVerbosity()
    {
        if (SE_Env::Inst().GetOptions().IsOptionArgumentSet("log_level"))
        {
            logLevel_ = GetVerbosityLevelFromStr(SE_Env::Inst().GetOptions().GetOptionArg("log_level"));
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

    bool TxtLogger::CreateLogFile()
    {
        std::string filePath = CreateLogFilePath();
        if (filePath.empty())
        {
            return false;
        }

        try
        {
            if (SE_Env::Inst().GetOptions().IsOptionArgumentSet("log_append"))
            {
                logFile_ = fopen(filePath.c_str(), "a");
            }
            else
            {
                logFile_ = fopen(filePath.c_str(), "w");
            }

            if (logFile_ == nullptr)
            {
                std::cout << "Unable to open log file " << filePath << std::endl;
            }
        }
        catch (const std::exception& ex)
        {
            std::cout << GetVersionInfoForLog() << '\n';
            std::cerr << "Logger initialization failed: " << ex.what() << std::endl;
            exit(-1);
        }
        catch (...)
        {
            std::cout << GetVersionInfoForLog() << '\n';
            std::cerr << "Logger initialization failed: Unknown exception" << std::endl;
            exit(-1);
        }
        currentLogFileName_ = filePath;
        return true;
    }

    void TxtLogger::SetLogFilePath(const std::string& path)
    {
        std::string filePath = ValidateAndCreateFilePath(path, LOG_FILENAME, "txt");
        if (path.empty() || currentLogFileName_ == filePath)
        {
            return;
        }
        StopFileLogging();
        if (filePath != SE_Env::Inst().GetOptions().GetOptionArg("logfile_path"))
        {
            SE_Env::Inst().GetOptions().SetOptionValue("logfile_path", filePath);
        }
        CreateLogFile();
    }

    LogLevel TxtLogger::GetVerbosityLevelFromStr(const std::string& str)
    {
        if ("debug" == str)
        {
            return LogLevel::debug;
        }
        else if ("info" == str)
        {
            return LogLevel::info;
        }
        else if ("warn" == str)
        {
            return LogLevel::warn;
        }
        else if ("error" == str)
        {
            return LogLevel::error;
        }
        return LogLevel::info;  // by default we set INFO
    }

    void TxtLogger::Stop()
    {
        StopFileLogging();
        logOnlyModules_.clear();
        logSkipModules_.clear();
        firstConsoleLog_ = true;
    }

    void TxtLogger::StopFileLogging()
    {
        if (logFile_ != nullptr)
        {
            fclose(logFile_);
            logFile_ = nullptr;
        }
        currentLogFileName_.clear();
        firstFileLog_ = true;
    }

    bool TxtLogger::ShouldLogModule(char const* file)
    {
        std::string fileName;
        if (!logOnlyModules_.empty())
        {
            fileName = fs::path(file).stem().string();
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
            if (fileName.empty())
            {
                fileName = fs::path(file).stem().string();
            }
            if (logSkipModules_.find(fileName) != logSkipModules_.end())
            {
                // file is present in the list, which means user has categorically disabled this file from logging
                return false;
            }
        }
        // if we are here then user doesn't have any enabled/disabled files, so we log
        return true;
    }

    std::string TxtLogger::AddTimeAndMetaData(char const*        function,
                                              char const*        file,
                                              long               line,
                                              const std::string& logLevelStr,
                                              const std::string& log)
    {
        if (metaDataEnabled_)
        {
            return fmt::format("[{}] [{}] [{}::{}::{}] {}\n",
                               time_ == nullptr ? "" : fmt::format("{:.3f}", *time_),
                               logLevelStr,
                               fs::path(file).filename().string(),
                               function,
                               line,
                               log);
        }
        else
        {
            return fmt::format("[{}] [{}] {}\n", time_ == nullptr ? "" : fmt::format("{:.3f}", *time_), logLevelStr, log);
        }
    }

    void TxtLogger::SetLoggerTime(double* ptr)
    {
        time_ = ptr;
    }

    void TxtLogger::LogVersion()
    {
        if (!SE_Env::Inst().GetOptions().IsOptionArgumentSet("disable_stdout"))
        {
            fputs(GetVersionInfoForLog().c_str(), stdout);
        }
        if (SE_Env::Inst().GetOptions().GetOptionSet("disable_log"))
        {
            if (logFile_ == nullptr)
            {
                if (!CreateLogFile())
                {
                    return;
                }
            }
            fputs(GetVersionInfoForLog().c_str(), logFile_);
        }
    }

    void TxtLogger::LogTimeOnly()
    {
        auto        now              = std::chrono::system_clock::now();
        auto        dateTimeUntilSec = std::chrono::time_point_cast<std::chrono::seconds>(now);
        std::string dateTime         = fmt::format("[{:%Y-%m-%d %H:%M:%S}]\n", dateTimeUntilSec);
        Log(dateTime);
    }

    void TxtLogger::Log(const std::string& msg)
    {
        if (consoleLoggingEnabled_)
        {
            if (firstConsoleLog_)
            {
                fputs(GetVersionInfoForLog().c_str(), stdout);
                firstConsoleLog_ = false;
            }
            fputs(msg.c_str(), stdout);
        }
        if (fileLoggingEnabled_)
        {
            if (logFile_ == nullptr)
            {
                if (!CreateLogFile())
                {
                    return;
                }
            }
            if (firstFileLog_)
            {
                fputs(GetVersionInfoForLog().c_str(), logFile_);
                firstFileLog_ = false;
            }
            fputs(msg.c_str(), logFile_);
        }
    }
}  // namespace esmini::common
