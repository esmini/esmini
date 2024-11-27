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

#pragma once

#include "CommonMini.hpp"
#include "spdlog/spdlog.h"
#include <unordered_set>
#include <string>
#include <iostream>

// Converts enum to its underlying integer type and formats it
template <typename T>
struct fmt::formatter<T, std::enable_if_t<std::is_enum_v<T>, char>> : fmt::formatter<int>
{
    template <typename FormatContext>
    auto format(T t, FormatContext& ctx)
    {
        return fmt::formatter<int>::format(static_cast<int>(t), ctx);
    }
};

namespace esmini::common
{
    // this function can be used for any file type i.e. txt, sim, dat, etc.
    // checks if the parent directory exists for the path
    // if the path is directory then appends default file name.
    // if the extension is missing then replcaes it with default extension
    std::string ValidateAndCreateFilePath(const std::string& path, const std::string& defaultFileName, const std::string& defaultExtension);

    class TxtLogger
    {
    public:
        // returns the instance of the logger (singleton pattern)
        static TxtLogger& Inst();

        // logs esmini version
        void LogVersion();

        // converts string to verbosirty level if doesn't succeed then returns verbosity level INFO as default
        spdlog::level::level_enum GetVerbosityLevelFromStr(const std::string& str);

        // puts one line in log with time only
        void LogTimeOnly();

        // sets logger time which will be used with logged messages, it is normally the scenario time
        void SetLoggerTime(double* ptr);

        // stops file logging
        void StopFileLogging();

        // stops logging
        void Stop();

        // stops console logging
        void StopConsoleLogging();

        // Sets logger file path
        void SetLogFilePath(const std::string& path);

        // returns metadata enable or not
        bool IsMetaDataEnabled() const;

        // returns modules that should be logged, if empty then all modules should be logged
        const std::unordered_set<std::string>& GetLogOnlyModules() const;

        // returns modules that should not be logged, if empty then no modules should be skipped
        const std::unordered_set<std::string>& GetLogSkipModules() const;

        // enables/disables metadata in log
        void SetMetaDataEnabled(bool enabled);

        // sets modules that should be logged, if empty then all modules should be logged
        void SetLogOnlyModules(const std::unordered_set<std::string>& logOnlyModules);

        // sets modules that should not be logged, if empty then no modules should be skipped
        void SetLogSkipModules(const std::unordered_set<std::string>& logSkipModules);

        // add time and metadata to log message
        std::string AddTimeAndMetaData(char const* function, char const* file, long line, const std::string& level, const std::string& log);

        // Returns true if logging to console should be done
        bool ShouldLogToConsole();

        // Returns true if logging to file should be done
        bool ShouldLogToFile();

        // Returns true if logging for the module should be done
        bool ShouldLogModule(char const* file);

        // creates and validates log file path
        std::string CreateLogFilePath();

        // private interface
    private:
        // Private constructor for use in singleton pattern
        TxtLogger() = default;

        // Sets logger verbosity level based on option
        void SetLoggerVerbosity(std::shared_ptr<spdlog::logger>& logger);

        // Creates a file logger with the given path and returns true otherwise returns false
        bool CreateFileLogger();

        // private data
    private:
        // modules that should be logged, if empty then all modules should be logged
        std::unordered_set<std::string> logOnlyModules_;
        // modules that should not be logged, if empty then no modules should be skipped
        std::unordered_set<std::string> logSkipModules_;
        // file with logger is currently using to log
        std::string currentLogFileName_ = "";
        // flag to enable/disable metadata in log
        bool metaDataEnabled_ = false;
        // time of the scenario
        double* time_ = nullptr;
    };  // class TxtLogger

    extern std::shared_ptr<spdlog::logger> consoleLogger;
    extern std::shared_ptr<spdlog::logger> fileLogger;

}  // namespace esmini::common

using TxtLogger = esmini::common::TxtLogger;

template <class... ARGS>
void __LOG_DEBUG__(char const* function, char const* file, long line, const std::string& log, ARGS... args)
{
    if (!TxtLogger::Inst().ShouldLogModule(file))
    {
        return;
    }
    std::string logWithTimeAndMeta;
    if (TxtLogger::Inst().ShouldLogToConsole())
    {
        logWithTimeAndMeta = TxtLogger::Inst().AddTimeAndMetaData(function, file, line, "debug", log);
        esmini::common::consoleLogger->debug(logWithTimeAndMeta, args...);
    }
    if (TxtLogger::Inst().ShouldLogToFile())
    {
        if (logWithTimeAndMeta.empty())
        {
            logWithTimeAndMeta = TxtLogger::Inst().AddTimeAndMetaData(function, file, line, "debug", log);
        }
        esmini::common::fileLogger->debug(logWithTimeAndMeta, args...);
    }
}

template <class... ARGS>
void __LOG_INFO__(char const* function, char const* file, long line, const std::string& log, ARGS... args)
{
    if (!TxtLogger::Inst().ShouldLogModule(file))
    {
        return;
    }
    std::string logWithTimeAndMeta;
    if (TxtLogger::Inst().ShouldLogToConsole())
    {
        logWithTimeAndMeta = TxtLogger::Inst().AddTimeAndMetaData(function, file, line, "info", log);
        esmini::common::consoleLogger->info(logWithTimeAndMeta, args...);
    }
    if (TxtLogger::Inst().ShouldLogToFile())
    {
        if (logWithTimeAndMeta.empty())
        {
            logWithTimeAndMeta = TxtLogger::Inst().AddTimeAndMetaData(function, file, line, "info", log);
        }
        esmini::common::fileLogger->info(logWithTimeAndMeta, args...);
    }
}

template <class... ARGS>
void __LOG_WARN__(char const* function, char const* file, long line, const std::string& log, ARGS... args)
{
    if (!TxtLogger::Inst().ShouldLogModule(file))
    {
        return;
    }
    std::string logWithTimeAndMeta;
    if (TxtLogger::Inst().ShouldLogToConsole())
    {
        logWithTimeAndMeta = TxtLogger::Inst().AddTimeAndMetaData(function, file, line, "warn", log);
        esmini::common::consoleLogger->warn(logWithTimeAndMeta, args...);
    }
    if (TxtLogger::Inst().ShouldLogToFile())
    {
        if (logWithTimeAndMeta.empty())
        {
            logWithTimeAndMeta = TxtLogger::Inst().AddTimeAndMetaData(function, file, line, "warn", log);
        }
        esmini::common::fileLogger->warn(logWithTimeAndMeta, args...);
    }
}

template <class... ARGS>
void __LOG_ERROR__(char const* function, char const* file, long line, const std::string& log, ARGS... args)
{
    if (!TxtLogger::Inst().ShouldLogModule(file))
    {
        return;
    }
    std::string logWithTimeAndMeta;
    if (TxtLogger::Inst().ShouldLogToConsole())
    {
        logWithTimeAndMeta = TxtLogger::Inst().AddTimeAndMetaData(function, file, line, "error", log);
        esmini::common::consoleLogger->error(logWithTimeAndMeta, args...);
    }
    if (TxtLogger::Inst().ShouldLogToFile())
    {
        if (logWithTimeAndMeta.empty())
        {
            logWithTimeAndMeta = TxtLogger::Inst().AddTimeAndMetaData(function, file, line, "error", log);
        }
        esmini::common::fileLogger->error(logWithTimeAndMeta, args...);
    }
}

template <class... ARGS>
void __LOG_ERROR__AND__QUIT__(char const* function, char const* file, long line, const std::string& log, ARGS... args)
{
    std::string logMsg;
    if (TxtLogger::Inst().ShouldLogToConsole())
    {
        logMsg = fmt::format(TxtLogger::Inst().AddTimeAndMetaData(function, file, line, "error", log), args...);
        esmini::common::consoleLogger->error(logMsg);
    }
    if (TxtLogger::Inst().ShouldLogToFile())
    {
        if (logMsg.empty())
        {
            logMsg = fmt::format(TxtLogger::Inst().AddTimeAndMetaData(function, file, line, "error", log), args...);
        }
        esmini::common::fileLogger->error(logMsg);
    }
    throw std::runtime_error(logMsg);
}

#define LOG_ERROR_AND_QUIT(...) __LOG_ERROR__AND__QUIT__(__func__, __FILE__, __LINE__, ##__VA_ARGS__)

#define LOG_ERROR_ONCE(...)                                         \
    static bool firstTime = true;                                   \
    if (firstTime)                                                  \
    {                                                               \
        __LOG_ERROR__(__func__, __FILE__, __LINE__, ##__VA_ARGS__); \
        firstTime = false;                                          \
    }

#define LOG_ERROR(...) __LOG_ERROR__(__func__, __FILE__, __LINE__, ##__VA_ARGS__)

#define LOG_WARN_ONCE(...)                                         \
    static bool firstTime = true;                                  \
    if (firstTime)                                                 \
    {                                                              \
        __LOG_WARN__(__func__, __FILE__, __LINE__, ##__VA_ARGS__); \
        firstTime = false;                                         \
    }

#define LOG_WARN(...) __LOG_WARN__(__func__, __FILE__, __LINE__, ##__VA_ARGS__)

#define LOG_INFO(...) __LOG_INFO__(__func__, __FILE__, __LINE__, ##__VA_ARGS__)

#define LOG_DEBUG(...) __LOG_DEBUG__(__func__, __FILE__, __LINE__, ##__VA_ARGS__)
