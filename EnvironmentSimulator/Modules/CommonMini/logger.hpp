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
#include "fmt/format.h"

#include <unordered_set>
#include <string>
#include <iostream>
#include <cstdio>

// Converts enum to its underlying integer type and formats it
template <typename T>
struct fmt::formatter<T, std::enable_if_t<std::is_enum_v<T>, char>> : fmt::formatter<int>
{
    template <typename FormatContext>
    auto format(T t, FormatContext& ctx) const
    {
        return fmt::formatter<int>::format(static_cast<int>(t), ctx);
    }
};

enum LogLevel
{
    debug = 0,
    info  = 1,
    warn  = 2,
    error = 3
};

namespace esmini::common
{
    // this function can be used for any file type i.e. txt, sim, dat, etc.
    // checks if the parent directory exists for the path
    // if the path is directory then appends default file name.
    // if the extension is missing then replaces it with default extension
    std::string ValidateAndCreateFilePath(const std::string& path, const std::string& defaultFileName, const std::string& defaultExtension);

    class TxtLogger
    {
    public:
        ~TxtLogger();
        // logs esmini version
        void LogVersion();

        // converts string to verbosity level if doesn't succeed then returns verbosity level INFO as default
        LogLevel GetVerbosityLevelFromStr(const std::string& str);

        // puts one line in log with time only
        void LogTimeOnly();

        // sets logger time which will be used with logged messages, it is normally the scenario time
        void SetLoggerTime(double* ptr);

        // stops file logging
        void StopFileLogging();

        // stops logging
        void Stop();

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
        std::string AddTimeAndMetaData(char const* function, char const* file, long line, const std::string& logLevelStr, const std::string& log);

        // Returns true if logging for the module should be done
        bool ShouldLogModule(char const* file);

        // creates and validates log file path
        std::string CreateLogFilePath();

        // sets verbosity level for all loggers, there might be a situation when logger is instantiated before parsing of options
        // with default log level i.e. info. This function should be called after parsing of options to set verbosity level
        // for all loggers based on the option i.e. log_level
        void SetLoggerVerbosity();

        // function type declaration for callbacks
        typedef void (*CallbackFuncPtr)(const std::string& msg);

        // register a callback function for receiving log messages
        void RegisterCallback(CallbackFuncPtr callback);

        // unregister all callbacks
        void ClearCallbacks();

        // get the number of registered callbacks
        unsigned int GetNumberOfCallbacks();

        template <class... ARGS>
        void
        Log(LogLevel msgLogLevel, const std::string& logStr, char const* function, char const* file, long line, const std::string& log, ARGS... args)
        {
            if (logLevel_ > msgLogLevel)
            {
                return;
            }
            if (!ShouldLogModule(file))
            {
                return;
            }
            consoleLoggingEnabled_ = !SE_Env::Inst().GetOptions().IsOptionArgumentSet("disable_stdout");
            fileLoggingEnabled_    = !SE_Env::Inst().GetOptions().GetOptionSet("disable_log");
            if (fileLoggingEnabled_ || consoleLoggingEnabled_)
            {
                Log(fmt::format(AddTimeAndMetaData(function, file, line, logStr, log), args...));
            }
        }
        // private interface
    private:
        // Creates a file logger with the given path and returns true otherwise returns false
        bool CreateLogFile();

        void Log(const std::string& msg);
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
        // log verbosity level
        LogLevel logLevel_ = LogLevel::info;
        // log file
        FILE*                        logFile_               = nullptr;
        bool                         firstConsoleLog_       = true;
        bool                         firstFileLog_          = true;
        bool                         consoleLoggingEnabled_ = true;
        bool                         fileLoggingEnabled_    = true;
        std::vector<CallbackFuncPtr> callbacks_;

    };  // class TxtLogger

}  // namespace esmini::common

using TxtLogger = esmini::common::TxtLogger;

extern TxtLogger txtLogger;

template <class... ARGS>
void __LOG_DEBUG__(char const* function, char const* file, long line, const std::string& log, ARGS... args)
{
    txtLogger.Log(LogLevel::debug, "debug", function, file, line, log, args...);
}

template <class... ARGS>
void __LOG_INFO__(char const* function, char const* file, long line, const std::string& log, ARGS... args)
{
    txtLogger.Log(LogLevel::info, "info", function, file, line, log, args...);
}

template <class... ARGS>
void __LOG_WARN__(char const* function, char const* file, long line, const std::string& log, ARGS... args)
{
    txtLogger.Log(LogLevel::warn, "warn", function, file, line, log, args...);
}

template <class... ARGS>
void __LOG_ERROR__(char const* function, char const* file, long line, const std::string& log, ARGS... args)
{
    txtLogger.Log(LogLevel::error, "error", function, file, line, log, args...);
}

template <class... ARGS>
void __LOG_ERROR__AND__QUIT__(char const* function, char const* file, long line, const std::string& log, ARGS... args)
{
    txtLogger.Log(LogLevel::error, "error", function, file, line, log, args...);
    txtLogger.SetLoggerTime(nullptr);  // stop logging time since pointer will be dangling with throw
    throw std::runtime_error(fmt::format(txtLogger.AddTimeAndMetaData(function, file, line, "error", log), args...));
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
