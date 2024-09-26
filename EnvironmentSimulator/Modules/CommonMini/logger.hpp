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

// Convert enum to its underlying integer type and format it
template <typename T>
struct fmt::formatter<T, std::enable_if_t<std::is_enum_v<T>, char>> : fmt::formatter<int>
{
    template <typename FormatContext>
    auto format(T t, FormatContext& ctx)
    {
        return fmt::formatter<int>::format(static_cast<int>(t), ctx);
    }
};

enum class LOG_PERSISTANCE_STATE
{
    LPS_UNDEFINED,
    LPS_TRUE,
    LPS_FALSE
};

struct LoggerConfig
{
    static LoggerConfig&            Inst();
    LOG_PERSISTANCE_STATE           persistedState_ = LOG_PERSISTANCE_STATE::LPS_UNDEFINED;
    std::string                     logFilePath_    = "log.txt";
    std::unordered_set<std::string> enabledFiles_;
    std::unordered_set<std::string> disabledFiles_;
    double*                         time_ = nullptr;

private:
    LoggerConfig() = default;
};

bool                      ShouldLogModule(char const* file);
void                      LogVersion();
std::string               AddTimeAndMetaData(char const* function, char const* file, long line, const std::string& level, const std::string& log);
spdlog::level::level_enum GetLogLevelFromStr(const std::string& str);
void                      LogTimeOnly();
void                      SetLoggerTime(double* ptr);
bool                      LogConsole();
bool                      LogFile(const std::string& filePath = "");
void                      StopFileLogging();
void                      StopConsoleLogging();
void                      CreateNewFileForLogging(const std::string&);
void                      EnableConsoleLogging(bool mode, bool persistant);

extern std::shared_ptr<spdlog::logger> consoleLogger;
extern std::shared_ptr<spdlog::logger> fileLogger;

extern LoggerConfig loggerConfig;

template <class... ARGS>
void __LOG_DEBUG__(char const* function, char const* file, long line, const std::string& log, ARGS... args)
{
    if (!ShouldLogModule(file))
    {
        return;
    }
    std::string logWithTimeAndMeta;
    if (LogConsole())
    {
        logWithTimeAndMeta = AddTimeAndMetaData(function, file, line, "debug", log);
        consoleLogger->debug(logWithTimeAndMeta, args...);
    }
    if (LogFile())
    {
        if (logWithTimeAndMeta.empty())
        {
            logWithTimeAndMeta = AddTimeAndMetaData(function, file, line, "debug", log);
        }
        fileLogger->debug(logWithTimeAndMeta, args...);
    }
}

template <class... ARGS>
void __LOG_INFO__(char const* function, char const* file, long line, const std::string& log, ARGS... args)
{
    if (!ShouldLogModule(file))
    {
        return;
    }
    std::string logWithTimeAndMeta;
    if (LogConsole())
    {
        logWithTimeAndMeta = AddTimeAndMetaData(function, file, line, "info", log);
        consoleLogger->info(logWithTimeAndMeta, args...);
    }
    if (LogFile())
    {
        if (logWithTimeAndMeta.empty())
        {
            logWithTimeAndMeta = AddTimeAndMetaData(function, file, line, "info", log);
        }
        fileLogger->info(logWithTimeAndMeta, args...);
    }
}

template <class... ARGS>
void __LOG_WARN__(char const* function, char const* file, long line, const std::string& log, ARGS... args)
{
    if (!ShouldLogModule(file))
    {
        return;
    }
    std::string logWithTimeAndMeta;
    if (LogConsole())
    {
        logWithTimeAndMeta = AddTimeAndMetaData(function, file, line, "warn", log);
        consoleLogger->warn(logWithTimeAndMeta, args...);
    }
    if (LogFile())
    {
        if (logWithTimeAndMeta.empty())
        {
            logWithTimeAndMeta = AddTimeAndMetaData(function, file, line, "warn", log);
        }
        fileLogger->warn(logWithTimeAndMeta, args...);
    }
}

template <class... ARGS>
void __LOG_ERROR__(char const* function, char const* file, long line, const std::string& log, ARGS... args)
{
    if (!ShouldLogModule(file))
    {
        return;
    }
    std::string logWithTimeAndMeta;
    if (LogConsole())
    {
        logWithTimeAndMeta = AddTimeAndMetaData(function, file, line, "error", log);
        consoleLogger->error(logWithTimeAndMeta, args...);
    }
    if (LogFile())
    {
        if (logWithTimeAndMeta.empty())
        {
            logWithTimeAndMeta = AddTimeAndMetaData(function, file, line, "error", log);
        }
        fileLogger->error(logWithTimeAndMeta, args...);
    }
}

template <class... ARGS>
void __LOG_ERROR__AND__QUIT__(char const* function, char const* file, long line, const std::string& log, ARGS... args)
{
    std::string logMsg;
    if (LogConsole())
    {
        logMsg = fmt::format(AddTimeAndMetaData(function, file, line, "error", log), args...);
        consoleLogger->error(logMsg);
    }
    if (LogFile())
    {
        if (logMsg.empty())
        {
            logMsg = fmt::format(AddTimeAndMetaData(function, file, line, "error", log), args...);
        }
        fileLogger->error(logMsg);
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