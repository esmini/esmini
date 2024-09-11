#pragma once

#include "CommonMini.hpp"
#include "logger.hpp"
#include "Utils.h"

static int ParseAndSetLoggerOptions(int argc, char** argv)
{
    SE_Options& opt = SE_Env::Inst().GetOptions();
    opt.AddOption("log_append", "log all scenarios in the same txt file");
    opt.AddOption("log_meta_data", "log file name, function name and line number");
    opt.AddOption("log_level", "log level debug, info, warn, error", "mode");
    opt.AddOption("log_only_modules", "log from only these modules. Overrides logSkip_Modules", "modulename(s)");
    opt.AddOption("log_skip_modules", "skip log from these modules, all remaining modules will be logged.", "modulename(s)");

    if (opt.ParseArgs(argc, argv) != 0)
    {
        return -2;
    }
    LoggerConfig logConfig;
    std::string  arg_str;
    if (opt.IsOptionArgumentSet("log_only_modules"))
    {
        arg_str             = opt.GetOptionArg("log_only_modules");
        const auto splitted = utils::SplitString(arg_str, ',');
        if (!splitted.empty())
        {
            logConfig.enabledFiles_.insert(splitted.begin(), splitted.end());
        }
    }
    if (opt.IsOptionArgumentSet("log_Skip_Modules"))
    {
        arg_str             = opt.GetOptionArg("log_Skip_Modules");
        const auto splitted = utils::SplitString(arg_str, ',');
        if (!splitted.empty())
        {
            logConfig.disabledFiles_.insert(splitted.begin(), splitted.end());
        }
    }

    SetupLogger(logConfig);
    return 0;
}

// by default no logging at all, the app code can create log though
// if user enable logging, by setting any of the above options, then enable console AND file logging with no append by default