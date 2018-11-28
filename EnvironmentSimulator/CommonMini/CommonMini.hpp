#pragma once


#include <vector>
#include <fstream>
#include <string>

#ifdef _WIN32
	#include <sdkddkver.h>
#else
	#include <inttypes.h>
	typedef int64_t __int64;
#endif

#ifdef _WIN32
   #define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#else
   #define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif

#define SMALL_NUMBER (1E-10)
#ifndef INFINITY
#define INFINITY (~0)
#endif

#define LOG(Format_, ...)  Logger::Inst().Log(__FILENAME__, __FUNCTION__, __LINE__, Format_, ##__VA_ARGS__)


// Time functions
__int64 SE_getSystemTime();
void SE_sleep(unsigned int msec);

std::string DirNameOf(const std::string& fname);
std::string FileNameOf(const std::string& fname);


// Global Logger class
class Logger
{
public:
	static Logger& Inst();
	void Log(char const* func, char const* file, int line, char const* format, ...);

private:
	Logger();
	~Logger();

	std::ofstream file_;
};
