#pragma once


#include <vector>
#include <fstream>
#include <string>

#ifdef _WIN32
	#include <sdkddkver.h>
#else
	#include <cstring>
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

#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7)

#else
	#include <thread>
	#include <mutex>
#endif

class SE_Thread
{
public:
#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7)
	SE_Thread() : thread_() {}
#else
	SE_Thread() {}
#endif
	~SE_Thread();

	void Wait();
	void Start(void(*func_ptr)(void*), void *arg);

private:
#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7)
	void *thread_;
#else
	std::thread thread_;
#endif
};

class SE_Mutex
{
public:
	SE_Mutex();

	void Lock();
	void Unlock();

private:
#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7)
	void *mutex_;
#else
	std::mutex mutex_;
#endif
};


std::string DirNameOf(const std::string& fname);
std::string FileNameOf(const std::string& fname);


// Global Logger class
class Logger
{
public:
	typedef void(*FuncPtr)(const char*);

	static Logger& Inst();
	void Log(char const* func, char const* file, int line, char const* format, ...);
	void SetCallback(FuncPtr callback) { callback_ = callback; }

private:
	Logger();
	~Logger();
	FuncPtr callback_;

	std::ofstream file_;
};
