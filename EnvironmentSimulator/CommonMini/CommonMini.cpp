#include <stdarg.h> 
#include <iostream>

#include "CommonMini.hpp"

#define DEBUG_TRACE
#define LOG_FILENAME "log.txt"


#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7)

	#include <windows.h>

	__int64 SE_getSystemTime()
	{
		return timeGetTime();
	}

	void SE_sleep(unsigned int msec)
	{
		Sleep(msec);
	}

#else

	#include <thread>
	#include <chrono>
	using namespace std::chrono;

	__int64 SE_getSystemTime()
	{
		return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
	}

	void SE_sleep(unsigned int msec)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000 * msec)));
	}

#endif

std::string DirNameOf(const std::string& fname)
{
	size_t pos = fname.find_last_of("\\/");
	return (std::string::npos == pos) ? "" : fname.substr(0, pos);
}

Logger::Logger()
{
	file_.open(LOG_FILENAME);
	if (file_.fail())
	{
		throw std::iostream::failure(std::string("Cannot open file: ") + LOG_FILENAME);
	}
}

Logger::~Logger()
{
	file_.close();
}

void Logger::Log(char const* file, char const* func, int line, char const* format, ...)
{
	char s[1024];
	va_list args;
	va_start(args, format);
	vsnprintf(s, 1024, format, args);
	file_ << file << "/" << line << "/" << func << "(): " << s << "\n";

#ifdef DEBUG_TRACE
	std::cout << file << "/" << line << "/" << func << "(): " << s << "\n";
#else
	std::cout << s << "\n";
#endif

	file_.flush();
	va_end(args);
}

Logger& Logger::Inst()
{
	static Logger instance;
	return instance;
}
