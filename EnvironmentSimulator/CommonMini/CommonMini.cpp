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

#include <stdarg.h> 
#include <stdio.h>
#include <iostream>

#include "CommonMini.hpp"

#define DEBUG_TRACE
#define LOG_FILENAME "log.txt"



double GetAbsAngleDifference(double angle1, double angle2)
{
	double diff = fmod(angle2 - angle1, 2 * M_PI);

	if (diff < 0)
	{
		diff += 2 * M_PI;
	}

	if (diff > M_PI)
	{
		diff = 2 * M_PI - diff;
	}

	return diff;
}

#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7)

	#include <windows.h>
	#include <process.h>

	#define snprintf _snprintf

	__int64 SE_getSystemTime()
	{
		return timeGetTime();
	}

	void SE_sleep(unsigned int msec)
	{
		Sleep(msec);
	}




#else

	#include <chrono>

	using namespace std::chrono;

	__int64 SE_getSystemTime()
	{
		return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
	}

	void SE_sleep(unsigned int msec)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds((int)(msec)));
	}


#endif

std::string DirNameOf(const std::string& fname)
{
	size_t pos = fname.find_last_of("\\/");
	return (std::string::npos == pos) ? "" : fname.substr(0, pos);
}

std::string FileNameOf(const std::string& fname)
{
	size_t pos = fname.find_last_of("\\/");
	return (std::string::npos == pos) ? "" : fname.substr(pos+1);
}

Logger::Logger()
{
	file_.open(LOG_FILENAME);
	if (file_.fail())
	{
		throw std::iostream::failure(std::string("Cannot open file: ") + LOG_FILENAME);
	}
	callback_ = 0;
}

Logger::~Logger()
{
	file_.close();
	callback_ = 0;
}

void Logger::Log(char const* file, char const* func, int line, char const* format, ...)
{
	static char complete_entry[2048];
	static char message[1024];

	va_list args;
	va_start(args, format);
	vsnprintf(message, 1024, format, args);

#ifdef DEBUG_TRACE
	snprintf(complete_entry, 2048, "%s / %d / %s(): %s", file, line, func, message);
#else
	strncpy(complete_entry, message, 1024);
#endif
	
	file_ << complete_entry << std::endl;
	if (callback_)
	{
		callback_(complete_entry);
	}

	file_.flush();
	va_end(args);
}

Logger& Logger::Inst()
{
	static Logger instance;
	return instance;
}

SE_Thread::~SE_Thread()
{
#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7)

#else
	if (thread_.joinable())
	{
		thread_.join();
	}
#endif
}

void SE_Thread::Start(void(*func_ptr)(void*), void *arg)
{
#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7)
	thread_ = (void*)_beginthread(func_ptr, 0, arg);
#else
	thread_ = std::thread(func_ptr, arg);
#endif
}


void SE_Thread::Wait()
{
#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7)
	WaitForSingleObject((HANDLE)thread_, 1000);  // Should never need to wait for more than 1 sec
#else
	thread_.join();
#endif
}

SE_Mutex::SE_Mutex()
{
#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7)
	mutex_ = (void*)CreateMutex(
		NULL,              // default security attributes
		0,             // initially not owned
		NULL);             // unnamed mutex

	if (mutex_ == NULL)
	{
		LOG("CreateMutex error: %d\n", GetLastError());
		mutex_ = 0;
	}
#else

#endif
}


void SE_Mutex::Lock()
{
#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7)
	WaitForSingleObject(mutex_, 1000);  // Should never need to wait for more than 1 sec
#else
	mutex_.lock();
#endif
}

void SE_Mutex::Unlock()
{
#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7)
	ReleaseMutex(mutex_);
#else
	mutex_.unlock();
#endif
}
