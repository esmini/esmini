
#include "CommonMini.hpp"

#if WINVER == _WIN32_WINNT_WIN7

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
