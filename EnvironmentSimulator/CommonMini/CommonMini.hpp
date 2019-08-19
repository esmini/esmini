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


#include <vector>
#include <fstream>
#include <string>
#define _USE_MATH_DEFINES
#include <math.h>

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


// Useful operations
/**
  Retrieve the absolute value of difference between two angles
*/
double GetAbsAngleDifference(double angle1, double angle2);

/**
  Retrieve the sum of two angles
*/
double GetAngleSum(double angle1, double angle2);

/**
  Retrieve difference between two angles
*/
double GetAngleDifference(double angle1, double angle2);

/**
  Retrieve the cross product of two vectors where z=0
*/
double GetCrossProduct2D(double x1, double y1, double x2, double y2);

/**
  Retrieve the dot product of two vectors where z=0
*/
double GetDotProduct2D(double x1, double y1, double x2, double y2);

/**
  Retrieve the intersection between two line segments/vectors a and b
  if found, x3 and x4 is the intersection point
  returns 0 if intersection exists, else -1
  Note: does not (yet) calculate whether point is within one of the line segments or not
*/
int GetIntersectionOfTwoLineSegments(double ax1, double ay1, double ax2, double ay2, double bx1, double by1, double bx2, double by2, double &x3, double &y3);

/**
  Retrieve the length of line segment
*/
double GetLengthOfVector(double x1, double y1, double x2, double y2);

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
	bool use_logfile_;
	Logger();
	Logger(bool use_logfile);
	~Logger();
	void Init();
	FuncPtr callback_;

	std::ofstream file_;
};
