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


#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7)
	#define snprintf _snprintf
#endif

#ifdef _WIN32
   #define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#else
   #define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif

#define SMALL_NUMBER (1E-10)
#define LARGE_NUMBER (1E+10)
#define SIGN(X) ((X<0)?-1:1)
#define MAX(x, y) (y > x ? y : x)
#define MIN(x, y) (y < x ? y : x)

#define LOG(Format_, ...)  Logger::Inst().Log(__FILENAME__, __FUNCTION__, __LINE__, Format_, ##__VA_ARGS__)

// Time functions
__int64 SE_getSystemTime();
void SE_sleep(unsigned int msec);
double SE_getSimTimeStep(__int64 &time_stamp, double min_time_step, double max_time_step);

// Useful operations

/**
  Concatenate a directory path and a file path
*/
std::string CombineDirectoryPathAndFilepath(std::string dir_path,  std::string file_path);

/**
  Retrieve the angle in the range [0,2*PI]
*/
double GetAngleIn2PIInterval(double angle);

/**
  Retrieve the angle of a vector
*/
double GetAngleOfVector(double x, double y);

/**
  Retrieve the absolute value of difference between two angles (angle1 - angle2)
*/
double GetAbsAngleDifference(double angle1, double angle2);

/**
  Retrieve the sum of two angles
*/
double GetAngleSum(double angle1, double angle2);

/**
  Retrieve the angle in the interval [0, 2xPI]
*/
double GetAngleInInterval2PI(double angle);

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
  Calculate distance between two 2D points
*/
double PointDistance2D(double x0, double y0, double x1, double y1);

/**
  Calculate distance between two 2D points, return square value - avoiding square root operation
*/
double PointSquareDistance2D(double x0, double y0, double x1, double y1);

/**
  Project a 2D point on a 2D vector 
*/
void ProjectPointOnVector2D(double x, double y, double vx1, double vy1, double vx2, double vy2, double &px, double &py);

/**
  Check whether projected point is in between vector endpoints, or outside
*/
bool PointInBetweenVectorEndpoints(double x3, double y3, double x1, double y1, double x2, double y2, double &sNorm);

/**
  Find out whether the point is left or right to a vector 
*/
int PointSideOfVec(double px, double py, double vx1, double vy1, double vx2, double vy2);

/**
  Retrieve the length of a 2D line segment
*/
double GetLengthOfLine2D(double x1, double y1, double x2, double y2);

/**
  Retrieve the length of a 3D vector
*/
double GetLengthOfVector3D(double x, double y, double z);

/**
  Rotate a 2D vector 
*/
void RotateVec2D(double x, double y, double angle, double &xr, double &yr);

/**
  Normalize a 2D vector
*/
void NormalizeVec2D(double x, double y, double &xn, double &yn);

/**
  Change byte order - can be useful for IP communication with non Intel platforms
*/
void SwapByteOrder(unsigned char *buf, int data_type_size, int buf_size);

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


std::vector<std::string> SplitString(const std::string &s, char separator);
std::string DirNameOf(const std::string& fname);
std::string FileNameOf(const std::string& fname);
std::string FileNameWithoutExtOf(const std::string& fname);


// Global Logger class
class Logger
{
public:
	typedef void(*FuncPtr)(const char*);

	static Logger& Inst();
	void Log(char const* func, char const* file, int line, char const* format, ...);
	void SetCallback(FuncPtr callback);

private:
	bool use_logfile_;
	Logger();
	Logger(bool use_logfile);
	~Logger();
	FuncPtr callback_;

	std::ofstream file_;
};

// Global Vehicle Data Logger
class VehicleLogger
{
public:
	typedef void(*FuncPtr)(const char*);

	//Instantiator
	static VehicleLogger& InstVehicleLog(std::string scenario_filename,
		int numvehicles);

	//Logging function called by VehicleLogger object using pass by value
	void LogVehicleData(bool isendline, int indexcounter, double timestamp, 
		char const* name_, int id_, double speed_, double wheel_angle_,
		double wheel_rot_, double posX_, double posY_, double posZ_,
		double distance_road_, double distance_lanem_, double heading_,
		double heading_angle_, double heading_angle_driving_direction_,
		double pitch_, double curvature_, ...);

	void SetCallback(FuncPtr callback);

private:
	//Constructor to be called by instantiator
	VehicleLogger(std::string scenario_filename, int numvehicles);

	//Destructor
	~VehicleLogger();

	//File output stream
	std::ofstream file_;

	//Callback function pointer for error logging
	FuncPtr callback_;
};


// Argument parser 

class SE_Option
{
public:
	std::string opt_str_;
	std::string opt_desc_;
	std::string opt_arg_;
	bool set_;
	std::string arg_value_;

	SE_Option(std::string opt_str, std::string opt_desc, std::string opt_arg = "") :
		opt_str_(opt_str), opt_desc_(opt_desc), opt_arg_(opt_arg), set_(false), arg_value_("") {}

	void Usage();
};

class SE_Options
{
#define OPT_PREFIX "--"

public:
	void AddOption(std::string opt_str, std::string opt_desc, std::string opt_arg = "");
	void PrintUsage();
	void PrintArgs(int argc, char *argv[], std::string message = "Unrecognized arguments:");
	bool GetOptionSet(std::string opt);
	std::string GetOptionArg(std::string opt);
	void ParseArgs(int *argc, char* argv[]);

private:
	std::vector<SE_Option> option_;
	std::string app_name_;

	SE_Option *GetOption(std::string opt);
};

class SE_SystemTimer
{
public:
	__int64 start_time_;

	SE_SystemTimer() : start_time_(0) {}
	void Start()
	{
		start_time_ = SE_getSystemTime();
	}

	void Reset() { start_time_ = 0; }

	bool Started() { return start_time_ > 0 ? true : false; }
	double DurationS() { return 1E-3 * (SE_getSystemTime() - start_time_); }

};