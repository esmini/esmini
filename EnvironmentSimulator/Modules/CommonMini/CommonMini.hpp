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
#define CLAMP(x, lo, hi) MIN(hi, MAX(lo, x))
#define OSI_MAX_LONGITUDINAL_DISTANCE 50
#define OSI_MAX_LATERAL_DEVIATION 0.05
#define LOG_FILENAME "log.txt"

#define LOG(Format_, ...)  Logger::Inst().Log(false, false, __FILENAME__, __FUNCTION__, __LINE__, Format_, ##__VA_ARGS__)
#define LOG_TRACE(Format_, ...)  Logger::Inst().Log(false, true, __FILENAME__, __FUNCTION__, __LINE__, Format_, ##__VA_ARGS__)
#define LOG_ONCE(Format_, ...)  { \
	static bool firstTime = true; \
	if (firstTime) \
	{ \
		Logger::Inst().Log(false, false, __FILENAME__, __FUNCTION__, __LINE__, Format_, ##__VA_ARGS__); \
		firstTime = false; \
	} \
}
#define LOG_TRACE_ONCE(Format_, ...)  { \
	static bool firstTime = true; \
	if (firstTime) \
	{ \
		Logger::Inst().Log(false, true, __FILENAME__, __FUNCTION__, __LINE__, Format_, ##__VA_ARGS__); \
		firstTime = false; \
	} \
}
#define LOG_AND_QUIT(Format_, ...)  Logger::Inst().Log(true, false, __FILENAME__, __FUNCTION__, __LINE__, Format_, ##__VA_ARGS__)
#define LOG_TRACE_AND_QUIT(Format_, ...)  Logger::Inst().Log(true, true, __FILENAME__, __FUNCTION__, __LINE__, Format_, ##__VA_ARGS__)

// Time functions
__int64 SE_getSystemTime();
void SE_sleep(unsigned int msec);
double SE_getSimTimeStep(__int64 &time_stamp, double min_time_step, double max_time_step);

// Useful types

enum class ControlDomains
{
	DOMAIN_NONE = 0,
	DOMAIN_LONG = 1,
	DOMAIN_LAT = 2,
	DOMAIN_BOTH = 3  // can also be interpreted as bitwise OR: DIM_LONG | DIM_LAT
};

std::string ControlDomain2Str(ControlDomains domains);

// Useful operations


/**
  Checks whether file with given path exists or not
*/
bool FileExists(const char* fileName);

/**
  Concatenate a directory path and a file path
*/
std::string CombineDirectoryPathAndFilepath(std::string dir_path,  std::string file_path);

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
  This function checks if the angle is in the interval of [-pi/2, +pi/2]
*/
bool IsAngleStraight(double teta);

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
 Calculate (shortest) distance between a point to a line, in 2D
 Inspiration: https://www.geeksforgeeks.org/shortest-distance-between-a-line-and-a-point-in-a-3-d-plane/
 But modified so that negtive distance means point is on right side of the line and vice versa
 @param px X-coordinate of the point
 @param py Y-coordinate of the point
 @param lx0 X-coordinate of the first endpoint of the line
 @param ly0 Y-coordinate of the first endpoint of the line
 @param lx1 X-coordinate of the second endpoint of the line
 @param ly1 Y-coordinate of the second endpoint of the line
 @return Signed distance (negative on the right, positive to the left)
*/
double PointToLineDistance2DSigned(double px, double py, double lx0, double ly0, double lx1, double ly1);

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
 @param x3 X-coordinate of the point to check
 @param y3 Y-coordinate of the point to check
 @param x1 X-coordinate of the first endpoint of the line
 @param y1 Y-coordinate of the first endpoint of the line
 @param x2 X-coordinate of the second endpoint of the line
 @param y2 Y-coordinate of the second endpoint of the line
 @param sNorm Reference parameter will contain normalized s value (0:1) in case of point inside line
  else the distance to point projected on extended line behind (negative) or in front (positive)
 @return true if projected point is inside line segment, else false
*/
bool PointInBetweenVectorEndpoints(double x3, double y3, double x1, double y1, double x2, double y2, double &sNorm);

/**
 Measure distance from point to edge. Strategy: Project the point on the edge interior or closest endpoint
 @param x3 X-coordinate of the point to check
 @param y3 Y-coordinate of the point to check
 @param x1 X-coordinate of the first endpoint of the edge
 @param y1 Y-coordinate of the first endpoint of the edge
 @param x2 X-coordinate of the second endpoint of the edge
 @param y2 Y-coordinate of the second endpoint of the edge
 @param x Return the X-coordinate of closest point on edge
 @param y Return the Y-coordinate of closest point on edge
 @return the distance
*/
double DistanceFromPointToEdge2D(double x3, double y3, double x1, double y1, double x2, double y2, double* x, double* y);

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
  Convert target (x,y) coordinates to coordinate system of the host
*/
void Global2LocalCoordinates(double xTargetGlobal, double yTargetGlobal,
							 double xHostGlobal, double yHostGlobal, double angleHost,
							 double &targetXforHost, double &targetYforHost);

/**
  Convert target (x,y) coordinates to the global coordinate system
*/
void Local2GlobalCoordinates(double &xTargetGlobal, double &yTargetGlobal,
							 double xHostGlobal, double yHostGlobal, double thetaGlobal,
							 double targetXforHost, double targetYforHost);

/**
  Normalize a 2D vector
*/
void NormalizeVec2D(double x, double y, double &xn, double &yn);

/**
  Find parallel line at specified offset (- means left, + right)
*/
void OffsetVec2D(double x0, double y0, double x1, double y1, double offset, double& xo0, double& yo0, double& xo1, double& y01);

/**
  Get Euler angles in local coordinates after rotation Z0 * Y * Z1 (heading, pitch, heading)
*/
void ZYZ2EulerAngles(double z0, double y, double z1, double& h, double& p, double& r);

/**
  Get Euler angles in local coordinates after rotation Z0 * Y * Z1 (heading, pitch, heading)
*/
void R0R12EulerAngles(double h0, double p0, double r0, double h1, double p1, double r1, double& h, double& p, double& r);


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
std::string FileNameExtOf(const std::string& fname);
std::string FileNameWithoutExtOf(const std::string& fname);

int strtoi(std::string s);
double strtod(std::string s);

// Global Logger class
class Logger
{
public:
	typedef void(*FuncPtr)(const char*);

	static Logger& Inst();
	void Log(bool quit, bool trace, char const* func, char const* file, int line, char const* format, ...);
	void SetCallback(FuncPtr callback);
	bool IsCallbackSet();
	void SetTimePtr(double* timePtr) { time_ = timePtr; }
	void OpenLogfile();
	void LogVersion();
	bool IsFileOpen() { return file_.is_open(); }

private:
	Logger();
	~Logger();

	FuncPtr callback_;
	std::ofstream file_;
	double* time_; // seconds
};

// Global Vehicle Data Logger
class CSV_Logger
{
public:
	typedef void(*FuncPtr)(const char*);

	//Instantiator
	static CSV_Logger& InstVehicleLog(std::string scenario_filename,
		int numvehicles, std::string csv_filename);

	//Logging function called by VehicleLogger object using pass by value
	void LogVehicleData(bool isendline, double timestamp, char const* name_, int id_, double speed_,
		double wheel_angle_, double wheel_rot_, double posX_, double posY_, double posZ_, double accX_, double accY_,
		double accZ_, double distance_road_, double distance_lanem_, double heading_, double heading_rate_,
		double heading_angle_, double heading_angle_driving_direction_, double pitch_, double curvature_, ...);

	void SetCallback(FuncPtr callback);

private:
	//Constructor to be called by instantiator
	CSV_Logger(std::string scenario_filename, int numvehicles, std::string csv_filename);

	//Destructor
	~CSV_Logger();

	//Counter for indexing each log entry
	int data_index_;

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
	std::string default_value_;

	SE_Option(std::string opt_str, std::string opt_desc, std::string opt_arg = "") :
		opt_str_(opt_str), opt_desc_(opt_desc), opt_arg_(opt_arg), set_(false), arg_value_("") {}

	SE_Option(std::string opt_str, std::string opt_desc, std::string opt_arg, std::string default_value) :
		opt_str_(opt_str), opt_desc_(opt_desc), opt_arg_(opt_arg), set_(false), arg_value_(""), default_value_(default_value) {}

	void Usage();
};

class SE_Options
{
#define OPT_PREFIX "--"

public:
	void AddOption(std::string opt_str, std::string opt_desc, std::string opt_arg = "");
	void AddOption(std::string opt_str, std::string opt_desc, std::string opt_arg, std::string opt_arg_default_value);

	void PrintUsage();
	void PrintArgs(int argc, char *argv[], std::string message = "Unrecognized arguments:");
	bool GetOptionSet(std::string opt);
	bool IsOptionArgumentSet(std::string opt);
	std::string GetOptionArg(std::string opt);
	void ParseArgs(int *argc, char* argv[]);
	std::vector<std::string>& GetOriginalArgs() { return originalArgs_; }

private:
	std::vector<SE_Option> option_;
	std::string app_name_;
	std::vector<std::string> originalArgs_;

	SE_Option *GetOption(std::string opt);
};

class SE_SystemTime
{
public:
	__int64 start_time_;

	SE_SystemTime() : start_time_(SE_getSystemTime()) {}
	void Reset() { start_time_ = SE_getSystemTime(); }
	double GetS() { return 1E-3 * (SE_getSystemTime() - start_time_); }
};

class SE_SystemTimer
{
public:
	__int64 start_time_;
	double duration_;

	SE_SystemTimer() : start_time_(0), duration_(0) {}
	void Start()
	{
		start_time_ = SE_getSystemTime();
	}
	void Start(double duration)
	{
		start_time_ = SE_getSystemTime();
		duration_ = duration;
	}

	void Reset() { start_time_ = 0; }
	bool Started() { return start_time_ > 0 ? true : false; }
	void SetDuration(double duration) { duration_ = duration; }
	double Elapsed() { return 1E-3 * (SE_getSystemTime() - start_time_); }
	double Remaining()
	{
		if (Expired())
		{
			return 0;
		}
		else
		{
			return duration_ - Elapsed();
		}
	}
	bool Expired() { return Elapsed() > duration_; }
};


class SE_SimulationTimer
{
public:
	double start_time_;
	double duration_;

	SE_SimulationTimer() : start_time_(0), duration_(0) {}
	void Start(double timestamp_s, double duration)
	{
		start_time_ = timestamp_s;
		duration_ = duration;
	}

	void Reset()
	{
		start_time_ = 0;
		duration_ = 0;
	}

	double Remaining(double timestamp_s)
	{
		if (Expired(timestamp_s))
		{
			return 0;
		}
		else
		{
			return duration_ - Elapsed(timestamp_s);
		}
	}

	bool Started() { return duration_ > SMALL_NUMBER; }
	double Elapsed(double timestamp_s) { return timestamp_s - start_time_; }
	double Expired(double timestamp_s) { return timestamp_s - start_time_ > duration_; }
	double GetDuration() { return duration_; }
};

class DampedSpring
{
public:
	// Custom damping factor, set 0 for no damping
	DampedSpring() : x_(0), x0_(0), t_(0), d_(0), v_(0), a_(0), critical_(false) {};

	// Custom damping factor, set 0 for no damping
	DampedSpring(double startValue, double targetValue, double tension, double damping) :
		x_(startValue), x0_(targetValue), t_(tension), d_(damping), v_(0), a_(0), critical_(false) {};

	// Critically damped (optimal)
	DampedSpring(double startValue, double targetValue, double tension) :
		x_(startValue), x0_(targetValue), t_(tension), d_(2 * sqrt(tension)), v_(0), a_(0), critical_(true) {};

	void Update(double timeStep)
	{
		a_ = -t_ * (x_ - x0_) - d_ * v_;
		v_ = v_ + a_ * timeStep;
		x_ = x_ + v_ * timeStep;
	};

	void SetValue(double value) { x_ = value; }
	double GetValue() { return x_; }
	double GetV() { return v_; }
	double GetA() { return a_; }
	void SetV(double value) { v_ = value; }
	void SetTargetValue(double targetValue) { x0_ = targetValue; }
	double GetTargetValue() { return x0_; }

	void SetTension(double tension)
	{
		t_ = tension;
		if (critical_)
		{
			d_ = 2 * sqrt(t_);
		}
	}

	void SetDamping(double damping)
	{
		d_ = damping;
		critical_ = false;
	}

	void SetOptimalDamping()
	{
		d_ = 2 * sqrt(t_);
		critical_ = true;
	}

private:
	double x0_;
	double x_;
	double v_;
	double a_;
	double t_;
	double d_;
	bool critical_;
};

class SE_Env
{
public:
	static SE_Env& Inst();
	std::vector<std::string> paths_;
	double osiMaxLongitudinalDistance_;
	double osiMaxLateralDeviation_;
	std::string logFilePath_;
	SE_SystemTime systemTime_;

	SE_Env() : osiMaxLongitudinalDistance_(OSI_MAX_LONGITUDINAL_DISTANCE),
		osiMaxLateralDeviation_(OSI_MAX_LATERAL_DEVIATION), logFilePath_(LOG_FILENAME) {}

	void SetOSIMaxLongitudinalDistance(double maxLongitudinalDistance) { osiMaxLongitudinalDistance_ = maxLongitudinalDistance; }
	void SetOSIMaxLateralDeviation(double maxLateralDeviation) { osiMaxLateralDeviation_ = maxLateralDeviation; }
	double GetOSIMaxLongitudinalDistance() { return osiMaxLongitudinalDistance_; }
	double GetOSIMaxLateralDeviation() { return osiMaxLateralDeviation_; }
	std::vector<std::string>& GetPaths() { return paths_; }
	int AddPath(std::string path);
	void ClearPaths() { paths_.clear(); }
	double GetSystemTime() { return systemTime_.GetS(); }

	/**
		Specify logfile name, optionally including directory path
		examples: "../logfile.txt" "c:/tmp/esmini.log" "my.log"
		Set "" to disable logfile
		@param path Logfile path
	*/
	void SetLogFilePath(std::string logFilePath);
	std::string GetLogFilePath() { return logFilePath_; }
};
