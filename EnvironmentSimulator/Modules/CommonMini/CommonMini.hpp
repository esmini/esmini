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
#include <random>
#include <fstream>
#include <string>
#define _USE_MATH_DEFINES
#include <math.h>
#include <condition_variable>
#include <cstring>
#include <map>

#ifndef _WIN32
#include <inttypes.h>
typedef int64_t __int64;
#else
#include <sdkddkver.h>
#endif

#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7)
#define snprintf _snprintf
#endif

#ifdef _WIN32
#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#else
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif

#define SMALL_NUMBER                  (1E-6)
#define SMALL_NUMBERF                 (1E-6f)
#define LARGE_NUMBER                  (1E+10)
#define LARGE_NUMBERF                 (1E+10f)
#define LARGE_NUMBER_INT              (0x7fffffff)  // largest signed 32 bit integer number
#define SIGN(X)                       ((X < 0) ? -1 : 1)
#define MAX(x, y)                     ((y) > (x) ? (y) : (x))
#define MIN(x, y)                     ((y) < (x) ? (y) : (x))
#define ABS_LIMIT(x, y)               (abs(x) > abs(y) ? (SIGN(x) * abs(y)) : x)  // limit abs value but keep sign
#define ABS_FLOOR(x, y)               (abs(x) < abs(y) ? (SIGN(x) * abs(y)) : x)  // limit abs value but keep sign
#define CLAMP(x, lo, hi)              MIN(hi, MAX(lo, x))
#define AVOID_ZERO(x)                 (SIGN(x) * MAX(SMALL_NUMBER, fabs(x)))
#define NEAR_ZERO(x)                  (abs(x) < SMALL_NUMBER)
#define NEAR_NUMBERS(x, y)            (abs(x - y) < SMALL_NUMBER)
#define NEAR_NUMBERSF(x, y)           (abs(x - y) < SMALL_NUMBERF)
#define IS_IN_SPAN(x, y, z)           ((x) >= (y) && (x) <= (z))
#define OSI_MAX_LONGITUDINAL_DISTANCE 50
#define OSI_MAX_LATERAL_DEVIATION     0.05
#define LOG_FILENAME                  "log.txt"
#define DAT_FILENAME                  "sim.dat"
#define GHOST_TRAIL_SAMPLE_TIME       0.2

#define LOG(...)       Logger::Inst().Log(false, false, __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__)
#define LOG_TRACE(...) Logger::Inst().Log(false, true, __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__)
#define LOG_ONCE(...)                                                                            \
    {                                                                                            \
        static bool firstTime = true;                                                            \
        if (firstTime)                                                                           \
        {                                                                                        \
            Logger::Inst().Log(false, false, __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); \
            firstTime = false;                                                                   \
        }                                                                                        \
    }
#define LOG_TRACE_ONCE(...)                                                                     \
    {                                                                                           \
        static bool firstTime = true;                                                           \
        if (firstTime)                                                                          \
        {                                                                                       \
            Logger::Inst().Log(false, true, __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); \
            firstTime = false;                                                                  \
        }                                                                                       \
    }
#define LOG_AND_QUIT(...)       Logger::Inst().Log(true, false, __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__)
#define LOG_TRACE_AND_QUIT(...) Logger::Inst().Log(true, true, __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__)

// Time functions
__int64 SE_getSystemTime();
void    SE_sleep(unsigned int msec);
double  SE_getSimTimeStep(__int64& time_stamp, double min_time_step, double max_time_step);

// Useful types
enum class KeyType  // copy key enums from OSG GUIEventAdapter
{
    KEY_Left   = 0xFF51, /* Left arrow */
    KEY_Up     = 0xFF52, /* Up arrow */
    KEY_Right  = 0xFF53, /* Right arrow */
    KEY_Down   = 0xFF54, /* Down arrow */
    KEY_Space  = 0x20,   /* Space */
    KEY_Return = 0xFF0D, /* Return, enter */
    KEY_Escape = 0xFF1B, /* Escape */

    // Mod key types
    KEY_Shift_L   = 0xFFE1, /* Left shift */
    KEY_Shift_R   = 0xFFE2, /* Right shift */
    KEY_Control_L = 0xFFE3, /* Left control */
    KEY_Control_R = 0xFFE4, /* Right control */
};

enum class ModKeyMask
{
    MODKEY_LEFT_SHIFT  = 0x0001,
    MODKEY_RIGHT_SHIFT = 0x0002,
    MODKEY_LEFT_CTRL   = 0x0004,
    MODKEY_RIGHT_CTRL  = 0x0008,
    MODKEY_CTRL        = (MODKEY_LEFT_CTRL | MODKEY_RIGHT_CTRL),
    MODKEY_SHIFT       = (MODKEY_LEFT_SHIFT | MODKEY_RIGHT_SHIFT),
};

enum class ControlDomains
{
    DOMAIN_NONE         = 0,
    DOMAIN_LONG         = 1,
    DOMAIN_LAT          = 1 << 1,
    DOMAIN_LIGHT        = 1 << 2,
    DOMAIN_ANIM         = 1 << 3,
    DOMAIN_LAT_AND_LONG = DOMAIN_LAT | DOMAIN_LONG,                               // = 3 (1+2)
    DOMAIN_ALL          = DOMAIN_LAT | DOMAIN_LONG | DOMAIN_LIGHT | DOMAIN_ANIM,  // = 15 (1+2+4+8)
};

enum class ControlOperationMode
{
    MODE_NONE     = 0,  // Controller not available or it is not active
    MODE_OVERRIDE = 1,  // Actions from the scenario are not applied, default
    MODE_ADDITIVE = 2,  // Actions from the scenario are applied
};

enum class ControlActivationMode
{
    UNDEFINED = 0,
    OFF       = 1,
    ON        = 2
};

enum class EntityScaleMode
{
    NONE,
    BB_TO_MODEL,  // Scale bounding box to 3D model
    MODEL_TO_BB,  // Scale 3D model to specified or generated bounding box
};

enum class FollowingMode
{
    FOLLOW,
    POSITION
};

std::string ControlDomain2Str(unsigned int domains);

enum class PixelFormat
{
    UNSPECIFIED = 0,
    RGB         = 0x1907,  // GL_RGB
    BGR         = 0x80E0   // GL_BGR
};

struct OffScreenImage
{
    int            width;
    int            height;
    int            pixelSize;
    int            pixelFormat;  // According to enum class PixelFormat
    unsigned char* data;
};

enum class GhostMode
{
    NORMAL,
    RESTART,    // the frame ghost is requested to restart
    RESTARTING  // ghost restart is ongoing, including the final restart timestep
};

class SE_Vector
{
public:
    SE_Vector() : x_(0.0), y_(0.0)
    {
    }
    SE_Vector(double x, double y) : x_(x), y_(y)
    {
    }

    void SetX(double x)
    {
        x_ = x;
    }
    void SetY(double y)
    {
        y_ = y;
    }
    void Set(double x, double y)
    {
        x_ = x, y_ = y;
    }

    SE_Vector operator+(SE_Vector const& p) const
    {
        SE_Vector res;
        res.x_ = x_ + p.x_;
        res.y_ = y_ + p.y_;
        return res;
    }

    SE_Vector operator-(SE_Vector const& p) const
    {
        SE_Vector res;
        res.x_ = x_ - p.x_;
        res.y_ = y_ - p.y_;
        return res;
    }

    SE_Vector& operator+=(SE_Vector const& p)
    {
        this->x_ += p.x_;
        this->y_ += p.y_;
        return *this;
    }

    SE_Vector& operator-=(SE_Vector const& p)
    {
        this->x_ -= p.x_;
        this->y_ -= p.y_;
        return *this;
    }

    SE_Vector Rotate(double angle)
    {
        SE_Vector res;
        res.x_ = x_ * cos(angle) - y_ * sin(angle);
        res.y_ = x_ * sin(angle) + y_ * cos(angle);
        return res;
    }

    double Dot(const SE_Vector& corner) const
    {
        return x_ * corner.x_ + y_ * corner.y_;
    }

    double Cross(const SE_Vector& vec) const
    {
        return x_ * vec.y_ - vec.x_ * y_;
    }

    double GetLength()
    {
        double l2 = x_ * x_ + y_ * y_;
        if (l2 > SMALL_NUMBER)
        {
            return sqrt(l2);
        }

        return 0.0;
    }

    void SetLength(double length)
    {
        double current_length = GetLength();
        if (current_length > SMALL_NUMBER)
        {
            x_ *= length / current_length;
            y_ *= length / current_length;
        }
        else
        {
            x_ = 0.0;
            y_ = 0.0;
        }
    }

    void Normalize()
    {
        double len = GetLength();
        if (len < SMALL_NUMBER)
        {
            len = SMALL_NUMBER;
        }
        x_ = x_ / len;
        y_ = y_ / len;
    }

    double x() const
    {
        return x_;
    }
    double y() const
    {
        return y_;
    }

private:
    double x_;
    double y_;
};

// Useful operations

/**
        Get model filename from model_id.
*/
std::string GetModelFilenameById(int model_id);

/**
        Checks whether file with given path exists or not
*/
bool FileExists(const char* fileName);

/**
        Concatenate a directory path and a file path
*/
std::string CombineDirectoryPathAndFilepath(std::string dir_path, std::string file_path);

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
        Retrieve the angle in the interval [-PI, PI]
*/
double GetAngleInIntervalMinusPIPlusPI(double angle);

/**
        Retrieve difference between two angles
*/
double GetAngleDifference(double angle1, double angle2);

/**
        This function checks if the angle is in quad 1 or 4, i.e. not in range [pi/2, 3pi/3]
*/
bool IsAngleForward(double teta);

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
int GetIntersectionOfTwoLineSegments(double  ax1,
                                     double  ay1,
                                     double  ax2,
                                     double  ay2,
                                     double  bx1,
                                     double  by1,
                                     double  bx2,
                                     double  by2,
                                     double& x3,
                                     double& y3);

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
        Project a 2D point on a 2D line (specified start- and endpoint)
        Project a 2D point on a 2D vector (from origin to specified point)
        @param x X coordinate of point
        @param y Y coordinate of point
        @param vx1 X coordinate of line start point
        @param vy1 Y coordinate of line start point
        @param vx2 X coordinate of line end point
        @param vy2 Y coordinate of line end point
        @param px X coordinate of projected point (reference parameter)
        @param py Y coordinate of projected point (reference parameter)
*/
void ProjectPointOnLine2D(double x, double y, double vx1, double vy1, double vx2, double vy2, double& px, double& py);

/**
        Project a 2D point on a 2D vector (from origin to specified point)
        @param x0 X coordinate of point
        @param y0 Y coordinate of point
        @param x1 X coordinate of vector
        @param y1 Y coordinate of vector
        @param px X coordinate of projected point (reference parameter)
        @param py Y coordinate of projected point (reference parameter)
*/
void ProjectPointOnVector2D(double x0, double y0, double x1, double y1, double& px, double& py);

/**
        Project a 2D point on a 2D vector (from origin to specified point). Return signed length of resultant.
        @param x0 X coordinate of point
        @param y0 Y coordinate of point
        @param x1 X coordinate of vector
        @param y1 Y coordinate of vector
        @param px X coordinate of projected point (reference parameter)
        @param py Y coordinate of projected point (reference parameter)
        @return Length of projected point relative given vector, negative if opposite direction
*/
double ProjectPointOnVector2DSignedLength(double x0, double y0, double x1, double y1, double& px, double& py);

/**
        Check whether projected point is in area formed by the two given vectors
        @param p Point to check
        @param l0p0 First point of the first vector
        @param l0p1 Second point of the first vector
        @param l1p0 First point of the second vector
        @param l1p1 Second point of the second vector
        @param sNorm Reference parameter will contain normalized s value (0:1) in case of point inside the area, positive
        if the point is on the right hemisphere from the first vector, else negative. If point is not between vectors
        the distance to closest vector. Negative if closer to first vector, positive if closer to the second vector.
        @return true if projected point is inside the area, else false
*/
bool IsPointWithinSectorBetweenTwoLines(SE_Vector p, SE_Vector l0p0, SE_Vector l0p1, SE_Vector l1p0, SE_Vector l1p1, double& sNorm);

/**
        Check whether projected point is in between vector endpoints, or outside
        NOTE: Assumes the x3, y3 point to be ON the line (x1,y1 - x2,y2)
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
bool PointInBetweenVectorEndpoints(double x3, double y3, double x1, double y1, double x2, double y2, double& sNorm);

/**
        Measure distance from point to edge. Strategy: Project the point on the edge interior or closest endpoint
        @param x3 X-coordinate of the point to check
        @param y3 Y-coordinate of the point to check
        @param x1 X-coordinate of the first endpoint of the edge
        @param y1 Y-coordinate of the first endpoint of the edge
        @param x2 X-coordinate of the second endpoint of the edge
        @param y2 Y-coordinate of the second endpoint of the edge
        @param x Return the X-coordinate of closest point on edge (set = 0 to skip)
        @param y Return the Y-coordinate of closest point on edge (set = 0 to skip)
        @return the distance
*/
double DistanceFromPointToEdge2D(double x3, double y3, double x1, double y1, double x2, double y2, double* x, double* y);

/**
        Measure distance from point to line given by two points.
        Strategy: Find and measure distance to closest/perpendicular point on line
        @param x3 X-coordinate of the point to check
        @param y3 Y-coordinate of the point to check
        @param x1 X-coordinate of the first point on line
        @param y1 Y-coordinate of the first point on line
        @param x2 X-coordinate of the second first point on line
        @param y2 Y-coordinate of the second first point on line
        @param x Return the X-coordinate of closest point on line
        @param y Return the Y-coordinate of closest point on line
        @return the distance
*/
double DistanceFromPointToLine2D(double x3, double y3, double x1, double y1, double x2, double y2, double* x, double* y);

/**
        Measure distance from point to line given by point and angle.
        Strategy: Find and measure distance to closest/perpendicular point on line
        @param x3 X-coordinate of the point to check
        @param y3 Y-coordinate of the point to check
        @param x1 X-coordinate of a point on the line
        @param y1 Y-coordinate of a point on the line
        @param angle angle of the line
        @return the distance
*/
double DistanceFromPointToLine2DWithAngle(double x3, double y3, double x1, double y1, double angle);

/**
        Find out whether the point is left or right to a vector
*/
int PointSideOfVec(double px, double py, double vx1, double vy1, double vx2, double vy2);

/**
        Retrieve the length of a 2D line segment
*/
double GetLengthOfLine2D(double x1, double y1, double x2, double y2);

/**
        Retrieve the length of a 2D vector
*/
double GetLengthOfVector2D(double x, double y);

/**
        Retrieve the length of a 3D vector
*/
double GetLengthOfVector3D(double x, double y, double z);

/**
        Rotate a 2D vector
*/
void RotateVec2D(double x, double y, double angle, double& xr, double& yr);

/**
        Convert target (x,y) coordinates to coordinate system of the host
*/
void Global2LocalCoordinates(double  xTargetGlobal,
                             double  yTargetGlobal,
                             double  xHostGlobal,
                             double  yHostGlobal,
                             double  angleHost,
                             double& targetXforHost,
                             double& targetYforHost);

/**
        Convert target (x,y) coordinates to the global coordinate system
*/
void Local2GlobalCoordinates(double& xTargetGlobal,
                             double& yTargetGlobal,
                             double  xHostGlobal,
                             double  yHostGlobal,
                             double  thetaGlobal,
                             double  targetXforHost,
                             double  targetYforHost);

/**
        Normalize a 2D vector
*/
void NormalizeVec2D(double x, double y, double& xn, double& yn);

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

int InvertMatrix3(const double m[3][3], double m_out[3][3]);

/**
        Get Euler angles for the orientation relative road surface orientation
        Rotation_rel = Rotation_road_inverse * Rotation_abs
*/
void CalcRelAnglesFromRoadAndAbsAngles(double  h_road,
                                       double  p_road,
                                       double  r_road,
                                       double  h_abs,
                                       double  p_abs,
                                       double  r_abs,
                                       double& h_rel,
                                       double& p_rel,
                                       double& r_rel);

void MultMatrixVector3d(const double m[3][3], const double v0[3], double v1[3]);
void MultMatrixMatrix3d(const double m0[3][3], const double m1[3][3], double m_out[3][3]);

void RotateVec3d(const double h0,
                 const double p0,
                 const double r0,
                 const double x0,
                 const double y0,
                 const double z0,
                 double&      x1,
                 double&      y1,
                 double&      z1);

/**
        Change byte order - can be useful for IP communication with non Intel platforms
*/
void SwapByteOrder(unsigned char* buf, int data_type_size, int buf_size);

#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7)
#else
#include <thread>
#include <mutex>
#endif

class SE_Thread
{
public:
#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7 || __MINGW32__)
    SE_Thread() : thread_()
    {
    }
#else
    SE_Thread()
    {
    }
#endif
    ~SE_Thread();

    void Wait();
    void Start(void (*func_ptr)(void*), void* arg);

private:
#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7 || __MINGW32__)
    void* thread_;
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
#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7 || __MINGW32__)
    void* mutex_;
#else
    std::mutex  mutex_;
#endif
};

// Semaphore implementation based on this "thread":
// https://stackoverflow.com/questions/4792449/c0x-has-no-semaphores-how-to-synchronize-threads/4793662#4793662
// But adapted for use cases where a thread should wait until a series, not necessarily overlapping, tasks from
// potentially various threads has been finished, e.g:
// 1. Raise flag from main thread when main application thread initiates rendering of scenario image
// 2. Lower flag from render thread when image has been created and copied into RAM
// 3. Meanwhile, main thread waits for the flag to fall and then can fetch the image
//
class SE_Semaphore
{
public:
    SE_Semaphore() : flag(0)
    {
    }

    inline void Set()
    {
#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7)
#elif (defined __MINGW32__)
#else
        std::unique_lock<std::mutex> lock(mtx);
#endif
        flag = true;
    }

    inline void Release()
    {
#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7 || __MINGW32__)
        flag = false;
#else
        std::unique_lock<std::mutex> lock(mtx);
        flag = false;
        cv.notify_one();  // notify the waiting thread
#endif
    }

    inline void Wait()
    {
#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7 || __MINGW32__)
#else
        std::unique_lock<std::mutex> lock(mtx);
        if (flag == true)
        {
            cv.wait(lock);  // wait on the mutex until notify is called
        }
#endif
    }

private:
#if (defined WINVER && WINVER == _WIN32_WINNT_WIN7 || __MINGW32__)
#else
    std::mutex              mtx;
    std::condition_variable cv;
#endif
    bool flag;
};

std::vector<std::string> SplitString(const std::string& s, char separator);
std::string              DirNameOf(const std::string& fname);
std::string              FileNameOf(const std::string& fname);
bool                     IsDirectoryName(const std::string& string);
std::string              FileNameExtOf(const std::string& fname);
std::string              FileNameWithoutExtOf(const std::string& fname);
std::string              FilePathWithoutExtOf(const std::string& fpath);
std::string              ToLower(const std::string in_str);
std::string              ToLower(const char* in_str);
FILE*                    FileOpen(const char* filename, const char* mode);

int    strtoi(std::string s);
double strtod(std::string s);

/**
        Copy string
        @param dest Destination buffer (resulting copy)
        @param src Source buffer (original string)
        @param size Maximum character written, including optional added terminating null (see terminate parameter)
        @param terminate If true the dest will always be terminated, at latest at dest[size-1]
*/
void StrCopy(char* dest, const char* src, size_t size, bool terminate = true);

// Global Logger class
class Logger
{
public:
    typedef void (*FuncPtr)(const char*);

    static Logger& Inst();
    void           Log(bool quit, bool trace, const char* func, const char* file, int line, const char* format, ...);
    void           SetCallback(FuncPtr callback);
    bool           IsCallbackSet();
    void           SetTimePtr(double* timePtr)
    {
        time_ = timePtr;
    }
    void OpenLogfile(std::string filename);
    void CloseLogFile();
    void LogVersion();
    bool IsFileOpen()
    {
        return file_.is_open();
    }

private:
    Logger();
    ~Logger();

    SE_Mutex      mutex_;
    FuncPtr       callback_;
    std::ofstream file_;
    double*       time_;  // seconds
};

// Global Vehicle Data Logger
class CSV_Logger
{
public:
    typedef void (*FuncPtr)(const char*);

    // Instantiator
    static CSV_Logger& Inst();

    // Logging function called by VehicleLogger object using pass by value
    void LogVehicleData(bool        isendline,
                        double      timestamp,
                        char const* name,
                        int         id,
                        double      speed,
                        double      wheel_angle,
                        double      wheel_rot,
                        double      bb_x,
                        double      bb_y,
                        double      bb_z,
                        double      bb_length,
                        double      bb_width,
                        double      bb_height,
                        double      posX,
                        double      posY,
                        double      posZ,
                        double      velX,
                        double      velY,
                        double      velZ,
                        double      accX,
                        double      accY,
                        double      accZ,
                        double      distance_road,   // s (longitudinal distance along current road)
                        double      distance_lanem,  // t (lateral offset from reference lane)
                        int         lane_id,
                        double      lane_offset,  // lateral offset from current lane center
                        double      heading,
                        double      heading_rate,
                        double      heading_angle,
                        double      heading_angle_driving_direction,
                        double      pitch,
                        double      curvature,
                        char const* collisions,
                        ...);

    void SetCallback(FuncPtr callback);
    void Open(std::string scenario_filename, int numvehicles, std::string csv_filename);

private:
    // Constructor to be called by instantiator
    CSV_Logger();

    // Destructor
    ~CSV_Logger();

    // Counter for indexing each log entry
    int data_index_;

    // File output stream
    std::ofstream file_;

    // Callback function pointer for error logging
    FuncPtr callback_;
};

// Argument parser

class SE_Option
{
public:
    std::string              opt_str_;
    std::string              opt_desc_;
    std::string              opt_arg_;
    bool                     set_;
    std::vector<std::string> arg_value_;
    std::string              default_value_;

    SE_Option(std::string opt_str, std::string opt_desc, std::string opt_arg = "")
        : opt_str_(opt_str),
          opt_desc_(opt_desc),
          opt_arg_(opt_arg),
          set_(false)
    {
    }

    SE_Option(std::string opt_str, std::string opt_desc, std::string opt_arg, std::string default_value)
        : opt_str_(opt_str),
          opt_desc_(opt_desc),
          opt_arg_(opt_arg),
          set_(false),
          default_value_(default_value)
    {
    }

    void Usage();
};

class SE_Options
{
#define OPT_PREFIX "--"

public:
    void AddOption(std::string opt_str, std::string opt_desc, std::string opt_arg = "");
    void AddOption(std::string opt_str, std::string opt_desc, std::string opt_arg, std::string opt_arg_default_value);

    void                      PrintUsage();
    void                      PrintUnknownArgs(std::string message = "Unrecognized arguments:");
    bool                      GetOptionSet(std::string opt);
    bool                      IsOptionArgumentSet(std::string opt);
    std::string               GetOptionArg(std::string opt, int index = 0);
    int                       ParseArgs(int argc, const char* const argv[]);
    std::vector<std::string>& GetOriginalArgs()
    {
        return originalArgs_;
    }
    bool IsInOriginalArgs(std::string opt);
    bool HasUnknownArgs();
    void Reset();
    int  ChangeOptionArg(std::string opt, std::string new_value, int index = 0);

private:
    std::vector<SE_Option>   option_;
    std::string              app_name_;
    std::vector<std::string> originalArgs_;
    std::vector<std::string> unknown_args_;

    SE_Option* GetOption(std::string opt);
};

class SE_SystemTime
{
public:
    __int64 start_time_;

    SE_SystemTime() : start_time_(SE_getSystemTime())
    {
    }
    void Reset()
    {
        start_time_ = SE_getSystemTime();
    }
    double GetS()
    {
        return 1E-3 * static_cast<double>((SE_getSystemTime() - start_time_));
    }
};

class SE_SystemTimer
{
public:
    __int64 start_time_;
    double  duration_;

    SE_SystemTimer() : start_time_(0), duration_(0)
    {
    }
    void Start()
    {
        start_time_ = SE_getSystemTime();
    }
    void Start(double duration)
    {
        start_time_ = SE_getSystemTime();
        duration_   = duration;
    }

    void Reset()
    {
        start_time_ = 0;
    }
    bool Started()
    {
        return start_time_ > 0 ? true : false;
    }
    void SetDuration(double duration)
    {
        duration_ = duration;
    }
    double Elapsed()
    {
        return 1E-3 * static_cast<double>((SE_getSystemTime() - start_time_));
    }
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
    bool Expired()
    {
        return Elapsed() > duration_ - SMALL_NUMBER;
    }
};

class SE_SimulationTimer
{
public:
    double start_time_;
    double duration_;

    SE_SimulationTimer() : start_time_(0), duration_(0)
    {
    }
    void Start(double timestamp_s, double duration)
    {
        start_time_ = timestamp_s;
        duration_   = duration;
    }

    void Reset()
    {
        start_time_ = 0;
        duration_   = 0;
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

    bool Started()
    {
        return duration_ > SMALL_NUMBER;
    }
    double Elapsed(double timestamp_s)
    {
        return timestamp_s - start_time_;
    }
    bool Expired(double timestamp_s)
    {
        return timestamp_s - start_time_ > duration_ - SMALL_NUMBER;
    }
    double GetDuration()
    {
        return duration_;
    }
};

class DampedSpring
{
public:
    // Custom damping factor, set 0 for no damping
    DampedSpring() : x_(0), x0_(0), t_(0), d_(0), v_(0), a_(0), critical_(false){};

    // Custom damping factor, set 0 for no damping
    DampedSpring(double startValue, double targetValue, double tension, double damping)
        : x_(startValue),
          x0_(targetValue),
          t_(tension),
          d_(damping),
          v_(0),
          a_(0),
          critical_(false){};

    // Critically damped (optimal)
    DampedSpring(double startValue, double targetValue, double tension)
        : x_(startValue),
          x0_(targetValue),
          t_(tension),
          d_(2 * sqrt(tension)),
          v_(0),
          a_(0),
          critical_(true){};

    void Update(double timeStep)
    {
        a_ = -t_ * (x_ - x0_) - d_ * v_;
        v_ = v_ + a_ * timeStep;
        x_ = x_ + v_ * timeStep;
    };

    void SetValue(double value)
    {
        x_ = value;
    }
    double GetValue()
    {
        return x_;
    }
    double GetV()
    {
        return v_;
    }
    double GetA()
    {
        return a_;
    }
    void SetV(double value)
    {
        v_ = value;
    }
    void SetTargetValue(double targetValue)
    {
        x0_ = targetValue;
    }
    double GetTargetValue()
    {
        return x0_;
    }

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
        d_        = damping;
        critical_ = false;
    }

    void SetOptimalDamping()
    {
        d_        = 2 * sqrt(t_);
        critical_ = true;
    }

private:
    double x_;
    double x0_;
    double t_;
    double d_;
    double v_;
    double a_;
    bool   critical_;
};

class SE_Rand
{
public:
    SE_Rand()
    {
        seed_ = (std::random_device())();
        gen_.seed(seed_);
    }

    void SetSeed(unsigned int seed)
    {
        seed_ = seed;
        gen_.seed(seed_);
    }

    // Get an integer in the range (min, max) NOTE: including max
    int GetNumberBetween(int min, int max)
    {
        if (max < min)
        {
            return min;
        }

        return std::uniform_int_distribution<>{min, max}(gen_);
    }

    double GetReal()  // returns a floating point number between 0 and 1
    {
        return std::uniform_real_distribution<>{}(gen_);
    }

    double GetRealBetween(double min, double max)
    {
        return std::uniform_real_distribution<>{min, max}(gen_);
    }

    unsigned int GetSeed()
    {
        return seed_;
    }
    std::mt19937& GetGenerator()
    {
        return gen_;
    }

private:
    unsigned int seed_;
    std::mt19937 gen_;
};

class SE_Env
{
public:
    SE_Env()
        : osiMaxLongitudinalDistance_(OSI_MAX_LONGITUDINAL_DISTANCE),
          osiMaxLateralDeviation_(OSI_MAX_LATERAL_DEVIATION),
          logFilePath_(LOG_FILENAME),
          datFilePath_(""),
          osiFilePath_(""),
          osiFileEnabled_(false),
          collisionDetection_(false),
          saveImagesToRAM_(false),
          ghost_mode_(GhostMode::NORMAL),
          ghost_headstart_(0.0)
    {
    }

    static SE_Env& Inst();

    void SetOSIMaxLongitudinalDistance(double maxLongitudinalDistance)
    {
        osiMaxLongitudinalDistance_ = maxLongitudinalDistance;
    }
    void SetOSIMaxLateralDeviation(double maxLateralDeviation)
    {
        osiMaxLateralDeviation_ = maxLateralDeviation;
    }
    double GetOSIMaxLongitudinalDistance()
    {
        return osiMaxLongitudinalDistance_;
    }
    double GetOSIMaxLateralDeviation()
    {
        return osiMaxLateralDeviation_;
    }
    void SetCollisionDetection(bool enable)
    {
        collisionDetection_ = enable;
    }
    bool GetCollisionDetection()
    {
        return collisionDetection_;
    }
    std::vector<std::string>& GetPaths()
    {
        return paths_;
    }
    int  AddPath(std::string path);
    void ClearPaths()
    {
        paths_.clear();
    }
    double GetSystemTime()
    {
        return systemTime_.GetS();
    }

    /**
            Specify scenario logfile (.txt) file path,
            optionally including directory path and/or filename
            Specify only directory (end with "/" or "\") to let esmini set default filename
            Specify only filename (no leading "/" or "\") to let esmini set default directory
            Set "" to disable logfile
            examples:
              "../logfile.txt" (relative current directory)
              "c:/tmp/esmini.log" (absolute path)
              "my.log" (put it in current directory)
              "c:/tmp/" (use default filename)
              "" (prevent creation of logfile)
            Note: Needs to be called prior to calling SE_Init()
            @param path Logfile path
    */
    void        SetLogFilePath(std::string logFilePath);
    std::string GetLogFilePath()
    {
        return logFilePath_;
    }

    /**
            Specify scenario recording (.dat) file path,
            optionally including directory path and/or filename
            Specify only directory (end with "/" or "\") to let esmini set default filename
            Specify only filename (no leading "/" or "\") to let esmini set default directory
            Set "" to disable logfile
            examples:
              "../logfile.txt" (relative current directory)
              "c:/tmp/esmini.log" (absolute path)
              "my.log" (put it in current directory)
              "c:/tmp/" (use default filename)
              "" (prevent creation of logfile)
            Note: Needs to be called prior to calling SE_Init()
            @param path Logfile path
    */
    void        SetDatFilePath(std::string datFilePath);
    std::string GetDatFilePath()
    {
        return datFilePath_;
    }

    /**
        Set flag controlling whether rendered images are saved and hence can be fetched
        @param state true/false
    */
    void SaveImagesToRAM(bool state)
    {
        saveImagesToRAM_ = state;
    }

    bool GetSaveImagesToRAM()
    {
        return saveImagesToRAM_;
    }

    void        EnableOSIFile(std::string osiFilePath);
    void        DisableOSIFile();
    std::string GetOSIFilePath()
    {
        return osiFilePath_;
    }
    bool GetOSIFileEnabled()
    {
        return osiFileEnabled_;
    }

    std::string GetModelFilenameById(int model_id);
    void        ClearModelFilenames()
    {
        entity_model_map_.clear();
    }

    SE_Rand& GetRand()
    {
        return rand_;
    }

    GhostMode GetGhostMode()
    {
        return ghost_mode_;
    }

    void SetGhostMode(GhostMode mode)
    {
        ghost_mode_ = mode;
    }

    double GetGhostHeadstart(void)
    {
        return ghost_headstart_;
    }

    void SetGhostHeadstart(double headstart_time)
    {
        ghost_headstart_ = headstart_time;
    }

    SE_Options& GetOptions()
    {
        return opt;
    };

private:
    std::vector<std::string>   paths_;
    double                     osiMaxLongitudinalDistance_;
    double                     osiMaxLateralDeviation_;
    std::string                logFilePath_;
    std::string                datFilePath_;
    std::string                osiFilePath_;
    bool                       osiFileEnabled_;
    SE_SystemTime              systemTime_;
    SE_Rand                    rand_;
    bool                       collisionDetection_;
    bool                       saveImagesToRAM_;
    std::map<int, std::string> entity_model_map_;
    GhostMode                  ghost_mode_;
    double                     ghost_headstart_;
    SE_Options                 opt;
};

/**
        Store RGB (3*8 bits color values) image data as a PPM image file
        PPM info: http://paulbourke.net/dataformats/ppm/
        @param filename File name including extension which should be ".ppm", e.g. "img0.ppm"
        @param width Width
        @param height Height
        @param rgbData Array of color values
        @param pixelSize 3 for RGB/BGR
        @param pixelFormat 0=Unspecified, 0x1907=RGB (GL_RGB), 0x80E0=BGR (GL_BGR)
        @param upsidedown false=lines stored from top to bottom, true=lines stored from bottom to top
        @return 0 if OK, -1 if failed to open file, -2 if unexpected pixelSize
*/
int SE_WritePPM(const char* filename, int width, int height, const unsigned char* data, int pixelSize, int pixelFormat, bool upsidedown);

/**
        Store RGB or BGR (3*8 bits color values) image data as a TGA image file
        TGA spec: https://www.dca.fee.unicamp.br/~martino/disciplinas/ea978/tgaffs.pdf
        TGA brief: http://paulbourke.net/dataformats/tga/
        @param filename File name including extension which should be ".tga", e.g. "img0.tga"
        @param width Width
        @param height Height
        @param rgbData Array of color values
        @param pixelSize 3 (RGB) or 4 (RGBA)
        @param pixelFormat 0=Unspecified, 0x1907=RGB (GL_RGB), 0x80E0=BGR (GL_BGR)
        @param upsidedown false=lines stored from top to bottom, true=lines stored from bottom to top
        @return 0 if OK, -1 if failed to open file, -2 if unexpected pixelSize
*/
int SE_WriteTGA(const char* filename, int width, int height, const unsigned char* data, int pixelSize, int pixelFormat, bool upsidedown);

/**
        Read a CSV file (comma separated values)
        @param filename File name including extension
        @param content Reference to variable where to put the entries (vector of vector of entries - per line)
        @param skip_lines Number of initial lines to skip (e.g. header). Optional (default = 0)
        @return 0 if OK, -1 if failed to open file
*/
int SE_ReadCSVFile(const char* filename, std::vector<std::vector<std::string>>& content, int skip_lines = 0);