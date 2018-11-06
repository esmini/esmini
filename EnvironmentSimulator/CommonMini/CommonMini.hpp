#include <fstream>
#include <string>
#include <sdkddkver.h>

#ifdef _WIN32
   #define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#else
   #define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif

#define LOG(Format_, ...)  Logger::Inst().Log(__FILENAME__, __FUNCTION__, __LINE__, Format_, __VA_ARGS__)


// Time functions
__int64 SE_getSystemTime();
void SE_sleep(unsigned int msec);

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
