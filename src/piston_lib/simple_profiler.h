#ifndef PISTON_LIB_SIMPLE_PROFILER_H_
#define PISTON_LIB_SIMPLE_PROFILER_H_

#include <string>
#include <vector>
#include <utility>
#include <numeric>
#include <map>
#include <chrono>
#include <atomic>

#include "framework.h"


#ifdef _DEBUG
#define PROFILE(NAME) Piston::SimpleProfiler(NAME)
#define PROFILE_PRINT() Piston::SimpleProfiler::printReport()
#else
#define PROFILE(NAME) {}
#define PROFILE_PRINT() {}
#endif

namespace Piston {

class SimpleProfiler {
	public:
		typedef std::vector<uint64_t> acc_t;
		
		using Clock = std::chrono::high_resolution_clock;
		using TimePoint = std::chrono::time_point<Clock>;
		using TimeDuration = std::chrono::duration<double, std::milli>;

		SimpleProfiler(const char* name);
		~SimpleProfiler();

		// produces report when called without parameters
		static void printReport();


	private:
		static std::map <std::string, acc_t> mMap;
		std::string mName;
		TimePoint mTimeStart;

		static std::atomic<size_t> mCallerNameWidth;
};

}  // namespace Piston

#endif  // PISTON_LIB_SIMPLE_PROFILER_H_
