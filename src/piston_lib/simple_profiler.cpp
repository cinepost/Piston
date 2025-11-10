#include "simple_profiler.h"

#include <numeric>



namespace Piston {

namespace {

template<typename T>
void updateMaximum(std::atomic<T>& maximum_value, T const& value) noexcept {
	T prev_value = maximum_value;
	while(prev_value < value && !maximum_value.compare_exchange_weak(prev_value, value)) {}
}

}


namespace sa {

double mean(std::vector<uint64_t> const& v){
	if(v.empty()){
        return 0;
    }

    return static_cast<double>(std::reduce(v.begin(), v.end())) / static_cast<double>(v.size());
}

size_t summ(std::vector<uint64_t> const& v) {
	return std::accumulate(v.begin(), v.end(), 0);
}

double stdv(std::vector<uint64_t> const& v, const double& mean) {
	double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
	return std::sqrt((sq_sum / static_cast<double>(v.size())) - (mean * mean));
}

} // namespace sa


SimpleProfiler::SimpleProfiler( const char* name ) {
	dbg_printf("%s\n", name);

	mName = std::string(name);
	updateMaximum(mCallerNameWidth, mName.size());
	mTimeStart = Clock::now();
}

SimpleProfiler::~SimpleProfiler() {
	TimePoint t = Clock::now();

	std::map<std::string, acc_t >::iterator p = mMap.find( mName );
	if( p == mMap.end() ) {
		// this is the first time this scope has run
		acc_t acc;
		std::pair<std::string,acc_t> pr(mName, acc);
		p = mMap.insert(pr).first;
	}
	// add the time of running to the accumulator for this scope
	p->second.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(t - mTimeStart).count());
}

// Generate profile report
void SimpleProfiler::printReport() {
	if(mMap.empty()) return;
	
	printf("SimpleProfiler report...\n");
	printf("%42s Calls\tMean (secs)\tStdDev\tTotal (secs)\n","Scope");
	for(std::map<std::string, acc_t>::iterator p = mMap.begin(); p != mMap.end(); p++ ) {
		double _avg = sa::mean(p->second);
		double _std = sa::stdv(p->second, _avg);
		size_t _sum = sa::summ(p->second);

		printf("%42s %lu\t%f\t%f\t%f\n", p->first.c_str(), p->second.size(), static_cast<float>(_avg) * 0.001f, static_cast<float>(_std) * 0.001f, float(_sum) * 0.001f);
	}
	printf("\n");
}

std::map<std::string, SimpleProfiler::acc_t> SimpleProfiler::mMap;

std::atomic<size_t> SimpleProfiler::mCallerNameWidth = 16; 

}  // namespace Falcor