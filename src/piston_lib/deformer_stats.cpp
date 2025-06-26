#include "deformer_stats.h"


namespace Piston {

DeformerStats::DeformerStats() {

}

void DeformerStats::clear() {
	mMap.clear();
}

std::string DeformerStats::toString() const {
	std::string str;

	if(!mMap.empty()) str += "Deformer stats:\n";

	for(const auto&[key, value]: mMap) {
		str += "\t" + std::string(key) + ": " + hboost::lexical_cast<std::string>(value) + "\n";
	}

	return str;
}

void DeformerStats::addEntry(const Key& key, const Value& value) {
	mMap[key] = value;
}

Timer::Timer(): mDuration(0.f) {
}

void Timer::start() {
	mStart = std::chrono::high_resolution_clock::now();
}

void Timer::stop() {
	mDuration += std::chrono::high_resolution_clock::now() - mStart;
}

std::string Timer::toString() const {
    float ms = mDuration.count() * 1000.0f;
    return std::to_string(ms) + " ms. (" + std::to_string(mDuration.count()) + " sec.)";
}

} // namespace Piston
