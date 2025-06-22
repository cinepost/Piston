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

} // namespace Piston
