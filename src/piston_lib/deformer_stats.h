#ifndef PISTON_LIB_DEFORMER_STATS_H_
#define PISTON_LIB_DEFORMER_STATS_H_

#include "framework.h"

#include <hboost/variant.hpp>
#include <hboost/lexical_cast.hpp>

#include <memory>
#include <string>
#include <map>
#include <chrono>


namespace Piston {

class DeformerStats {
	public:
		using Key = std::string;
		using Value = hboost::variant<size_t, uint32_t, int, float, char, std::string>;
		
		DeformerStats();

		std::string toString() const;

		void addEntry(const Key& key, const Value& value);
		void clear();

	private:
		std::map<Key, Value> mMap;
};

struct Timer {
    Timer();

    void start();
    void stop();

    const std::chrono::duration<float>& getDuration() const { return mDuration; }

    std::string toString() const;

    std::chrono::time_point<std::chrono::high_resolution_clock> mStart;
    std::chrono::duration<float> mDuration;
};

} // namespace Piston

#endif // PISTON_LIB_DEFORMER_STATS_H_