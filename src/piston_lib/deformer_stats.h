#ifndef PISTON_LIB_DEFORMER_STATS_H_
#define PISTON_LIB_DEFORMER_STATS_H_

#include <memory>
#include <string>
#include <map>

#include <hboost/variant.hpp>
#include <hboost/lexical_cast.hpp>

#include "framework.h"


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

} // namespace Piston

#endif // PISTON_LIB_DEFORMER_STATS_H_