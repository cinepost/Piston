#ifndef PISTON_LIB_PXR_POINTS_LRU_CACHE_H_
#define PISTON_LIB_PXR_POINTS_LRU_CACHE_H_

#include "framework.h"
#include "common.h"

#include <pxr/usd/usd/timeCode.h>
#include <pxr/base/gf/matrix3f.h>

#include <mutex>
#include <unordered_map>
#include <list>
#include <string>


namespace Piston {

class PxrPointsLRUCache {
	public:
		struct CompositeKey {
			std::string name;
			pxr::UsdTimeCode time;

			bool operator==(const CompositeKey &other) const { 
				return (name == other.name && time == other.time);
			}

			struct Hasher {
				size_t operator()(const CompositeKey& key) const {
					size_t name_hash = std::hash<std::string>()(key.name);
					size_t time_hash = hash_value(key.time) << 1;
					return name_hash ^ time_hash;
				}
			};
		};

		using PointsList = std::vector<pxr::GfVec3f>;

		typedef typename std::pair<CompositeKey, PointsList> key_value_pair_t;
		typedef typename std::list<key_value_pair_t>::iterator list_iterator_t;

		using UniquePtr = std::unique_ptr<PxrPointsLRUCache>;
		static UniquePtr create(const size_t max_mem_size_bytes);

	public:

		const PointsList* put(const CompositeKey& key, const PointsList& points);
		const PointsList* get(const CompositeKey& key) const;

		bool exists(const CompositeKey& key) const {
			return mCacheItemsMap.find(key) != mCacheItemsMap.end();
		}

		// Used memory ignoring keys and iterators mem usage. PointsList mem usage only
		size_t getMemSize() const;

		void setMaxMemSize(size_t max_mem_size_bytes);

		size_t size() const { return mCacheItemsMap.size(); }

	private:
		PxrPointsLRUCache(const size_t max_mem_size_bytes);
	
		void reduceMemUsage(const size_t mem_size_bytes);

	private:
		mutable std::list<key_value_pair_t> mCacheItemsList;
		std::unordered_map<CompositeKey, list_iterator_t, CompositeKey::Hasher> mCacheItemsMap;
		size_t mMaxMemSizeBytes;
		
};

} // namespace Piston

using Key = Piston::PxrPointsLRUCache::CompositeKey;

inline std::string to_string(const Key& key) {
	return key.name + ":" + std::to_string(key.time.GetValue());
}

#endif // PISTON_LIB_PXR_POINTS_LRU_CACHE_H_