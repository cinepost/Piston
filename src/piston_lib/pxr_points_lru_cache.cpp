#include "pxr_points_lru_cache.h"

static inline size_t pointsArrayMemUsage(const std::vector<pxr::GfVec3f>& points) {
	return points.size() * sizeof(pxr::GfVec3f);
}

namespace Piston {

PxrPointsLRUCache::PxrPointsLRUCache(const size_t max_mem_size_bytes): mMaxMemSizeBytes(max_mem_size_bytes) {

}

PxrPointsLRUCache::UniquePtr PxrPointsLRUCache::create(const size_t max_mem_size_bytes) {
	PxrPointsLRUCache::UniquePtr pResult = PxrPointsLRUCache::UniquePtr(new PxrPointsLRUCache(max_mem_size_bytes));

	return pResult;
}

size_t PxrPointsLRUCache::getMemSize() const {
	size_t result = 0;

	for(const auto& pair: mCacheItemsList) {
		result += pointsArrayMemUsage(pair.second);
	}
}

const PxrPointsLRUCache::PointsList* PxrPointsLRUCache::put(const PxrPointsLRUCache::CompositeKey& key, const PxrPointsLRUCache::PointsList& points) {
	const size_t new_points_mem_reqs = pointsArrayMemUsage(points);
	reduceMemUsage(mMaxMemSizeBytes - new_points_mem_reqs);

	auto it = mCacheItemsMap.find(key);
	mCacheItemsList.push_front(key_value_pair_t(key, points));
	if (it != mCacheItemsMap.end()) {
		mCacheItemsList.erase(it->second);
		mCacheItemsMap.erase(it);
	}

	mCacheItemsMap[key] = mCacheItemsList.begin();
	return &mCacheItemsList.begin()->second;
}

const PxrPointsLRUCache::PointsList* PxrPointsLRUCache::get(const PxrPointsLRUCache::CompositeKey& key) const {
	auto it = mCacheItemsMap.find(key);
	if (it == mCacheItemsMap.end()) {
		std::cerr << "There is no such key (" << key.name << ") in PxrPointsLRUCache !!!" << std::endl;
		return nullptr;
	} else {
		mCacheItemsList.splice(mCacheItemsList.begin(), mCacheItemsList, it->second);
		dbg_printf("URA!");
		return &it->second->second;
	}
}

void PxrPointsLRUCache::setMaxMemSize(size_t max_mem_size_bytes) {
	if(mMaxMemSizeBytes == max_mem_size_bytes) return;
	
	reduceMemUsage(max_mem_size_bytes);
	mMaxMemSizeBytes = max_mem_size_bytes;
}

void PxrPointsLRUCache::reduceMemUsage(const size_t mem_size_bytes) {
	while (getMemSize() > mem_size_bytes) {
		auto last = mCacheItemsList.end();
		last--;
		mCacheItemsMap.erase(last->first);
		mCacheItemsList.pop_back();
	}
}
	
} // namespace Piston
