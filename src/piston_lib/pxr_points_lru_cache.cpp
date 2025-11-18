#include "pxr_points_lru_cache.h"

static inline size_t pointsArrayMemUsageRequirements(const std::vector<pxr::GfVec3f>& points) {
	return points.size() * sizeof(pxr::GfVec3f);
}

namespace Piston {

PxrPointsLRUCache::PxrPointsLRUCache(const size_t max_mem_size_bytes): mMaxMemSizeBytes(max_mem_size_bytes), mCurrentMemSizeBytes(0), mShrinkLock(false) {

}

PxrPointsLRUCache::UniquePtr PxrPointsLRUCache::create(const size_t max_mem_size_bytes) {
	PxrPointsLRUCache::UniquePtr pResult = PxrPointsLRUCache::UniquePtr(new PxrPointsLRUCache(max_mem_size_bytes));

	return pResult;
}

size_t PxrPointsLRUCache::getMemSize() const {
	if(mCurrentMemSizeBytes != kInvalidUsedMemSize) return mCurrentMemSizeBytes;

	mCurrentMemSizeBytes = 0;

	for(const auto& pair: mCacheItemsList) {
		mCurrentMemSizeBytes += pointsArrayMemUsageRequirements(pair.second);
	}

	return mCurrentMemSizeBytes;
}

const PxrPointsLRUCache::PointsList* PxrPointsLRUCache::put(const PxrPointsLRUCache::CompositeKey& key, const PxrPointsLRUCache::PointsList& points) {
	const size_t new_points_mem_reqs = pointsArrayMemUsageRequirements(points);
	reduceMemUsage(mMaxMemSizeBytes - new_points_mem_reqs);

	auto it = mCacheItemsMap.find(key);
	mCacheItemsList.push_front(key_value_pair_t(key, points));
	if (it != mCacheItemsMap.end()) {
		mCacheItemsList.erase(it->second);
		mCacheItemsMap.erase(it);
	}

	mCacheItemsMap[key] = mCacheItemsList.begin();

	mCurrentMemSizeBytes = kInvalidUsedMemSize;

	return &mCacheItemsList.begin()->second;
}

const PxrPointsLRUCache::PointsList* PxrPointsLRUCache::get(const PxrPointsLRUCache::CompositeKey& key) const {
	auto it = mCacheItemsMap.find(key);
	if (it == mCacheItemsMap.end()) {
		dbg_printf("There is no such key (%s) in PxrPointsLRUCache.\n", key.name.c_str());
		return nullptr;
	} else {
		mCacheItemsList.splice(mCacheItemsList.begin(), mCacheItemsList, it->second);
		return &it->second->second;
	}
}

void PxrPointsLRUCache::setMaxMemSize(size_t max_mem_size_bytes) {
	if(mMaxMemSizeBytes == max_mem_size_bytes) return;
	
	if(max_mem_size_bytes < mMaxMemSizeBytes) {
		reduceMemUsage(max_mem_size_bytes);
	}

	mMaxMemSizeBytes = max_mem_size_bytes;
}

void PxrPointsLRUCache::reduceMemUsage(const size_t mem_size_bytes) {
	if(mShrinkLock) return;

	while (getMemSize() > mem_size_bytes) {
		auto last = mCacheItemsList.end();
		last--;
		mCacheItemsMap.erase(last->first);
		mCacheItemsList.pop_back();
		mCurrentMemSizeBytes = kInvalidUsedMemSize;
	}
}

std::string PxrPointsLRUCache::getCacheUtilizationString() const {
	static char _buffer[50];

	const float utilization = 100.0f * (static_cast<double>(getMemSize()) / static_cast<float>(mMaxMemSizeBytes)); 

    sprintf(_buffer, "%.2f", utilization);
    return std::string(_buffer);
}
	
} // namespace Piston
