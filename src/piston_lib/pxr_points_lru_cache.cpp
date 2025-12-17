#include "pxr_points_lru_cache.h"
#include "common.h"


namespace Piston {

static const size_t kMinEntries = 4; // 1 frame ofo current deformed curve points, 2 more frames for worst case motoin blur and 1 frame for velocities.

PxrPointsLRUCache::PxrPointsLRUCache(const size_t max_mem_size_bytes): mMaxMemSizeBytes(max_mem_size_bytes), mCurrentMemSizeBytes(0), mShrinkLock(false), mMinEntries(kMinEntries) {

}

PxrPointsLRUCache::UniquePtr PxrPointsLRUCache::create(const size_t max_mem_size_bytes) {
	PxrPointsLRUCache::UniquePtr pResult = PxrPointsLRUCache::UniquePtr(new PxrPointsLRUCache(max_mem_size_bytes));

	return pResult;
}

size_t PxrPointsLRUCache::getMemSize() const {
	if(mCurrentMemSizeBytes != kInvalidUsedMemSize) return mCurrentMemSizeBytes;

	mCurrentMemSizeBytes = 0;

	for(const auto& pair: mCacheItemsList) {
		mCurrentMemSizeBytes += pair.second.sizeInBytes();
	}

	return mCurrentMemSizeBytes;
}

PointsList* PxrPointsLRUCache::put(const CompositeKey& key, size_t points_count, bool init_to_zero) {
	assert(points_count > 0);

	PointsList points(points_count);

	if(init_to_zero) points.fillWithZero();

	return put(key, std::move(points));
}

PointsList* PxrPointsLRUCache::put(const PxrPointsLRUCache::CompositeKey& key, PointsList&& points) {
	std::lock_guard<std::mutex> lock(mMutex);
	const size_t new_points_mem_reqs = points.sizeInBytes();
	reduceMemUsage(mMaxMemSizeBytes - new_points_mem_reqs);

	auto it = mCacheItemsMap.find(key);
	mCacheItemsList.push_front(key_value_pair_t(key, std::move(points)));
	if (it != mCacheItemsMap.end()) {
		mCacheItemsList.erase(it->second);
		mCacheItemsMap.erase(it);
	}

	mCacheItemsMap[key] = mCacheItemsList.begin();

	mCurrentMemSizeBytes = kInvalidUsedMemSize;

	return &mCacheItemsList.begin()->second;
}

const PointsList* PxrPointsLRUCache::get(const PxrPointsLRUCache::CompositeKey& key) const {
	std::lock_guard<std::mutex> lock(mMutex);
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

	while ((getMemSize() > mem_size_bytes) && (mCacheItemsList.size() > mMinEntries)) {
		auto last = mCacheItemsList.end();
		last--;
		mCacheItemsMap.erase(last->first);
		mCacheItemsList.pop_back();
		mCurrentMemSizeBytes = kInvalidUsedMemSize;
	}
}

size_t PxrPointsLRUCache::removeByName(const std::string& name) {
	size_t removed_count = 0;
	std::list<key_value_pair_t>::iterator i = mCacheItemsList.begin();

	while(i != mCacheItemsList.end()) {
		if(i->first.name == name) {
			mCacheItemsMap.erase(i->first);
			mCacheItemsList.pop_back();
			mCurrentMemSizeBytes = kInvalidUsedMemSize;
			removed_count++;
		} else {
			i++;
		}
	}

	return removed_count;
}

void PxrPointsLRUCache::clear() {
	std::lock_guard<std::mutex> lock(mMutex);
	mCacheItemsMap.clear();
	mCacheItemsList.clear();
	mCurrentMemSizeBytes = kInvalidUsedMemSize;
}

std::string PxrPointsLRUCache::getCacheUtilizationString() const {
	static char _buffer[50];

	const float utilization = 100.0f * (static_cast<double>(getMemSize()) / static_cast<float>(mMaxMemSizeBytes)); 

    sprintf(_buffer, "%.2f", utilization);
    return std::string(_buffer);
}

std::string PxrPointsLRUCache::getMemUsageString() const {
	return std::string(stringifyMemSize(getMemSize()));
}
	
} // namespace Piston
