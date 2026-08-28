#ifndef PISTON_LIB_PXR_POINTS_LRU_CACHE_H_
#define PISTON_LIB_PXR_POINTS_LRU_CACHE_H_

#include "framework.h"
#include "common.h"

#include <pxr/usd/usd/timeCode.h>
#include <pxr/base/gf/matrix3f.h>

#include <atomic>
#include <mutex>
#include <unordered_map>
#include <list>
#include <string>


namespace Piston {

class CPxrPointsLRUCacheShrinkLock;

class PxrPointsLRUCacheBase {
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

	protected:
		virtual void reduceMemUsage(const size_t mem_size_bytes) = 0;

		void shrink_lock() { mShrinkLock = true; }
		void shrink_unlock() { mShrinkLock = false; reduceMemUsage(mMaxMemSizeBytes);}

		std::atomic<bool> mShrinkLock;
		mutable std::mutex mMutex;

		size_t mMaxMemSizeBytes;
		static size_t sMaxMemSizeBytes;

		friend class PxrPointsLRUCacheShrinkLock;
};


template<typename T>
class PxrPointsLRUCache: public PxrPointsLRUCacheBase {
	public:
		using CompositeKey = PxrPointsLRUCacheBase::CompositeKey;
		typedef typename std::pair<CompositeKey, T> key_value_pair_t;
		typedef typename std::list<key_value_pair_t>::iterator list_iterator_t;

		using UniquePtr = std::unique_ptr<PxrPointsLRUCache<T>>;
		static UniquePtr create(const size_t max_mem_size_bytes);

	public:

		T* put(const CompositeKey& key, size_t points_count, bool init_to_zero = false);
		T* put(const CompositeKey& key, T&& points);
		const T* get(const CompositeKey& key) const;

		bool exists(const CompositeKey& key) const {
			return mCacheItemsMap.find(key) != mCacheItemsMap.end();
		}

		size_t removeByName(const std::string& name);

		// Used memory ignoring keys and iterators mem usage. PointsList mem usage only
		size_t getMemSize() const;

		void setMaxMemSize(size_t max_mem_size_bytes);

		size_t size() const { return mCacheItemsMap.size(); }

		inline float getMemUsagePercent() const { return 100.0f * (static_cast<float>(getMemSize()) / static_cast<float>(mMaxMemSizeBytes)); }
		std::string getCacheUtilizationString() const;
		std::string getMemUsageString() const;

		size_t itemsCount() const { return mCacheItemsList.size(); }

		void clear();

	private:
		static constexpr size_t kInvalidUsedMemSize = std::numeric_limits<size_t>::max();

		PxrPointsLRUCache(const size_t max_mem_size_bytes);
	
		virtual void reduceMemUsage(const size_t mem_size_bytes) override final;

	private:
		mutable std::list<key_value_pair_t> mCacheItemsList;
		std::unordered_map<CompositeKey, list_iterator_t, CompositeKey::Hasher> mCacheItemsMap;
		
		mutable size_t mCurrentMemSizeBytes;

		size_t mMinEntries;
};

class PxrPointsLRUCacheShrinkLock{
	public:
		PxrPointsLRUCacheShrinkLock(PxrPointsLRUCacheBase* m) : mValid(false), mMtx(*m) {
			if(m) {
				mValid = true;
				mMtx.shrink_lock();
			}
		}

		PxrPointsLRUCacheShrinkLock(PxrPointsLRUCacheBase & m) : mValid(true), mMtx(m){
		    mMtx.shrink_lock();
		}
		~PxrPointsLRUCacheShrinkLock(){
		    if(mValid){
				mMtx.shrink_unlock();
			}
		}

		bool isValid() const { return mValid; }

	private:
		bool mValid;
		PxrPointsLRUCacheBase& mMtx;
};

inline std::string to_string(const Piston::PxrPointsLRUCacheBase::CompositeKey& key) {
	return key.name + ":" + std::to_string(key.time.GetValue());
}

} // namespace Piston

#endif // PISTON_LIB_PXR_POINTS_LRU_CACHE_H_