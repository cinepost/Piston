#include "deformer_data_cache.h"

namespace Piston {

static constexpr size_t kDefaultPxrPointsLRUCacheMaxSize = 1024 * 1024 * 256 * 4; 

DeformerDataCache& DeformerDataCache::getInstance() {
    if (mInstancePtr == nullptr) {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mInstancePtr == nullptr) {
            mInstancePtr = new DeformerDataCache();
        }
    }
    return *mInstancePtr;
}

template< class T>
std::shared_ptr<T> DeformerDataCache::getOrCreateData(const Key& key) {
	static_assert(std::is_base_of<SerializableDeformerDataBase, T>::value, "Class needs to be SerializableDeformerDataBase");

	if(!key.IsAbsolutePath()) {
		std::cerr << "DeformerDataCache relative keys are not supported !" << std::endl;
		return nullptr;
	}

	const std::lock_guard<std::mutex> lock(mMutex);
	auto it = mDataMap.find(key);
	
	if (it == mDataMap.end()) {
		mDataMap[key] = std::make_shared<T>();
	}

	return *it;
}

template< class T>
std::shared_ptr<T> DeformerDataCache::getOrCreateData(const std::string& name) {
	return getOrCreateData<T>(pxr::SdfPath(name));
}

void DeformerDataCache::clear() {
	mDataMap.clear();
}

DeformerDataCache::~DeformerDataCache() {

}

DeformerDataCache::DeformerDataCache() {

}


} // namespace Piston

// Initialize static members
Piston::DeformerDataCache* Piston::DeformerDataCache::mInstancePtr = nullptr;
std::mutex Piston::DeformerDataCache::mMutex;