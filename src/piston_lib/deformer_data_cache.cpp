#include "deformer_data_cache.h"
#include "adjacency.h"
#include "phantom_trimesh.h"

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

DeformerDataCache::Key::Key(const std::type_index& _type_idx, const UsdPrimHandle& handle):type_idx(_type_idx), path(handle.getPath()) {
	assert(path.IsAbsolutePath());
	const auto& topology = handle.getTopology();
	topology_variant = topology.topology_variant;
	topology_hash = topology.topology_hash;
}	

//template< class T>
//std::shared_ptr<T> DeformerDataCache::getOrCreateData(const pxr::SdfPath& path) {
//	static_assert(std::is_base_of<SerializableDeformerDataBase, T>::value, "Class needs to be SerializableDeformerDataBase");

//	if(!path.IsAbsolutePath()) {
//		std::cerr << "DeformerDataCache relative keys are not supported !" << std::endl;
//		return nullptr;
//	}

//	const DeformerDataCache::Key key(std::type_index(typeid(T)), path);

//	const std::lock_guard<std::mutex> lock(mMutex);
//	auto it = mDataMap.find(key);
//	if (it != mDataMap.end()) {
//		return std::dynamic_pointer_cast<T>(it->second);
//	}

//	auto result = mDataMap.emplace(key, std::make_shared<T>());
//	return std::dynamic_pointer_cast<T>(result.first->second);
//}

//template< class T>
//std::shared_ptr<T> DeformerDataCache::getOrCreateData(const std::string& name) {
//	return getOrCreateData<T>(pxr::SdfPath(name));
//}

template< class T>
std::shared_ptr<T> DeformerDataCache::getOrCreateData(const UsdPrimHandle& handle) {
	static_assert(std::is_base_of<SerializableDeformerDataBase, T>::value, "Class needs to be SerializableDeformerDataBase");

	if(!handle.getPath().IsAbsolutePath()) {
		std::cerr << "DeformerDataCache relative keys are not supported !" << std::endl;
		return nullptr;
	}

	const DeformerDataCache::Key key(std::type_index(typeid(T)), handle);

	const std::lock_guard<std::mutex> lock(mMutex);
	auto it = mDataMap.find(key);
	if (it != mDataMap.end()) {
		LOG_TRC << "Data " << it->first.path << " used from cache for " << handle << " with hash " << it->first.topology_hash;
		return std::dynamic_pointer_cast<T>(it->second);
	}

	LOG_TRC << "Data " << key.path << " placed in cache for " << handle << " with hash " << key.topology_hash;
	auto result = mDataMap.emplace(key, std::make_shared<T>());
	return std::dynamic_pointer_cast<T>(result.first->second);
}

void DeformerDataCache::clear() {
	mDataMap.clear();
}

DeformerDataCache::~DeformerDataCache() { }

DeformerDataCache::DeformerDataCache() { }

//template std::shared_ptr<SerializablePhantomTrimesh> DeformerDataCache::getOrCreateData(const pxr::SdfPath& path);
//template std::shared_ptr<SerializablePhantomTrimesh> DeformerDataCache::getOrCreateData(const std::string& name);
template std::shared_ptr<SerializablePhantomTrimesh> DeformerDataCache::getOrCreateData(const UsdPrimHandle& handle);

//template std::shared_ptr<SerializableUsdGeomMeshFaceAdjacency> DeformerDataCache::getOrCreateData(const pxr::SdfPath& path);
//template std::shared_ptr<SerializableUsdGeomMeshFaceAdjacency> DeformerDataCache::getOrCreateData(const std::string& name);
template std::shared_ptr<SerializableUsdGeomMeshFaceAdjacency> DeformerDataCache::getOrCreateData(const UsdPrimHandle& handle);

} // namespace Piston

// Initialize static members
Piston::DeformerDataCache* Piston::DeformerDataCache::mInstancePtr = nullptr;
std::mutex Piston::DeformerDataCache::mMutex;