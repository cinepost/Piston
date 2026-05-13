#include "base_curves_deformer.h"
#include "deformer_data_cache.h"
#include "adjacency.h"
#include "phantom_trimesh.h"
#include "deformer_factory.h"

#include "fast_curves_deformer.h"
#include "fast_curves_deformer_data.h"
#include "wrap_curves_deformer_data.h"
#include "guide_curves_deformer_data.h"


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

DeformerDataCache::KeyBase::KeyBase(const std::type_index& _type_idx, const UsdPrimHandle& handle, pxr::UsdTimeCode time_code): type_idx(_type_idx) {
	const auto path(handle.getPath());
	assert(path.IsAbsolutePath());
	paths.emplace_back(std::move(path));
	const auto& topology = handle.getTopology(time_code);
	topologies_hash_sum = topology.topology_hash;

	static auto& cache = DeformerDataCache::getInstance();
	topology_indices.push_back(cache.getTopologyIndexFromPool(topology));
}

DeformerDataCache::KeyBase::KeyBase(const std::type_index& _type_idx, const std::vector<const UsdPrimHandle*>& handles, pxr::UsdTimeCode time_code):type_idx(_type_idx) {
	assert(!handles.empty());
	
	const size_t handles_count = handles.size();
	paths.resize(handles_count);
	topology_indices.resize(handles_count);

	static auto& cache = DeformerDataCache::getInstance();

	for(size_t i = 0; i < handles.size(); ++i) {
		const UsdPrimHandle* pHandle = handles[i];
		assert(pHandle);
		const auto path(pHandle->getPath());
		assert(path.IsAbsolutePath());
		paths[i] = std::move(path); 
	}

	// place topology hashes sorted by path
	{
		std::vector<const UsdPrimHandle*> vec(handles);
		std::sort(vec.begin(), vec.end(), [](const UsdPrimHandle* a, const UsdPrimHandle* b) {
       		return a->getPath() < b->getPath();
    	});

		for(const auto* pHandle: vec) {
			const auto& topology = pHandle->getTopology(time_code);
			topologies_hash_sum += topology.topology_hash;
			topology_indices.push_back(cache.getTopologyIndexFromPool(topology));
		}
	}
}

template< class T>
std::shared_ptr<T> DeformerDataCache::getOrCreateData(const BaseCurvesDeformer* pDeformer, const UsdPrimHandle& handle, pxr::UsdTimeCode time_code, bool& created) {
	static_assert(std::is_base_of<SerializableDeformerDataBase, T>::value, "Class needs to be SerializableDeformerDataBase");

	const std::vector<const UsdPrimHandle*> handle_ptrs({&handle});
	return getOrCreateData<T>(pDeformer, handle_ptrs, time_code, created);
}

template< class T>
std::shared_ptr<T> DeformerDataCache::getOrCreateData(const BaseCurvesDeformer* pDeformer, const std::vector<const UsdPrimHandle*>& handles, pxr::UsdTimeCode time_code, bool& created) {
	static_assert(std::is_base_of<SerializableDeformerDataBase, T>::value, "Class needs to be SerializableDeformerDataBase");

	assert(pDeformer);

	for(const auto* pHandle: handles) {
		assert(pHandle);
		if(!pHandle->getPath().IsAbsolutePath()) {
			LOG_ERR << "Invalid handle path: " << pHandle->getPath() << "! DeformerDataCache relative keys are not supported !";
			return nullptr;
		}
	}

	created = false;
	const std::lock_guard<std::mutex> lock(mMutex);
	MapType::iterator it = mDataMap.end();
	KeyVariant key;

	if(mUseDataInstancing && pDeformer->getInstancingState()) {
		key = Key(std::type_index(typeid(T)), handles, time_code);
		LOG_TRC << "Getting " << typeid(T).name() << " using loose (topology similar) key with topologies_hash_sum " << std::get<Key>(key).topologies_hash_sum;
		it = mDataMap.find(key);
	} else {
		key = KeyStrict(std::type_index(typeid(T)), handles, time_code);
		LOG_TRC << "Getting " << typeid(T).name() << " using strict (prim exact paths) key with paths " << std::get<KeyStrict>(key).paths;
		it = mDataMap.find(key);
	}

	if (it != mDataMap.end()) {
		LOG_TRC << "Data " << typeid(T).name() << " found in cache";
		return std::dynamic_pointer_cast<T>(it->second);
	}

	LOG_TRC << "Data " << typeid(T).name() << " placed in cache";
	auto result = mDataMap.emplace(key, std::make_shared<T>());
	created = true;
	return std::dynamic_pointer_cast<T>(result.first->second);
}

void DeformerDataCache::clear() {
	const std::lock_guard<std::mutex> lock(mMutex);
	const std::lock_guard<std::mutex> topo_lock(mTopologyPoolMutex);
	mDataMap.clear();
	if(mDataMap.empty()) {

		mTopologyPool.clear();
	}
}

template< class T>
void DeformerDataCache::invalidate(const UsdPrimHandle& handle, pxr::UsdTimeCode time_code) {
	LOG_DBG << " DeformerDataCache::invalidate(...) " << handle;

	const std::vector<const UsdPrimHandle*> handle_ptrs({&handle});
	invalidate<T>(handle_ptrs, time_code);
}

template< class T>
void DeformerDataCache::invalidate(const std::vector<const UsdPrimHandle*>& handles, pxr::UsdTimeCode time_code) {
	const DeformerDataCache::KeyStrict key(std::type_index(typeid(T)), handles, time_code);

	const std::lock_guard<std::mutex> lock(mMutex);

	auto it = mDataMap.find(key);
	if (it != mDataMap.end()) {
		assert(it->second);
		it->second->clear();
	}
}

template< class T>
void DeformerDataCache::invalidate(const std::shared_ptr<T>& pData) {
	if(!pData) return;

	const std::lock_guard<std::mutex> lock(mMutex);
	for (auto it = mDataMap.begin(); it != mDataMap.end(); ++it) {
		if (it->second == pData) {
			it->second->clear();
		}
	}
}

void DeformerDataCache::cleanup() {
	const std::lock_guard<std::mutex> lock(mMutex);
	for (auto it = mDataMap.begin(); it != mDataMap.end(); ) {
		if (it->second.use_count() == 1) {
			mDataMap.erase(it);
		} else {
			++it;
		} 
	}
}

DeformerDataCache::~DeformerDataCache() { }

DeformerDataCache::DeformerDataCache() { 
	mUseDataInstancing = CurvesDeformerFactory::getInstance().getDataInstancingState();
}

// Specialization Macro
#define SPECIALIZE_TYPE_NAME(type) \
template std::shared_ptr<type> DeformerDataCache::getOrCreateData(const BaseCurvesDeformer* pDeformer, const UsdPrimHandle& handle, pxr::UsdTimeCode time_code, bool& created); \
template std::shared_ptr<type> DeformerDataCache::getOrCreateData(const BaseCurvesDeformer* pDeformer, const std::vector<const UsdPrimHandle*>& handles, pxr::UsdTimeCode time_code, bool& created); \
template void DeformerDataCache::invalidate<type>(const UsdPrimHandle& handle, pxr::UsdTimeCode time_code); \
template void DeformerDataCache::invalidate<type>(const std::vector<const UsdPrimHandle*>& handles, pxr::UsdTimeCode time_code); \
template void DeformerDataCache::invalidate(const std::shared_ptr<type>& pData);

SPECIALIZE_TYPE_NAME(SerializablePhantomTrimesh)
SPECIALIZE_TYPE_NAME(SerializableUsdGeomMeshFaceAdjacency)
SPECIALIZE_TYPE_NAME(FastCurvesDeformerData)
SPECIALIZE_TYPE_NAME(WrapCurvesDeformerData)
SPECIALIZE_TYPE_NAME(GuideCurvesDeformerData)

#undef SPECIALIZE_TYPE_NAME

} // namespace Piston

// Initialize static members
Piston::DeformerDataCache* Piston::DeformerDataCache::mInstancePtr = nullptr;
std::mutex Piston::DeformerDataCache::mMutex;
std::mutex Piston::DeformerDataCache::mTopologyPoolMutex;