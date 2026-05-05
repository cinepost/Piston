#include "base_curves_deformer.h"
#include "deformer_data_cache.h"
#include "adjacency.h"
#include "phantom_trimesh.h"
#include "deformer_factory.h"

#include "fast_curves_deformer.h"
#include "fast_curves_deformer_data.h"


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

DeformerDataCache::Key::Key(const std::type_index& _type_idx, const UsdPrimHandle& handle): type_idx(_type_idx) {
	const auto path(handle.getPath());
	assert(path.IsAbsolutePath());
	ordered_paths.emplace_back(std::move(path));
	const auto& topology = handle.getTopology();
	topologies_hash_sum = topology.topology_hash;

	static auto& cache = DeformerDataCache::getInstance();
	topology_indices.push_back(cache.getTopologyIndexFromPool(topology));
}

DeformerDataCache::Key::Key(const std::type_index& _type_idx, const std::vector<const UsdPrimHandle*>& handles):type_idx(_type_idx) {
	assert(!handles.empty());
	
	const size_t handles_count = handles.size();
	ordered_paths.resize(handles_count);
	topology_indices.resize(handles_count);

	static auto& cache = DeformerDataCache::getInstance();

	for(size_t i = 0; i < handles.size(); ++i) {
		const UsdPrimHandle* pHandle = handles[i];
		assert(pHandle);
		const auto path(pHandle->getPath());
		assert(path.IsAbsolutePath());
		ordered_paths[i] = std::move(path);
		const auto& topology = pHandle->getTopology();
		topologies_hash_sum += topology.topology_hash;
		topology_indices.push_back(cache.getTopologyIndexFromPool(topology));
	}

	std::sort(ordered_paths.begin(), ordered_paths.end());
}

template< class T>
std::shared_ptr<T> DeformerDataCache::getOrCreateData(const BaseCurvesDeformer* pDeformer, const UsdPrimHandle& handle, bool& created) {
	static_assert(std::is_base_of<SerializableDeformerDataBase, T>::value, "Class needs to be SerializableDeformerDataBase");

	std::vector<const UsdPrimHandle*> handle_ptrs({&handle});
	return getOrCreateData<T>(pDeformer, handle_ptrs, created);
}

template< class T>
std::shared_ptr<T> DeformerDataCache::getOrCreateData(const BaseCurvesDeformer* pDeformer, const std::vector<const UsdPrimHandle*>& handles, bool& created) {
	static_assert(std::is_base_of<SerializableDeformerDataBase, T>::value, "Class needs to be SerializableDeformerDataBase");

	assert(pDeformer);
	if(mUseDataInstancing && pDeformer->getInstancingState()) {
		for(const auto* pHandle: handles) {
			assert(pHandle);
			if(!pHandle->getPath().IsAbsolutePath()) {
				std::cerr << "Invalid handle path: " << pHandle->getPath() << "! DeformerDataCache relative keys are not supported !" << std::endl;
				return nullptr;
			}
		}

		const DeformerDataCache::Key key(std::type_index(typeid(T)), handles);
		const std::lock_guard<std::mutex> lock(mMutex);

		auto it = mDataMap.find(key);
		if (it != mDataMap.end()) {
			LOG_DBG << "Data " << typeid(T).name() << " found in cache";
			return std::dynamic_pointer_cast<T>(it->second);
		}

		LOG_DBG << "Data " << typeid(T).name() << " placed in cache";
		auto result = mDataMap.emplace(key, std::make_shared<T>());
		created = true;
		return std::dynamic_pointer_cast<T>(result.first->second);
	} else {
		return std::dynamic_pointer_cast<T>(std::make_shared<T>());
	}
}

void DeformerDataCache::clear() {
	mDataMap.clear();
	if(mDataMap.empty()) {
		mTopologyPool.clear();
	}
}

void DeformerDataCache::invalidate(const UsdPrimHandle& handle) {
	const std::lock_guard<std::mutex> lock(mMutex);
	const std::lock_guard<std::mutex> topo_lock(mTopologyPoolMutex);

	LOG_ERR << "DeformerDataCache::invalidate(const UsdPrimHandle& handle) UNIMPLEMENTED !!!";
}

DeformerDataCache::~DeformerDataCache() { }

DeformerDataCache::DeformerDataCache() { 
	mUseDataInstancing = CurvesDeformerFactory::getInstance().getDataInstancingState();
}


template std::shared_ptr<SerializablePhantomTrimesh> DeformerDataCache::getOrCreateData(const BaseCurvesDeformer* pDeformer, const UsdPrimHandle& handle, bool& created);
template std::shared_ptr<SerializablePhantomTrimesh> DeformerDataCache::getOrCreateData(const BaseCurvesDeformer* pDeformer, const std::vector<const UsdPrimHandle*>& handles, bool& created);

template std::shared_ptr<SerializableUsdGeomMeshFaceAdjacency> DeformerDataCache::getOrCreateData(const BaseCurvesDeformer* pDeformer, const UsdPrimHandle& handle, bool& created);
template std::shared_ptr<SerializableUsdGeomMeshFaceAdjacency> DeformerDataCache::getOrCreateData(const BaseCurvesDeformer* pDeformer, const std::vector<const UsdPrimHandle*>& handles, bool& created);

template std::shared_ptr<FastCurvesDeformerData> DeformerDataCache::getOrCreateData(const BaseCurvesDeformer* pDeformer, const UsdPrimHandle& handle, bool& created);
template std::shared_ptr<FastCurvesDeformerData> DeformerDataCache::getOrCreateData(const BaseCurvesDeformer* pDeformer, const std::vector<const UsdPrimHandle*>& handles, bool& created);

} // namespace Piston

// Initialize static members
Piston::DeformerDataCache* Piston::DeformerDataCache::mInstancePtr = nullptr;
std::mutex Piston::DeformerDataCache::mMutex;
std::mutex Piston::DeformerDataCache::mTopologyPoolMutex;