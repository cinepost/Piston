#ifndef PISTON_LIB_DEFORMER_DATA_CACHE_H_
#define PISTON_LIB_DEFORMER_DATA_CACHE_H_

#include "framework.h"
#include "common.h"
#include "topology.h"
#include "serializable_data.h"
#include "simple_profiler.h"
#include "logging.h"

#include <string>
#include <vector>
#include <map>
#include <mutex>


namespace Piston {

/*
 * Factory singleton class
 */
class DeformerDataCache {
	public:
	struct Key {
		std::type_index 	type_idx;
		pxr::SdfPath 		path;
		size_t              topology_hash;
		PxrTopologyVariant 	topology_variant;

		bool operator==(const Key& other) const {
			if(type_idx != other.type_idx) return false;
			if(path == other.path) return true;

			if(topology_hash == other.topology_hash) {
				return topology_variant == other.topology_variant;
			}
			return false;
		}

		bool operator<(const Key& other) const {
        	if (type_idx != other.type_idx) return type_idx < other.type_idx;

        	if (path == other.path) return false;

        	if (topology_hash != other.topology_hash) return topology_hash < other.topology_hash;

        	if (topology_variant == other.topology_variant) return false; 

        	if(topology_variant < other.topology_variant) return true;

        	return path < other.path;
		}

		Key(const std::type_index& _type_idx, const UsdPrimHandle& handle);
	};

	using MapType = std::map<Key, std::shared_ptr<SerializableDeformerDataBase>>; // we dont's expect large number of entries here. Also we might opt fo std::string as a key so we use std::map for now  

	public:
		~DeformerDataCache();

		// Deleting the copy constructor to prevent copies
		DeformerDataCache(const DeformerDataCache& obj) = delete;

		// Static method to get the CurvesDeformerFactory instance
		static DeformerDataCache& getInstance();

//		template< class T>
//		std::shared_ptr<T> getOrCreateData(const std::string& name);

		template< class T>
		std::shared_ptr<T> getOrCreateData(const UsdPrimHandle& handle);

//		template< class T>
//		std::shared_ptr<T> getOrCreateData(const pxr::SdfPath& path);

	protected:
		void clear();

	private:
		MapType mDataMap;

		// Mutex to ensure thread safety
		static std::mutex mMutex;

		// Static pointer to the CurvesDeformerFactory instance
		static DeformerDataCache* mInstancePtr;

		DeformerDataCache();
};

} // namespace Piston

#endif // PISTON_LIB_CURVES_DEFORMER_FACTORY_H_