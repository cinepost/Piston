#ifndef PISTON_LIB_DEFORMER_DATA_CACHE_H_
#define PISTON_LIB_DEFORMER_DATA_CACHE_H_

#include "framework.h"
#include "common.h"
#include "serializable_data.h"
#include "simple_profiler.h"

#include <string>
#include <vector>
#include <map>
#include <mutex>

namespace Piston {

static const bool kDefaultCacheState = false;

/*
 * Factory singleton class
 */
class DeformerDataCache {
	public:
	using Key = pxr::SdfPath;
	using MapType = std::map<Key, std::shared_ptr<SerializableDeformerDataBase>>; // we dont's expect large number of entries here. Also we might opt fo std::string as a key so we use std::map for now  

	public:
		~DeformerDataCache();

		// Deleting the copy constructor to prevent copies
		DeformerDataCache(const DeformerDataCache& obj) = delete;

		// Static method to get the CurvesDeformerFactory instance
		static DeformerDataCache& getInstance();

		template< class T>
		std::shared_ptr<T> getOrCreateData(const std::string& name);

		template< class T>
		std::shared_ptr<T> getOrCreateData(const Key& key);

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