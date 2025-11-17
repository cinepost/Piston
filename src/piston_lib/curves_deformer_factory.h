#ifndef PISTON_LIB_CURVES_DEFORMER_FACTORY_H_
#define PISTON_LIB_CURVES_DEFORMER_FACTORY_H_

#include "base_curves_deformer.h"
#include "fast_curves_deformer.h"
#include "wrap_curves_deformer.h"
#include "pxr_points_lru_cache.h"

#include "simple_profiler.h"

#include <string>
#include <vector>
#include <map>
#include <mutex>

namespace Piston {

/*
 * Factory singleton class
 */
class CurvesDeformerFactory {
	public:
		struct Key {
			BaseCurvesDeformer::Type type;
			std::string name;

			bool operator< (const Key &other) const {   
               if(type == other.type) return name < other.name;
               return type < other.type; 
            } 
		};

	public:
		~CurvesDeformerFactory();
		
    	// Deleting the copy constructor to prevent copies
    	CurvesDeformerFactory(const CurvesDeformerFactory& obj) = delete;

    	// Static method to get the CurvesDeformerFactory instance
	    static CurvesDeformerFactory& getInstance();

	    static FastCurvesDeformer::SharedPtr getFastDeformer(const std::string& name);
	    static WrapCurvesDeformer::SharedPtr getWrapDeformer(const std::string& name);

	    PxrPointsLRUCache* getPxrPointsLRUCachePtr();

	private:
		BaseCurvesDeformer::SharedPtr getDeformer(BaseCurvesDeformer::Type type, const std::string& name);

	private:
		std::map<Key, BaseCurvesDeformer::SharedPtr> mDeformers;
		PxrPointsLRUCache::UniquePtr mpPxrPointsLRUCache;

		// Mutex to ensure thread safety
    	static std::mutex mMutex;

    	// Static pointer to the CurvesDeformerFactory instance
    	static CurvesDeformerFactory* mInstancePtr;



    	CurvesDeformerFactory() {}
};

} // namespace Piston

#endif // PISTON_LIB_CURVES_DEFORMER_FACTORY_H_