#ifndef PISTON_LIB_DEFORMER_FACTORY_H_
#define PISTON_LIB_DEFORMER_FACTORY_H_

#include "base_curves_deformer.h"
#include "fast_curves_deformer.h"
#include "wrap_curves_deformer.h"
#include "guide_curves_deformer.h"
#include "pxr_points_lru_cache.h"
#include "os.h"
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

		enum class DataToPrimStorageMethod {
			METADATA = 0,
			ATTRIBUTE = 1
		};

	public:
		~CurvesDeformerFactory();
		
    	// Deleting the copy constructor to prevent copies
    	CurvesDeformerFactory(const CurvesDeformerFactory& obj) = delete;

    	// Static method to get the CurvesDeformerFactory instance
	    static CurvesDeformerFactory& getInstance();

	    static FastCurvesDeformer::SharedPtr getFastDeformer(const std::string& name);
	    static WrapCurvesDeformer::SharedPtr getWrapDeformer(const std::string& name);
	    static GuideCurvesDeformer::SharedPtr getGuidesDeformer(const std::string& name);

		static void setPointsCacheUsageState(bool state);
		static bool getPointsCacheUsageState();

		static void setDefaultRestTimeCode(pxr::UsdTimeCode time_code);
		static pxr::UsdTimeCode getDefaultRestTimeCode();

		static void setDefaultDataPrimPath(const std::string& path);
		static const pxr::SdfPath& getDefaultDataPrimPath();
		static bool isDefaultDataPrimPath(const std::string& path);
		static bool isDefaultDataPrimPath(const pxr::SdfPath& path);

		static DataToPrimStorageMethod getDataStorageMethod();

	    static void clear();

	    PxrPointsLRUCache* getPxrPointsLRUCachePtr() { return mpPxrPointsLRUCache.get(); }

	private:
		BaseCurvesDeformer::SharedPtr getDeformer(BaseCurvesDeformer::Type type, const std::string& name);

	private:
		std::map<Key, BaseCurvesDeformer::SharedPtr> mDeformers;
		PxrPointsLRUCache::UniquePtr mpPxrPointsLRUCache;

		// Mutex to ensure thread safety
    	static std::mutex mMutex;

    	// Static pointer to the CurvesDeformerFactory instance
    	static CurvesDeformerFactory* mInstancePtr;

    private:
    	DataToPrimStorageMethod 	mDataToPrimStorageMethod;
    	pxr::UsdTimeCode 			mDefaultRestTimeCode;
    	pxr::SdfPath    		 	mDefaultDataPrimPath;

    	CurvesDeformerFactory();
};

inline std::string to_string(const CurvesDeformerFactory::DataToPrimStorageMethod& m) {
	switch(m) {
		case CurvesDeformerFactory::DataToPrimStorageMethod::METADATA:
			return "METADATA";
		case CurvesDeformerFactory::DataToPrimStorageMethod::ATTRIBUTE:
		default:
			return "ATTRIBUTE"; 
	}
} 

} // namespace Piston

#endif // PISTON_LIB_DEFORMER_FACTORY_H_