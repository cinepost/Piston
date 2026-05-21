#ifndef PISTON_LIB_GLOBAL_CONFIG_H_
#define PISTON_LIB_GLOBAL_CONFIG_H_

#include "os.h"
#include "deformer_factory.h"
#include "simple_profiler.h"

#include <string>
#include <vector>
#include <map>
#include <mutex>

namespace Piston {

/*
 * Factory singleton class
 */
class GlobalConfig {
	public:
		using DataToPrimStorageMethod = CurvesDeformerFactory::DataToPrimStorageMethod;

	public:

		~GlobalConfig();
		
    	// Deleting the copy constructor to prevent copies
    	GlobalConfig(const GlobalConfig& obj) = delete;

    	// Static method to get the GlobalConfig instance
	    static GlobalConfig& getInstance();

	    void setDataInstancingState(bool state);
	    bool getDataInstancingState() const;

		void setPointsCacheUsageState(bool state);
		bool getPointsCacheUsageState() const;

		void setDefaultRestTimeCode(pxr::UsdTimeCode time_code);
		pxr::UsdTimeCode getDefaultRestTimeCode()const ;

		void setDefaultDataPrimPath(const std::string& path);
		pxr::SdfPath getDefaultDataPrimPath() const;
		bool isDefaultDataPrimPath(const std::string& path) const;
		bool isDefaultDataPrimPath(const pxr::SdfPath& path) const;

		DataToPrimStorageMethod getDataStorageMethod() const;

	private:
		// Mutex to ensure thread safety
    	static std::mutex mMutex;

    	// Static pointer to the GlobalConfig instance
    	static GlobalConfig* mInstancePtr;

    private:
    	DataToPrimStorageMethod 	mDataToPrimStorageMethod;
    	pxr::UsdTimeCode 			mDefaultRestTimeCode;
    	pxr::SdfPath    		 	mDefaultDataPrimPath;
    	bool                        mPointCacheState;
    	bool                        mDataInstancingState;

    	GlobalConfig();
};

} // namespace Piston

#endif // PISTON_LIB_DEFORMER_FACTORY_H_