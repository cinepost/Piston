#include "deformer_factory.h"
#include "logging.h"

#include <algorithm>
#include <cctype>


namespace Piston {

static constexpr size_t kDefaultPxrPointsLRUCacheMaxSize = 1024 * 1024 * 256 * 4; 

static std::string tolower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
    return s;
}

CurvesDeformerFactory& CurvesDeformerFactory::getInstance() {
    if (mInstancePtr == nullptr) {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mInstancePtr == nullptr) {
            mInstancePtr = new CurvesDeformerFactory();
        }
    }

    return *mInstancePtr;
}

FastCurvesDeformer::SharedPtr CurvesDeformerFactory::getFastDeformer(const std::string& name) {
	return std::dynamic_pointer_cast<FastCurvesDeformer>(getInstance().getDeformer(BaseCurvesDeformer::Type::FAST, name));
}

WrapCurvesDeformer::SharedPtr CurvesDeformerFactory::getWrapDeformer(const std::string& name) {
	return std::dynamic_pointer_cast<WrapCurvesDeformer>(getInstance().getDeformer(BaseCurvesDeformer::Type::WRAP, name));
}

GuideCurvesDeformer::SharedPtr CurvesDeformerFactory::getGuidesDeformer(const std::string& name) {
	return std::dynamic_pointer_cast<GuideCurvesDeformer>(getInstance().getDeformer(BaseCurvesDeformer::Type::GUIDES, name));
}

BaseCurvesDeformer::SharedPtr CurvesDeformerFactory::getDeformer(BaseCurvesDeformer::Type type, const std::string& name) {
	const CurvesDeformerFactory::Key key = {type, name};
	auto it = mDeformers.find(key);
	if(it != mDeformers.end()) {
		return it->second;
	}

	switch(type) {
		case BaseCurvesDeformer::Type::WRAP: 
		{
			auto result = mDeformers.emplace(key, WrapCurvesDeformer::create(name));
			if(result.second) return result.first->second;
			throw std::runtime_error("Error creating WrapCurvesDeformer !");
		}
		case BaseCurvesDeformer::Type::GUIDES: 
		{
			auto result = mDeformers.emplace(key, GuideCurvesDeformer::create(name));
			if(result.second) return result.first->second;
			throw std::runtime_error("Error creating GuideCurvesDeformer !");
		}
		case BaseCurvesDeformer::Type::FAST:
		default:
		{
			auto result = mDeformers.emplace(key, FastCurvesDeformer::create(name));
			if(result.second) return result.first->second;
			throw std::runtime_error("Error creating FastCurvesDeformer !");
		}
	}
}

void CurvesDeformerFactory::setPointsCacheUsageState(bool state) {
	CurvesDeformerFactory& factory = getInstance();
	std::lock_guard<std::mutex> lock(factory.mMutex);

	if(state) {
		if(factory.mpPxrPointsLRUCache) return;

		factory.mpPxrPointsLRUCache = PxrPointsLRUCache::create(kDefaultPxrPointsLRUCacheMaxSize);
		LOG_DBG << "PxrPointsLRUCache enabled.";
	} else {
		if(!factory.mpPxrPointsLRUCache) return;

		factory.mpPxrPointsLRUCache->clear();
		factory.mpPxrPointsLRUCache = nullptr;
		LOG_DBG << "PxrPointsLRUCache disabled.";
	}
}

bool CurvesDeformerFactory::getPointsCacheUsageState() {
	CurvesDeformerFactory& factory = getInstance();
	std::lock_guard<std::mutex> lock(factory.mMutex);
	return factory.mpPxrPointsLRUCache != nullptr;
}

void CurvesDeformerFactory::setDefaultRestTimeCode(pxr::UsdTimeCode time_code) {
	CurvesDeformerFactory& factory = getInstance();
	if(factory.getDefaultRestTimeCode() == time_code) return;

	std::lock_guard<std::mutex> lock(factory.mMutex);
	factory.mDefaultRestTimeCode = time_code;
}

pxr::UsdTimeCode CurvesDeformerFactory::getDefaultRestTimeCode() {
	CurvesDeformerFactory& factory = getInstance();
	std::lock_guard<std::mutex> lock(factory.mMutex);

	return factory.mDefaultRestTimeCode;
}

void CurvesDeformerFactory::clear() {
	CurvesDeformerFactory& factory = getInstance();
	std::lock_guard<std::mutex> lock(factory.mMutex);
	
	factory.mDeformers.clear();
	if(factory.mpPxrPointsLRUCache) {
		factory.mpPxrPointsLRUCache->clear();
	}
}

CurvesDeformerFactory::~CurvesDeformerFactory() {
	//SimpleProfiler::printReport();
}

CurvesDeformerFactory::CurvesDeformerFactory(): mDefaultRestTimeCode(pxr::UsdTimeCode::Default()) {
	bool enable_cache = true;
	std::string cache_var_value;
	if(getEnvVar("PISTON_PTCACHE", cache_var_value)) {
		cache_var_value = tolower(cache_var_value) ;
		if(cache_var_value == "off" || cache_var_value == "false" || cache_var_value == "0") {
			enable_cache = false;
		}
	}
	mpPxrPointsLRUCache = enable_cache ? PxrPointsLRUCache::create(kDefaultPxrPointsLRUCacheMaxSize) : nullptr;

	std::string tpose_default_frame_string;
	if(getEnvVar("PISTON_DEFAULT_TPOSE_FRAME", tpose_default_frame_string)) {
		try {
        	float d = std::stod(tpose_default_frame_string	);
        	mDefaultRestTimeCode = d;
		} catch (const std::invalid_argument& e) {
        	LOG_ERR << "Invalid \"PISTON_DEFAULT_TPOSE_FRAME\" environment variable: " << e.what();
    	} catch (const std::out_of_range& e) {
        	LOG_ERR << "\"PISTON_DEFAULT_TPOSE_FRAME\" environment variable out of range: " << e.what();
    	}
	}

	if(!mDefaultRestTimeCode.IsDefault()) {
		LOG_INF << "Default system T-Pose frame is " << mDefaultRestTimeCode;
	}
}


} // namespace Piston

// Initialize static members
Piston::CurvesDeformerFactory* Piston::CurvesDeformerFactory::mInstancePtr = nullptr;
std::mutex Piston::CurvesDeformerFactory::mMutex;