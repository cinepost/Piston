#include "deformer_factory.h"
#include "deformer_data_cache.h"
#include "logging.h"

#include <algorithm>
#include <cctype>


namespace Piston {

static const bool sDataInstancingDefaultState = true;
static const pxr::SdfPath sDefaultPrimPath("/__piston_data__");
static const CurvesDeformerFactory::DataToPrimStorageMethod sDefaultDataToPrimStorage(CurvesDeformerFactory::DataToPrimStorageMethod::ATTRIBUTE);

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

CurvesDeformerFactory::DeformersMap& CurvesDeformerFactory::deformers() { 
	CurvesDeformerFactory& factory = getInstance();
	std::lock_guard<std::mutex> lock(factory.mMutex);
	return factory.mDeformers; 
}

void CurvesDeformerFactory::deleteDeformer(BaseCurvesDeformer::Type type, const std::string& name) {
	const CurvesDeformerFactory::Key key = {type, name};
	CurvesDeformerFactory& factory = getInstance();
	std::lock_guard<std::mutex> lock(factory.mMutex);

	auto it = factory.mDeformers.find(key);
	if(it != factory.mDeformers.end()) {
		factory.mDeformers.erase(key);
	}
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
	std::lock_guard<std::mutex> lock(mMutex);

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

void CurvesDeformerFactory::setDataInstancingState(bool state) {
	CurvesDeformerFactory& factory = getInstance();
	std::lock_guard<std::mutex> lock(factory.mMutex);

	if(factory.mDataInstancingState == state) return;
	factory.mDataInstancingState = state;
}

bool CurvesDeformerFactory::getDataInstancingState() {
	CurvesDeformerFactory& factory = getInstance();
	std::lock_guard<std::mutex> lock(factory.mMutex);
	return factory.mDataInstancingState;
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

void CurvesDeformerFactory::setDefaultDataPrimPath(const std::string& path) {
	const pxr::SdfPath new_path(path);
	CurvesDeformerFactory& factory = getInstance();
	if(factory.getDefaultDataPrimPath() == new_path) return;

	std::lock_guard<std::mutex> lock(factory.mMutex);
	factory.mDefaultDataPrimPath = new_path;
}

const pxr::SdfPath& CurvesDeformerFactory::getDefaultDataPrimPath() {
	CurvesDeformerFactory& factory = getInstance();
	std::lock_guard<std::mutex> lock(factory.mMutex);

	return factory.mDefaultDataPrimPath;
}

bool CurvesDeformerFactory::isDefaultDataPrimPath(const std::string& path) {
	return CurvesDeformerFactory::isDefaultDataPrimPath(pxr::SdfPath(path));
}

bool CurvesDeformerFactory::isDefaultDataPrimPath(const pxr::SdfPath& path) {
	CurvesDeformerFactory& factory = getInstance();
	std::lock_guard<std::mutex> lock(factory.mMutex);

	return factory.mDefaultDataPrimPath == path;
}

CurvesDeformerFactory::DataToPrimStorageMethod CurvesDeformerFactory::getDataStorageMethod() {
	CurvesDeformerFactory& factory = getInstance();
	std::lock_guard<std::mutex> lock(factory.mMutex);

	return factory.mDataToPrimStorageMethod;
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

CurvesDeformerFactory::CurvesDeformerFactory(): mDataToPrimStorageMethod(sDefaultDataToPrimStorage), mDefaultRestTimeCode(pxr::UsdTimeCode::Default()), mDefaultDataPrimPath(sDefaultPrimPath), mDataInstancingState(sDataInstancingDefaultState) {
	std::cout << std::endl;
	
	bool enable_cache = true;
	std::string cache_var_value;
	if(getEnvVar("PISTON_PTCACHE", cache_var_value)) {
		cache_var_value = tolower(cache_var_value) ;
		if(cache_var_value == "off" || cache_var_value == "false" || cache_var_value == "0") {
			enable_cache = false;
		}
	}
	mpPxrPointsLRUCache = enable_cache ? PxrPointsLRUCache::create(kDefaultPxrPointsLRUCacheMaxSize) : nullptr;
	LOG_INF << "Point cache is " << (mpPxrPointsLRUCache ? "ON" : "OFF");

	std::string data_instancing_var_value;
	if(getEnvVar("PISTON_DATA_INSTANCING", data_instancing_var_value)) {
		data_instancing_var_value = tolower(cache_var_value) ;
		if(data_instancing_var_value == "off" || data_instancing_var_value == "false" || data_instancing_var_value == "0") {
			mDataInstancingState = false;
		} else {
			mDataInstancingState = true;
		}
	}
	LOG_INF << "Deformers data instancing is " << (mDataInstancingState ? "ON" : "OFF");

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

	std::string default_data_prim_path_override;
	if(getEnvVar("PISTON_DEFAULT_DATA_PRIM_PATH", default_data_prim_path_override)) {
		const pxr::SdfPath new_path(default_data_prim_path_override);
		if(new_path.IsPrimPath()) {
			mDefaultDataPrimPath = new_path;
		} else {
			LOG_ERR << "Unable to set default data prim path to \"" << default_data_prim_path_override << "\". Path is not a prim path !";
		}
	}

	if(mDefaultDataPrimPath != sDefaultPrimPath) {
		LOG_INF << "Default system data prim path is " << mDefaultDataPrimPath;
	}

	std::string data_to_prim_storage_method;
	if(getEnvVar("PISTON_DATA_TO_PRIM_STORAGE", data_to_prim_storage_method)) {
		auto default_storage_method = mDataToPrimStorageMethod;
		if(tolower(data_to_prim_storage_method) == "metadata") {
			mDataToPrimStorageMethod = DataToPrimStorageMethod::METADATA;
		} else if (tolower(data_to_prim_storage_method) == "attribute") {
			mDataToPrimStorageMethod = DataToPrimStorageMethod::ATTRIBUTE;
		} else {
			LOG_ERR << "Unknown data storage method \"" << data_to_prim_storage_method << "\" !!! Reverting to default method (" << to_string(default_storage_method) << ").";
		}
		if(mDataToPrimStorageMethod != sDefaultDataToPrimStorage) {
			LOG_INF << "Data storage method is \"" << to_string(mDataToPrimStorageMethod) << "\"";
		}
	}
}

} // namespace Piston

// Initialize static members
Piston::CurvesDeformerFactory* Piston::CurvesDeformerFactory::mInstancePtr = nullptr;
std::mutex Piston::CurvesDeformerFactory::mMutex;