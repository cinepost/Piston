#include "global_config.h"
#include "deformer_factory.h"
#include "deformer_data_cache.h"
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
	auto& instance = CurvesDeformerFactory::getInstance();
	static const auto& conf = GlobalConfig::getInstance();

	{
		const std::lock_guard<std::mutex> lock(instance.mMutex);
		if(instance.mPointCacheState == state) return;
		instance.mPointCacheState = conf.getPointsCacheUsageState() && state;
	}

	LOG_INF << "Point cache is " << (instance.mpPxrPointsLRUCache ? "ON" : "OFF");
}

bool CurvesDeformerFactory::getPointsCacheUsageState() {
	auto& instance = CurvesDeformerFactory::getInstance();
	static const auto& conf = GlobalConfig::getInstance();
	const std::lock_guard<std::mutex> lock(instance.mMutex);

	return instance.mPointCacheState && conf.getPointsCacheUsageState();
}

void CurvesDeformerFactory::clear() {
	CurvesDeformerFactory& factory = getInstance();
	const std::lock_guard<std::mutex> lock(factory.mMutex);
	
	factory.mDeformers.clear();
	if(factory.mpPxrPointsLRUCache) {
		factory.mpPxrPointsLRUCache->clear();
	}
}

PxrPointsLRUCache*  CurvesDeformerFactory::getPxrPointsLRUCachePtr() {
	const bool cache_enabled = CurvesDeformerFactory::getPointsCacheUsageState();

	PxrPointsLRUCache* p_cache = nullptr;
	{
		const std::lock_guard<std::mutex> lock(mMutex); 
	
		if(cache_enabled && !mpPxrPointsLRUCache) {
			mpPxrPointsLRUCache = PxrPointsLRUCache::create(kDefaultPxrPointsLRUCacheMaxSize);
		} else if(!cache_enabled && mpPxrPointsLRUCache ) {
			mpPxrPointsLRUCache = nullptr;
		}
		p_cache = mpPxrPointsLRUCache.get(); 
	}
	return p_cache;
}

CurvesDeformerFactory::~CurvesDeformerFactory() {
	//SimpleProfiler::printReport();
}

CurvesDeformerFactory::CurvesDeformerFactory(): mpPxrPointsLRUCache(nullptr) {

}

} // namespace Piston

// Initialize static members
Piston::CurvesDeformerFactory* Piston::CurvesDeformerFactory::mInstancePtr = nullptr;
std::mutex Piston::CurvesDeformerFactory::mMutex;