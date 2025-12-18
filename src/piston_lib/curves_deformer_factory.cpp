#include "curves_deformer_factory.h"

namespace Piston {

static constexpr size_t kDefaultPxrPointsLRUCacheMaxSize = 1024 * 1024 * 256 * 4; 

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

GuidesCurvesDeformer::SharedPtr CurvesDeformerFactory::getGuidesDeformer(const std::string& name) {
	return std::dynamic_pointer_cast<GuidesCurvesDeformer>(getInstance().getDeformer(BaseCurvesDeformer::Type::GUIDE, name));
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
		case BaseCurvesDeformer::Type::GUIDE: 
		{
			auto result = mDeformers.emplace(key, GuidesCurvesDeformer::create(name));
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

void CurvesDeformerFactory::clear() {
	CurvesDeformerFactory& factory = getInstance();
	std::lock_guard<std::mutex> lock(factory.mMutex);
	factory.mDeformers.clear();
	if(factory.mpPxrPointsLRUCache) {
		factory.mpPxrPointsLRUCache->clear();
	}
}

CurvesDeformerFactory::~CurvesDeformerFactory() {
	SimpleProfiler::printReport();
}

CurvesDeformerFactory::CurvesDeformerFactory() {
	mpPxrPointsLRUCache = PxrPointsLRUCache::create(kDefaultPxrPointsLRUCacheMaxSize);
}


} // namespace Piston

// Initialize static members
Piston::CurvesDeformerFactory* Piston::CurvesDeformerFactory::mInstancePtr = nullptr;
std::mutex Piston::CurvesDeformerFactory::mMutex;