#include "curves_deformer_factory.h"

namespace Piston {

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

BaseCurvesDeformer::SharedPtr CurvesDeformerFactory::getDeformer(BaseCurvesDeformer::Type type, const std::string& name) {
	const CurvesDeformerFactory::Key key = {type, name};
	auto it = mDeformers.find(key);
	if(it != mDeformers.end()) {
		return it->second;
	}

	switch(type) {
		case BaseCurvesDeformer::Type::WRAP: 
		{
			auto result = mDeformers.emplace(key, WrapCurvesDeformer::create());
			if(result.second) return result.first->second;
			throw std::runtime_error("Error creating WrapCurvesDeformer !");
		}
		case BaseCurvesDeformer::Type::FAST:
		default:
		{
			auto result = mDeformers.emplace(key, FastCurvesDeformer::create());
			if(result.second) return result.first->second;
			throw std::runtime_error("Error creating FastCurvesDeformer !");
		}
	}
}

} // namespace Piston

// Initialize static members
Piston::CurvesDeformerFactory* Piston::CurvesDeformerFactory::mInstancePtr = nullptr;
std::mutex Piston::CurvesDeformerFactory::mMutex;