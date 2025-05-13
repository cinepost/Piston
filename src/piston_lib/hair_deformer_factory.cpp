#include "hair_deformer_factory.h"

HairDeformerFactory& HairDeformerFactory::getInstance() {
    if (mInstancePtr == nullptr) {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mInstancePtr == nullptr) {
            mInstancePtr = new HairDeformerFactory();
        }
    }
    return *mInstancePtr;
}

FastHairDeformer::SharedPtr HairDeformerFactory::getFastDeformer(const std::string& name) {
	return std::dynamic_pointer_cast<FastHairDeformer>(getDeformer(BaseHairDeformer::Type::FAST, name));
}

BaseHairDeformer::SharedPtr HairDeformerFactory::getDeformer(BaseHairDeformer::Type type, const std::string& name) {
	const HairDeformerFactory::Key key = {type, name};
	auto it = mDeformers.find(key);
	if(it != mDeformers.end()) {
		return it->second;
	}

	switch(type) {
		case BaseHairDeformer::Type::FAST:
		default:
			auto result = mDeformers.emplace(key, FastHairDeformer::create());
			if(result.second) return result.first->second;
			throw std::runtime_error("Error creating FastHairDeformer !");
	}
}

// Initialize static members
HairDeformerFactory* HairDeformerFactory::mInstancePtr = nullptr;
std::mutex HairDeformerFactory::mMutex;