#ifndef PISTON_LIB_HAIR_DEFORMER_FACTORY_H_
#define PISTON_LIB_HAIR_DEFORMER_FACTORY_H_

#include "base_hair_deformer.h"
#include "fast_hair_deformer.h"

#include <string>
#include <vector>
#include <map>
#include <mutex>

/*
 * Factory singleton class
 */
class HairDeformerFactory {
	public:
		struct Key {
			BaseHairDeformer::Type type;
			std::string name;

			bool operator< (const Key &other) const {   
               if(type == other.type) return name < other.name;
               return type < other.type; 
            } 
		};

	public:
    	// Deleting the copy constructor to prevent copies
    	HairDeformerFactory(const HairDeformerFactory& obj) = delete;

    	// Static method to get the HairDeformerFactory instance
	    static HairDeformerFactory& getInstance();

	    FastHairDeformer::SharedPtr getFastDeformer(const std::string& name);

	private:
		BaseHairDeformer::SharedPtr getDeformer(BaseHairDeformer::Type type, const std::string& name);

	private:
		std::map<Key, BaseHairDeformer::SharedPtr> mDeformers;

		// Mutex to ensure thread safety
    	static std::mutex mMutex;

    	// Static pointer to the HairDeformerFactory instance
    	static HairDeformerFactory* mInstancePtr;

    	HairDeformerFactory() {}
};

#endif // PISTON_LIB_HAIR_DEFORMER_FACTORY_H_