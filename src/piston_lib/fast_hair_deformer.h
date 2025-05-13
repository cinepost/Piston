#ifndef PISTON_LIB_FAST_HAIR_DEFORMER_H_
#define PISTON_LIB_FAST_HAIR_DEFORMER_H_

#include "framework.h"
#include "base_hair_deformer.h"

#include <memory>
#include <string>
#include <pxr/usd/usd/prim.h>

class FastHairDeformer : public BaseHairDeformer, public inherit_shared_from_this<BaseHairDeformer, FastHairDeformer> {
	public:
		using SharedPtr = std::shared_ptr<FastHairDeformer>;
		
	public:
		static SharedPtr create();

		virtual const std::string& greet() const override;

	protected:
		FastHairDeformer();
};

#endif // PISTON_LIB_FAST_HAIR_DEFORMER_H_