#ifndef PISTON_LIB_CURVES_CONTAINER_H_
#define PISTON_LIB_CURVES_CONTAINER_H_

#include "framework.h"
#include "kdtree.hpp"
#include "base_hair_deformer.h"

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/curves.h>
#include <pxr/base/gf/matrix3f.h>

#include <limits>
#include <string>
#include <array>
#include <unordered_map>
#include <memory>


namespace Piston {

class PxrCurvesContainer {
	public:
		using UniquePtr = std::unique_ptr<PxrCurvesContainer>;

		static UniquePtr create(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode reference_time_code = pxr::UsdTimeCode::Default());

	private:
		PxrCurvesContainer();
	
	private:
		size_t                                  mCurvesCount;
		pxr::VtArray<int> 						mCurveVertexCounts;
		std::vector<uint32_t> 					mCurveOffsets;
};

} // namespace Piston

#endif // PISTON_LIB_CURVES_CONTAINER_H_