#include "curves_container.h"

namespace Piston {


PxrCurvesContainer::PxrCurvesContainer() {

}

PxrCurvesContainer::UniquePtr PxrCurvesContainer::create(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode reference_time_code) {
	auto geom_curves = pxr::UsdGeomCurves(prim_handle.getPrim());
	if(!geom_curves) {
		printf("Error getting hair geometry !\n");
		return nullptr;
	}

	const size_t curves_count = geom_curves.GetCurveCount(reference_time_code);
	if(curves_count == 0) {
		printf("No curves exist in curves primitive !\n");
		return nullptr;
	}

	PxrCurvesContainer::UniquePtr pResult = PxrCurvesContainer::UniquePtr(new PxrCurvesContainer());

	return pResult;
}
	

} // namespace Piston
