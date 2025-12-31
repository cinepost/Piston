#include "guide_curves_container.h"


namespace Piston {

PxrCurvesContainer::PxrCurvesContainer(): mCurvesCount(0) {
	mCurveOffsets.reserve(1024);
}

GuideCurvesContainer::SharedPtr GuideCurvesContainer::create() {
	return GuideCurvesContainer::UniquePtr(new GuideCurvesContainer());
}

} // namespace Piston
