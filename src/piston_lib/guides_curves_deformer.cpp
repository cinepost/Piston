#include "simple_profiler.h"
#include "guides_curves_deformer.h"

#include "common.h"
#include "kdtree.hpp"
#include "geometry_tools.h"

#include <pxr/base/gf/matrix4f.h>

#include <random>
#include <algorithm>


namespace Piston {

GuidesCurvesDeformer::GuidesCurvesDeformer(const std::string& name): BaseCurvesDeformer(BaseCurvesDeformer::Type::GUIDES, name) {
	dbg_printf("GuidesCurvesDeformer::GuidesCurvesDeformer(%s)\n", name.c_str());
	mpGuidesCurvesDeformerData = std::make_unique<GuidesCurvesDeformerData>();
	mpLiveGuideCurvesContainer = PxrCurvesContainer::create();
}

GuidesCurvesDeformer::SharedPtr GuidesCurvesDeformer::create(const std::string& name) {
	return SharedPtr(new GuidesCurvesDeformer(name));
}

bool GuidesCurvesDeformer::validateDeformerGeoPrim(const pxr::UsdPrim& geoPrim) {
	return isBasisCurvesGeoPrim(geoPrim);
}

const std::string& GuidesCurvesDeformer::toString() const {
	static const std::string kFastDeformerString = "GuidesCurvesDeformer";
	return kFastDeformerString;
}

void GuidesCurvesDeformer::setGuideIDPrimAttrName(const std::string& name) {
	if(mGuideIDPrimAttrName == name) return;
	mGuideIDPrimAttrName = name;
	makeDirty();
}

bool GuidesCurvesDeformer::deformImpl(PointsList& points, pxr::UsdTimeCode time_code) {
	return __deform__(points, false, time_code);
}

bool GuidesCurvesDeformer::deformMtImpl(PointsList& points, pxr::UsdTimeCode time_code) {
	return __deform__(points, true, time_code);
}

bool GuidesCurvesDeformer::__deform__(PointsList& points, bool multi_threaded, pxr::UsdTimeCode time_code) {
	if(!mpLiveGuideCurvesContainer->update(mDeformerGeoPrimHandle, time_code)) {
		std::cerr << "Error updating guide curves from prim" << mDeformerGeoPrimHandle.getPath().GetText() << " !" << std::endl;
		return false;
	}
	
	return true;
}

bool GuidesCurvesDeformer::buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	mpRestGuideCurvesContainer = PxrCurvesContainer::create(mDeformerGeoPrimHandle, rest_time_code);
	if(!mpRestGuideCurvesContainer) {
		std::cerr << "Error creating guide curves container !" << std::endl;
		return false;
	}

	return true;
}

bool GuidesCurvesDeformer::writeJsonDataToPrimImpl() const {
	if(!mCurvesGeoPrimHandle.writeDataToBson(mpGuidesCurvesDeformerData.get())) {
		std::cerr << "Error writing " << mpGuidesCurvesDeformerData->typeName() << " deformer data to json !";	
		return false;
	}
	return true;
}

GuidesCurvesDeformer::~GuidesCurvesDeformer() {
	PROFILE_PRINT();
}

} // namespace Piston