#include "simple_profiler.h"
#include "guide_curves_deformer.h"

#include "common.h"
#include "kdtree.hpp"
#include "geometry_tools.h"

#include <pxr/base/gf/matrix4f.h>

#include <random>
#include <algorithm>


namespace Piston {

GuidesCurvesDeformer::GuidesCurvesDeformer(const std::string& name): BaseCurvesDeformer(BaseCurvesDeformer::Type::FAST, name) {
	dbg_printf("GuidesCurvesDeformer::GuidesCurvesDeformer(%s)\n", name.c_str());
	mpFastCurvesDeformerData = std::make_unique<FastCurvesDeformerData>();
}

GuidesCurvesDeformer::SharedPtr GuidesCurvesDeformer::create(const std::string& name) {
	return SharedPtr(new GuidesCurvesDeformer(name));
}

const std::string& GuidesCurvesDeformer::toString() const {
	static const std::string kFastDeformerString = "GuidesCurvesDeformer";
	return kFastDeformerString;
}

bool GuidesCurvesDeformer::deformImpl(PointsList& points, pxr::UsdTimeCode time_code) {
	return __deform__(points, false, time_code);
}

bool GuidesCurvesDeformer::deformMtImpl(PointsList& points, pxr::UsdTimeCode time_code) {
	return __deform__(points, true, time_code);
}

bool GuidesCurvesDeformer::validateDeformerGeoPrim(const pxr::UsdPrim& geoPrim) {
	return isBasisCurvesGeoPrim(geoPrim);
}

GuidesCurvesDeformer::~GuidesCurvesDeformer() {
	PROFILE_PRINT();
}

} // namespace Piston