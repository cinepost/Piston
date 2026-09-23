#include "deformer_factory.h"
#include "base_curves_deformer.h"
#include "geometry_tools.h"
#include "pxr_points_lru_cache.h"
#include "topology.h"
#include "logging.h"

#include <thread>
#include <atomic>

#include <pxr/usd/usd/editContext.h>

static std::string gLRUCacheStatsLastUsageStr = "-";

namespace Piston {

BaseCurvesDeformer::BaseCurvesDeformer(const BaseCurvesDeformer::Type t, const std::string& name): BaseDeformer(t, name) {	
	DLOG_TRC << "BaseCurvesDeformer::BaseCurvesDeformer()";
}

void BaseCurvesDeformer::setCurvesGeoPrim(const pxr::UsdPrim& prim) {
	if(!prim.IsValid() || mCurvesGeoPrimHandle == prim) return;

	if(!isBasisCurvesGeoPrim(prim)) {
		DLOG_ERR << "Curves geometry prim is not \"BasisCurves\"!";
		return;
	}

	if(mDeformerGeoPrimHandle == prim) {
		DLOG_ERR << "Can't use the same prim " << mDeformerGeoPrimHandle << " for deformer and curves geometry !!!";
		return;
	}

	auto new_handle = UsdPrimHandle(prim);
	const bool same_topology = mCurvesGeoPrimHandle.isValid() ? isSameTopology(mCurvesGeoPrimHandle, new_handle, getRestTimeCode()) : false;

	mCurvesGeoPrimHandle = std::move(new_handle);
	if(!same_topology) {
		makeDirty();
	}

	DLOG_DBG << "Curves geometry prim is set to: " << mCurvesGeoPrimHandle;
}

const pxr::UsdPrim& BaseCurvesDeformer::getCurvesGeoPrim() const {
	return mCurvesGeoPrimHandle.getPrim();
}

void BaseCurvesDeformer::setCurvesRestAttrName(const std::string& name) {
	if(mCurvesGeoPrimHandle.getRestAttrName() == name) return;
	mCurvesGeoPrimHandle.setRestAttrName(name);
	makeDirty();

	DLOG_DBG << "Curves rest attribute name is set to: " << name;
}

size_t BaseCurvesDeformer::getDeformedPointsCount() const {
	assert(mpCurvesContainer);
	return mpCurvesContainer->getTotalVertexCount();
}

bool BaseCurvesDeformer::outputDeformedPoints(const PointsList* pPointsList, pxr::UsdTimeCode time_code) {
	assert(pPointsList);
	bool result = false;

	pxr::VtArray<pxr::GfVec3f> new_points(pPointsList->getPointsVtArray().size());

	for(size_t i = 0; i < pPointsList->getPointsVtArray().size(); ++i) {
		new_points[i] = pPointsList->getPointsVtArray()[i];
	}

	pxr::UsdGeomCurves curves(mCurvesGeoPrimHandle.getPrim());
	result = curves.GetPointsAttr().Set(new_points, time_code);

	return result;
}

bool BaseCurvesDeformer::buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	DLOG_DBG << "BaseCurvesDeformer::buildDeformerDataImpl";

	if(!mCurvesGeoPrimHandle) {
		DLOG_ERR << "No curves UsdPrim is set !!!";
		return false;
	}

	if(!mpCurvesContainer) {
		DLOG_DBG << "Creating curves container for " << mCurvesGeoPrimHandle;
		mpCurvesContainer = PxrCurvesContainer::create(mCurvesGeoPrimHandle, getCurvesRestAttrName(), rest_time_code);
		if(!mpCurvesContainer) {
			DLOG_ERR << "Error creating curves container for prim " << mCurvesGeoPrimHandle << " !";
			return false;
		}
		DLOG_DBG << mCurvesGeoPrimHandle << "curves container created";
	} else {
		DLOG_DBG << "initializing curves container for " << mCurvesGeoPrimHandle;
		if(!mpCurvesContainer->init(mCurvesGeoPrimHandle, getCurvesRestAttrName(), rest_time_code)) {
			DLOG_ERR << "Error initializing curves container for prim " << mCurvesGeoPrimHandle << " !";
			return false;
		}
		DLOG_DBG << mCurvesGeoPrimHandle << "curves container initilized";
	}

	return true;
}

const std::string& BaseCurvesDeformer::toString() const {
	static const std::string kBaseDeformerString = "BaseCurvesDeformer";
	return kBaseDeformerString;
}

} // namespace Piston
