#include "simple_profiler.h"
#include "guide_curves_deformer.h"

#include "common.h"
#include "kdtree.hpp"
#include "geometry_tools.h"

#include <pxr/base/gf/matrix4f.h>

#include <random>
#include <algorithm>
#include <mutex>


namespace Piston {

GuideCurvesDeformer::GuideCurvesDeformer(const std::string& name): BaseCurvesDeformer(BaseCurvesDeformer::Type::GUIDES, name) {
	dbg_printf("GuideCurvesDeformer::GuideCurvesDeformer(%s)\n", name.c_str());
	mpGuideCurvesDeformerData = std::make_unique<GuideCurvesDeformerData>();
	mpGuideCurvesContainer = GuideCurvesContainer::create();
}

GuideCurvesDeformer::SharedPtr GuideCurvesDeformer::create(const std::string& name) {
	return SharedPtr(new GuideCurvesDeformer(name));
}

bool GuideCurvesDeformer::validateDeformerGeoPrim(const pxr::UsdPrim& geoPrim) {
	return isBasisCurvesGeoPrim(geoPrim);
}

const std::string& GuideCurvesDeformer::toString() const {
	static const std::string kFastDeformerString = "GuideCurvesDeformer";
	return kFastDeformerString;
}

void GuideCurvesDeformer::setGuideIDPrimAttrName(const std::string& name) {
	if(mGuideIDPrimAttrName == name) return;
	mGuideIDPrimAttrName = name;
	makeDirty();
}

bool GuideCurvesDeformer::deformImpl(PointsList& points, pxr::UsdTimeCode time_code) {
	return __deform__(points, false, time_code);
}

bool GuideCurvesDeformer::deformMtImpl(PointsList& points, pxr::UsdTimeCode time_code) {
	return __deform__(points, true, time_code);
}

bool GuideCurvesDeformer::__deform__(PointsList& points, bool multi_threaded, pxr::UsdTimeCode time_code) {
	if(!mpGuideCurvesContainer->update(mDeformerGeoPrimHandle, time_code)) {
		std::cerr << "Error updating guide curves from prim" << mDeformerGeoPrimHandle.getPath().GetText() << " !" << std::endl;
		return false;
	}

	return true;
}

bool GuideCurvesDeformer::buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	assert(mpGuideCurvesDeformerData);
	assert(mpGuideCurvesContainer);

	if(!mpGuideCurvesContainer->init(mDeformerGeoPrimHandle, rest_time_code)) {
		std::cerr << "Error initializing guide curves container !" << std::endl;
		return false;
	}

	const auto total_guides_count = mpGuideCurvesContainer->getCurvesCount();
	const auto total_curves_count = mpCurvesContainer->getCurvesCount();

	if(mGuideIDPrimAttrName.empty() || getBindMode() == BindMode::SPACE) {
		// UNIMPLEMENTED !!!
		std::cerr << "SPACE points binding method is unimplemented yet !" << std::endl;
		return false;
	} 

	pxr::UsdGeomPrimvarsAPI curvesPrimvarsApi = mCurvesGeoPrimHandle.getPrimvarsAPI();
	pxr::UsdGeomPrimvar guideIDSPrimVar = curvesPrimvarsApi.GetPrimvar(pxr::TfToken(mGuideIDPrimAttrName));

	if(!guideIDSPrimVar) {
		std::cerr << "Error getting guide(clump) id prmitive attribute \"" << mGuideIDPrimAttrName << "\" !" << std::endl;
		return false;
	}

	if(guideIDSPrimVar.GetAttr().Get(&mGuideIndices, rest_time_code)) {
		std::cerr << "Error getting curves " << mCurvesGeoPrimHandle.getPath() << " \"" << mGuideIDPrimAttrName << "\" guide indices !" << std::endl;
		return false;
	}
	assert(mGuideIndices.size() == total_curves_count);

	// check guide indices are not out of range
	int max_guide_index = 0;
	for(const int idx: mGuideIndices) {
		max_guide_index = std::max(max_guide_index, idx);
	}

	if(max_guide_index >= total_guides_count) {
		std::cerr << "Curves " << mCurvesGeoPrimHandle.getPath() << " guide indices are out of range !" << std::endl; 
		return false;
	}

	return buildDeformerDataDistMode(rest_time_code, multi_threaded);
}

bool GuideCurvesDeformer::buildDeformerDataDistMode(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {

	const size_t guides_count = mpGuideCurvesContainer->getCurvesCount();

	std::vector<std::mutex> kdtrees_mutexes(guides_count);  // protects kdree initialisation
	std::vector<std::unique_ptr<neighbour_search::KDTree<float, 3>>> kdtrees(guides_count);

	const size_t curves_count = mpCurvesContainer->getCurvesCount();

	auto& pointBinds = mpGuideCurvesDeformerData->pointBinds();

	auto func = [&](const size_t curve_index) {
    	const int guide_id = mGuideIndices[curve_index];
    	assert(guide_id < guides_count);
        const neighbour_search::KDTree<float, 3>* pKDTree;
        
        {
        	// build guide kdtree if needed
        	const std::lock_guard<std::mutex> lock(kdtrees_mutexes[guide_id]);
        	if(!kdtrees[guide_id]) {
        		kdtrees[guide_id] = std::make_unique<neighbour_search::KDTree<float, 3>>(mpGuideCurvesContainer->getRestCurvePoints(), false /*threaded*/);
        	}
        	pKDTree = kdtrees[guide_id].get();
        }

    };

	if(multi_threaded) {
        BS::multi_future<void> loop = mPool.submit_loop(0, curves_count, func);
        loop.wait();
    } else {
        for(size_t i = 0; i < curves_count; ++i) {
            func(i);
        }
    }
}

bool GuideCurvesDeformer::writeJsonDataToPrimImpl() const {
	if(!mCurvesGeoPrimHandle.writeDataToBson(mpGuideCurvesDeformerData.get())) {
		std::cerr << "Error writing " << mpGuideCurvesDeformerData->typeName() << " deformer data to json !";	
		return false;
	}
	return true;
}

void GuideCurvesDeformer::setBindMode(GuideCurvesDeformer::BindMode mode) {
	assert(mpGuideCurvesDeformerData);
	if(mpGuideCurvesDeformerData->getBindMode() == mode) return;

	mpGuideCurvesDeformerData->setBindMode(mode);
	makeDirty();
}

const GuideCurvesDeformer::BindMode& GuideCurvesDeformer::getBindMode() const {
	assert(mpGuideCurvesDeformerData);
	return mpGuideCurvesDeformerData->getBindMode();
}


GuideCurvesDeformer::~GuideCurvesDeformer() {
	PROFILE_PRINT();
}


} // namespace Piston