#include "simple_profiler.h"
#include "point_instancer_deformer.h"

#include "common.h"
#include "logging.h"
#include "kdtree.hpp"
#include "geometry_tools.h"

#include <pxr/base/gf/matrix4f.h>

#include <atomic>
#include <random>
#include <algorithm>
#include <optional>


namespace Piston {

static constexpr float kEpsilon = std::numeric_limits<float>::epsilon();
static constexpr float kMaxFloat = std::numeric_limits<float>::max();

PointInstancerDeformer::PointInstancerDeformer(const std::string& name): BaseDeformer(BaseDeformer::Type::POINT_INSTANCER, name), mBindMode(BindMode::SIMPLE) {

}

PointInstancerDeformer::SharedPtr PointInstancerDeformer::create(const std::string& name) {
	return SharedPtr(new PointInstancerDeformer(name));
}

const std::string& PointInstancerDeformer::toString() const {
	static const std::string kFastDeformerString = "PointInstancerDeformer";
	return kFastDeformerString;
}

bool PointInstancerDeformer::validateDeformerGeoPrim(const pxr::UsdPrim& geoPrim) {
	return isMeshGeoPrim(geoPrim);
}

void PointInstancerDeformer::setInstancerGeoPrim(const pxr::UsdPrim& prim) {
	if(!prim.IsValid() || mInstancerGeoPrimHandle == prim) return;

	if(!isPointInstancerGeoPrim(prim)) {
		DLOG_ERR << "Instancer geometry prim is not \"pxr::UsdGeomPointInstancer\"!";
		return;
	}

	if(mDeformerGeoPrimHandle == prim) {
		DLOG_ERR << "Can't use the same prim " << mDeformerGeoPrimHandle << " for deformer and Instancer geometry !!!";
		return;
	}

	auto new_handle = UsdPrimHandle(prim);
	const bool same_topology = mInstancerGeoPrimHandle.isValid() ? isSameTopology(mInstancerGeoPrimHandle, new_handle, getRestTimeCode()) : false;

	mInstancerGeoPrimHandle = std::move(new_handle);
	if(!same_topology) {
		makeDirty();
	}

	DLOG_DBG << "Instancer geometry prim is set to: " << mInstancerGeoPrimHandle;
}

const pxr::UsdPrim& PointInstancerDeformer::getInstancerGeoPrim() const {
	return mInstancerGeoPrimHandle.getPrim();
}

void PointInstancerDeformer::setInstancerRestAttrName(const std::string& name) {
	if(mInstancerGeoPrimHandle.getRestAttrName() == name) return;
	mInstancerGeoPrimHandle.setRestAttrName(name);
	makeDirty();

	DLOG_DBG << "Deformer instancer rest attribute name is set to: " <<  name;
}

void PointInstancerDeformer::invalidateData(DeformerDataCache& cache) {
	cache.invalidate(mpAdjacencyData);
	cache.invalidate(mpPhantomTrimeshData);
	cache.invalidate(mpPointInstancerDeformerData);
}

size_t PointInstancerDeformer::getDeformedPointsCount() const {
	LOG_DBG << "PointInstancerDeformer::getDeformedPointsCount";
	assert(mpInstancerContainer);

	LOG_DBG << "PointInstancerDeformer::getDeformedPointsCount" << mpInstancerContainer->getInstanceCount();
	return mpInstancerContainer->getInstanceCount();
}

bool PointInstancerDeformer::outputDeformedPoints(const PointsList* pPointsList, pxr::UsdTimeCode time_code) {
	assert(pPointsList);

	pxr::UsdGeomPointInstancer instancer(mInstancerGeoPrimHandle.getPrim());
	pxr::UsdAttribute attr_p = instancer.GetPositionsAttr();

	if(!attr_p || !attr_p.Set(pPointsList->getPointsVtArray(), time_code)) {
		return false;
	}

	return true;
}

bool PointInstancerDeformer::outputVelocites(const PointsList* pVelocitiesList, pxr::UsdTimeCode time_code) {
	assert(pVelocitiesList);

	pxr::UsdGeomPointInstancer instancer(mInstancerGeoPrimHandle.getPrim());
	pxr::UsdAttribute attr_v = instancer.GetVelocitiesAttr();

	if(!attr_v || !attr_v.Set(pVelocitiesList->getPointsVtArray(), time_code)) {
		return false;	
	}

	return true;
}

bool PointInstancerDeformer::writeJsonDataToPrimImpl() const {
	LOG_DBG << "PointInstancerDeformer::buildDeformerDataImpl";

	if(mpAdjacencyData && !mDeformerGeoPrimHandle.writeDataToBson(getDataPrimPath(), mpAdjacencyData.get())) {
		DLOG_ERR << "Error writing " << mpAdjacencyData->typeName() << " deformer data to json !";
		return false;
	}

	if(mpPhantomTrimeshData && !mInstancerGeoPrimHandle.writeDataToBson(getDataPrimPath(), mpPhantomTrimeshData.get())) {
		DLOG_ERR << "Error writing " << mpPhantomTrimeshData->typeName() << " curves data to json !";
		return false;
	}

	if(mpPointInstancerDeformerData && !mInstancerGeoPrimHandle.writeDataToBson(getDataPrimPath(), mpPointInstancerDeformerData.get())) {
		DLOG_ERR << "Error writing " << mpPointInstancerDeformerData->typeName() << " deformer data to json !";	
		return false;
	}
	return true;
}

bool PointInstancerDeformer::__deform__(PointsList& points, bool multi_threaded, pxr::UsdTimeCode time_code) {
	DLOG_DBG << "PointInstancerDeformer::__deform__";

	assert(mpInstancerContainer);
	assert(mpPointInstancerDeformerData);

	const auto& rest_positions = mpInstancerContainer->getRestInstancePoints();

	DLOG_DBG << "Points count " << points.size();
	DLOG_DBG << "Instances container instances count " << mpInstancerContainer->getInstanceCount();
	DLOG_DBG << "Instances count " << rest_positions.size();

	assert(points.size() == rest_positions.size());

	auto* pOutPoints = points.points();

	for(size_t i = 0; i < points.size(); ++i) {
		pOutPoints[i] = rest_positions[i] + pxr::GfVec3f(static_cast<float>(time_code.GetValue()), 0.0, 0.0);
	}

	return true;
}

bool PointInstancerDeformer::buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	DLOG_DBG << "PointInstancerDeformer::buildDeformerDataImpl";

	if(!mInstancerGeoPrimHandle) {
		DLOG_ERR << "No instancer UsdPrim is set !!!";
		return false;
	}

	if(!mpInstancerContainer) {
		mpInstancerContainer = InstancerContainer::create();
		if(!mpInstancerContainer) {
			DLOG_ERR << "Error creating instancer container for prim " << mInstancerGeoPrimHandle << " !";
			return false;
		}
	} 
	
	if(!mpInstancerContainer->init(mInstancerGeoPrimHandle, rest_time_code)) {
		DLOG_ERR << "Error initializing instancer container for prim " << mInstancerGeoPrimHandle << " !";
		return false;
	}

	// 
	DeformerDataCache& dataCache = DeformerDataCache::getInstance();

	bool adjacency_data_created = true;
	if(!mpAdjacencyData) {
		mpAdjacencyData = dataCache.getOrCreateData<SerializableUsdGeomMeshFaceAdjacency>(this, mDeformerGeoPrimHandle, rest_time_code, adjacency_data_created);
	}

	// Get primitive adjacency json data if present
	if(adjacency_data_created || !getReadJsonDataState() || !mDeformerGeoPrimHandle.getDataFromBson(getDataPrimPath(), mpAdjacencyData.get())) {
		// Build in place if no json data present or not needed
		if(!mpAdjacencyData->buildInPlace(mDeformerGeoPrimHandle)) {
			DLOG_ERR << "Error building mesh adjacency data!";
			return false;
		}
	}

	if(!mpAdjacencyData || !mpAdjacencyData->getAdjacency() || !mpAdjacencyData->getAdjacency()->isValid()) {
		DLOG_ERR << "No valid mesh adjacency data!";
		return false;
	}

	bool trimesh_data_created = true;
	if(!mpPhantomTrimeshData) {
		mpPhantomTrimeshData = dataCache.getOrCreateData<SerializablePhantomTrimesh>(this, {&mDeformerGeoPrimHandle, &mInstancerGeoPrimHandle}, rest_time_code, trimesh_data_created);
	}

	// Get phantom mesh json data if present
	if(trimesh_data_created || !getReadJsonDataState() || !mInstancerGeoPrimHandle.getDataFromBson(getDataPrimPath(), mpPhantomTrimeshData.get())) {
		// Build in place if no json data present or not needed
		if(!mpPhantomTrimeshData->buildInPlace(mDeformerGeoPrimHandle)) {
			DLOG_ERR << "Error building phantom mesh data!";
			return false;
		}
	}

	assert(mpAdjacencyData);
	const auto* pAdjacency = mpAdjacencyData->getAdjacencyFinal();
	
	assert(mpPhantomTrimeshData);
	auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();

	bool deformer_data_created;
	if(!mpPointInstancerDeformerData) {
		mpPointInstancerDeformerData = dataCache.getOrCreateData<PointInstancerDeformerData>(this, {&mDeformerGeoPrimHandle, &mInstancerGeoPrimHandle}, rest_time_code, deformer_data_created);
		mpPointInstancerDeformerData->setBindMode(mBindMode);
	}

	if(deformer_data_created || !mpPointInstancerDeformerData->isValid() || !mpPhantomTrimeshData->isValid()) {
		if(!getReadJsonDataState() || !mInstancerGeoPrimHandle.getDataFromBson(getDataPrimPath(), mpPointInstancerDeformerData.get())) {
			// Build deformer data in place if no json data present or not needed

			std::vector<pxr::GfVec3f> rest_vertex_normals;
			buildVertexNormals(pAdjacency, pPhantomTrimesh, rest_vertex_normals, mpDeformerMeshContainer->getRestPositions(), (multi_threaded ? &mPool : nullptr));
			mLiveVertexNormals.resize(rest_vertex_normals.size());

			// Bind curve points
			DLOG_DBG << "Binding " << mpInstancerContainer->getInstanceCount() << " instancer points.";	

			bool result = false;
			auto threads_timer = Timer();
			threads_timer.start();

			// Build bind data
			switch(mpPointInstancerDeformerData->getBindMode()) {
				case BindMode::SIMPLE:
				default:
					result = buildDeformerData_SimpleMode(multi_threaded, rest_vertex_normals, rest_time_code);
					break;
			}

			auto& face_flags = pPhantomTrimesh->getFaceFlags(); 
			assert(face_flags.size() == pPhantomTrimesh->getFaceCount());

			for(const auto& bind: mpPointInstancerDeformerData->getPointBinds()) {
				if(bind.isValid()) face_flags[bind.face_id] = PhantomTrimesh::TriFace::Flags::Bound;
			}

			threads_timer.stop();
			if(multi_threaded) {
				DLOG_TRC << "PointInstancerDeformer::buildDeformerDataImpl() " << mPool.get_thread_count() << " threads finished in " << threads_timer.toString();
			} else {
				DLOG_TRC << "PointInstancerDeformer::buildDeformerDataImpl() finished in " << threads_timer.toString();
			}

			mpPointInstancerDeformerData->setValid(result);
		}
		mpPhantomTrimeshData->setValid(mpPointInstancerDeformerData->isValid());
	}

	return mpPointInstancerDeformerData->isValid();
}

bool PointInstancerDeformer::buildDeformerData_SimpleMode(bool multi_threaded, const std::vector<pxr::GfVec3f>& rest_vertex_normals, pxr::UsdTimeCode rest_time_code) {
	return true;
}

void PointInstancerDeformer::drawDebugGeometry(pxr::UsdTimeCode time_code, const PointsList* pDeformedPoints) {

}

PointInstancerDeformer::~PointInstancerDeformer() {
	PROFILE_PRINT();
}

} // namespace Piston