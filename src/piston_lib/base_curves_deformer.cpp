#include "deformer_factory.h"
#include "base_curves_deformer.h"
#include "geometry_tools.h"
#include "pxr_points_lru_cache.h"
#include "topology.h"
#include "logging.h"

#include <thread>
#include <atomic>

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

bool BaseCurvesDeformer::buildDeformerData(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	if(!mDirty) return true;

	SimpleProfiler::clear();

	if(!mDeformerGeoPrimHandle) {
		DLOG_ERR << "No deformer UsdPrim is set !!!";
		return false;
	}

	if(!mCurvesGeoPrimHandle) {
		DLOG_ERR << "No curves UsdPrim is set !!!";
		return false;
	}

	if(!mpCurvesContainer) {
		mpCurvesContainer = PxrCurvesContainer::create(mCurvesGeoPrimHandle, getCurvesRestAttrName(), rest_time_code);
		if(!mpCurvesContainer) {
			DLOG_ERR << "Error creating curves container for prim " << mCurvesGeoPrimHandle << " !";
			return false;
		}
	} else {
		if(!mpCurvesContainer->init(mCurvesGeoPrimHandle, getCurvesRestAttrName(), rest_time_code)) {
			DLOG_ERR << "Error initializing curves container for prim " << mCurvesGeoPrimHandle << " !";
			return false;
		}
	}

	if(!mpDeformerMeshContainer) {
		mpDeformerMeshContainer = MeshContainer::create(mDeformerGeoPrimHandle, rest_time_code);
		if(!mpDeformerMeshContainer) {
			DLOG_ERR << "Error creating deformer mesh container for prim " << mDeformerGeoPrimHandle << " !";
			return false;
		}
	} else {
		if(!mpDeformerMeshContainer->init(mDeformerGeoPrimHandle, rest_time_code)) {
			DLOG_ERR << "Error initializing curves container for prim " << mDeformerGeoPrimHandle << " !";
			return false;
		}
	}


	{
		const std::string entry_name_str = toString() + ":" + getName() + ":buildDeformerData";
		PROFILE(entry_name_str.c_str());

		if(!buildDeformerDataImpl(rest_time_code, multi_threaded)) {
			DLOG_ERR << "Error building " << mName <<" deformer data !";
			return false;
		}

	}

	mDirty = false;
	return true;
}

bool BaseCurvesDeformer::deform_dbg(pxr::UsdTimeCode time_code, bool ignoreVelocities) {	
	return deform(time_code, false, ignoreVelocities);
}

bool BaseCurvesDeformer::deform(pxr::UsdTimeCode time_code, bool multi_threaded, bool ignoreVelocities) {
	DLOG_TRC << "Deform at time code: " << time_code.GetValue();
		
	const pxr::UsdTimeCode bind_time_code = getRestTimeCode();
	if(!buildDeformerData(bind_time_code, multi_threaded)) {
		return false;
	}

	assert(mpDeformerMeshContainer);
	if(!mpDeformerMeshContainer || mpDeformerMeshContainer->getRestPositions().empty()) {
		return false;
	}

	assert(mpCurvesContainer);	
	if(!mpCurvesContainer || mpCurvesContainer->empty()) {
		return false;
	}

	DLOG_TRC << "Curves in " << to_string(mpCurvesContainer->getSpace()) << " space";

	auto deformPoints = [this](bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code) {
		if(!mpDeformerMeshContainer->update(mDeformerGeoPrimHandle, time_code, isDirty())) {
			return false;
		}

		if(mpCurvesContainer && !mpCurvesContainer->update(mCurvesGeoPrimHandle, time_code, isDirty())) {
			return false;
		}
		
		if(multi_threaded) { return deformMtImpl(points, time_code); }

		return deformImpl(points, time_code);
	};

	auto getTempVelocitiesList = [this](size_t list_size) {
		if(!mpTempVelocitiesList) {
			mpTempVelocitiesList = std::make_unique<PointsList>(list_size);
		} else {
			mpTempVelocitiesList->resize(list_size);
		}

		return (PointsList*)mpTempVelocitiesList.get();
	};


	auto getDeformedPoints = [this, &deformPoints](std::unique_ptr<PointsList>& points, bool multi_threaded, PxrCurvesContainer* pCurves, const PxrPointsLRUCache::CompositeKey& key) {
		assert(pCurves);
		DLOG_TRC << "Deforming curves at " << key.time;

		const size_t points_count = pCurves->getTotalVertexCount();

		if(!points) {
			points = std::make_unique<PointsList>(points_count);
		} else {
			points->resize(points_count);
		}

		PointsList* points_list = points.get(); 

		if (deformPoints(multi_threaded, *points_list, key.time)) {
			return (const PointsList*)points_list;
		}

		return (const PointsList*)nullptr;
	};

	auto getDeformedPointsLRU = [this, &deformPoints](bool multi_threaded, PxrCurvesContainer* pCurves, PxrPointsLRUCache* pPointsLRUCache, const PxrPointsLRUCache::CompositeKey& key) {
		assert(pCurves);
		assert(pPointsLRUCache);
		DLOG_TRC << "Deforming curves (using cache) at " << key.time;

		static const PointsList* sNull = nullptr;

		const PointsList* p_points_list_ptr = pPointsLRUCache->get(key);
		if(p_points_list_ptr) {
			DLOG_TRC << "Cache has entry key " << to_string(key);
			return p_points_list_ptr;
		}

		PointsList* p_new_points_list = pPointsLRUCache->put(key, pCurves->getTotalVertexCount());
		if (deformPoints(multi_threaded, *p_new_points_list, key.time)) {
			return (const PointsList*)p_new_points_list;
		}

		return sNull;
	};

	const PxrPointsLRUCache::CompositeKey curr_key = {uniqueName(), time_code};
	PxrPointsLRUCache* pPointsLRUCache = mUsePointsCache ? CurvesDeformerFactory::getInstance().getPxrPointsLRUCachePtr() : nullptr;

	const PxrPointsLRUCache::CompositeKey key_from = {uniqueName(), (motionBlurDirection() != MotionBlurDirection::LEADING) ? pxr::UsdTimeCode(time_code.GetValue() - 1.0) : time_code};
	const PxrPointsLRUCache::CompositeKey key_to = {uniqueName(), (motionBlurDirection() != MotionBlurDirection::TRAILING) ? pxr::UsdTimeCode(time_code.GetValue() + 1.0) : time_code};

	PxrPointsLRUCacheShrinkLock cache_shrink_lock(pPointsLRUCache); // avoid cache shrinking during deformation stage
	if(cache_shrink_lock.isValid()) {
		DLOG_TRC << "pPointsLRUCache locked";
	}

	DLOG_TRC << "Velocities calculation is possible " << (mDeformerGeoPrimHandle.hasPositionsTimeSamples(key_from.time, key_to.time) ? "YES" : "NO");
	DLOG_TRC << "Velocities calculation is ignored " << (ignoreVelocities ? "YES" : "NO");

	const PointsList* pPointsVBlurFrom = nullptr;
	const PointsList* pPointsVBlurTo = nullptr;

	const PxrPointsLRUCache::CompositeKey velocity_key = {velocityKeyName(), time_code};
	const PointsList* veolcities_list_ptr = pPointsLRUCache ? pPointsLRUCache->get(velocity_key) : nullptr;
	bool output_motion_vectors = false;

	if(!ignoreVelocities && calcMotionVectors() && mDeformerGeoPrimHandle.hasPositionsTimeSamples(key_from.time, key_to.time)) {
		if(!veolcities_list_ptr) {
			pPointsVBlurFrom = (motionBlurDirection() == MotionBlurDirection::LEADING) ? nullptr :
				(pPointsLRUCache ? getDeformedPointsLRU(multi_threaded, mpCurvesContainer.get(), pPointsLRUCache, key_from) : getDeformedPoints(mpDeformedPointsListStep, multi_threaded, mpCurvesContainer.get(), key_from));
			
			pPointsVBlurTo = (motionBlurDirection() == MotionBlurDirection::TRAILING) ? nullptr : 
				(pPointsLRUCache ? getDeformedPointsLRU(multi_threaded, mpCurvesContainer.get(), pPointsLRUCache, key_to) : getDeformedPoints(mpDeformedPointsListStep, multi_threaded, mpCurvesContainer.get(), key_to));
		}

		output_motion_vectors = true;
	}

	const PointsList* deformed_points_list_ptr = pPointsLRUCache ? getDeformedPointsLRU(multi_threaded, mpCurvesContainer.get(), pPointsLRUCache, curr_key) : getDeformedPoints(mpDeformedPointsList, multi_threaded, mpCurvesContainer.get(), curr_key);
	assert(deformed_points_list_ptr);
	if(!deformed_points_list_ptr) {
		DLOG_ERR << "Error getting deformed points list!";
		return false;
	}

	pxr::UsdGeomCurves curves(mCurvesGeoPrimHandle.getPrim());
	pxr::UsdAttribute attr_v = curves.GetVelocitiesAttr();

	if(output_motion_vectors && attr_v) {

		if(!veolcities_list_ptr) {
			DLOG_TRC << "Calc velocities from " <<  std::to_string(key_from.time.GetValue()) << " to " <<  std::to_string(key_to.time.GetValue());
			
			assert(pPointsVBlurFrom || pPointsVBlurTo);
			const pxr::GfVec3f* p_pts_from_ptr = pPointsVBlurFrom ?  pPointsVBlurFrom->points() : deformed_points_list_ptr->points();
			const pxr::GfVec3f* p_pts_to_ptr = pPointsVBlurTo ? pPointsVBlurTo->points() : deformed_points_list_ptr->points();

			assert(p_pts_from_ptr != p_pts_to_ptr);

			const float k = ((motionBlurDirection() == MotionBlurDirection::CENTERED) ? .5f : 1.0f) * static_cast<float>(mDeformerGeoPrimHandle.getStageTimeCodesPerSecond());

			PointsList* tmp_velicities_list_ptr = pPointsLRUCache ? pPointsLRUCache->put(velocity_key, mpCurvesContainer->getTotalVertexCount()) : getTempVelocitiesList(mpCurvesContainer->getTotalVertexCount());
			assert(tmp_velicities_list_ptr);

			auto calcVectorsFunc = [&](const std::size_t start, const std::size_t end) {
				auto velocities = tmp_velicities_list_ptr->points();
				if(p_pts_from_ptr == p_pts_to_ptr) {

					for(size_t i = start; i < end; ++i) {
						velocities[i] = {0.0, 0.0, 0.0};
					}
				} else {
					for(size_t i = start; i < end; ++i) {
						velocities[i] = (p_pts_to_ptr[i] - p_pts_from_ptr[i]) * k;
					}
				}
			};

			if(multi_threaded) {
				BS::multi_future<void> blocks = mPool.submit_blocks(0u, tmp_velicities_list_ptr->size(), calcVectorsFunc);
				blocks.wait();
			} else {
				calcVectorsFunc(0u, tmp_velicities_list_ptr->size());
			}

			veolcities_list_ptr = (const PointsList*)tmp_velicities_list_ptr;
			
			assert(veolcities_list_ptr);
		}

		if(!attr_v.Set(veolcities_list_ptr->getPointsVtArray(), time_code)) {
			DLOG_ERR << "Error setting velocities attribute !";
			return false;
		}
	}

	if(!curves.GetPointsAttr().Set(deformed_points_list_ptr->getPointsVtArray(), time_code)) {
		DLOG_ERR << "Error setting deformerd points to " << mCurvesGeoPrimHandle << " !";
		return false;
	}

	if(pPointsLRUCache) {
		const std::string current_usage_str = pPointsLRUCache->getCacheUtilizationString();
		if(gLRUCacheStatsLastUsageStr != current_usage_str) {
			DLOG_DBG << "Points cache utilization: " << current_usage_str << "%";
			gLRUCacheStatsLastUsageStr = current_usage_str;
		}
	}

	if(mShowDebugGeometry) {
		drawDebugSubdivDeformerGeometry(time_code);
		drawDebugGeometry(time_code, deformed_points_list_ptr);
	}

	return true;
}

const std::string& BaseCurvesDeformer::toString() const {
	static const std::string kBaseDeformerString = "BaseCurvesDeformer";
	return kBaseDeformerString;
}

} // namespace Piston
