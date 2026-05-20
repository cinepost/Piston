#ifndef PISTON_LIB_POINTS_LIST_H_
#define PISTON_LIB_POINTS_LIST_H_

#include "framework.h"

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/imaging/hd/meshTopology.h>
#include <pxr/imaging/hd/basisCurvesTopology.h>

#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <type_traits>


namespace Piston {

struct TemplatedPointsListBase {
    using PointType = pxr::GfVec3f;
};

template <typename T>
class TemplatedPointsList : public TemplatedPointsListBase {
	public:
		TemplatedPointsList(size_t size);
		TemplatedPointsList(Piston::TemplatedPointsList<T>&& other);

		size_t size() const;

		PointType& operator [](size_t idx);
		const PointType& operator [](size_t idx) const;

		PointType* data();
		const PointType* data() const;

		//std::vector<pxr::GfVec3f>& getVector() { return mPoints; }
		//const std::vector<pxr::GfVec3f>& getVector() const { return mPoints; }

		const pxr::VtArray<PointType>& getVtArray() const { return mVtArray; }

		void resize(size_t new_size);

		void fillWithZero();

		size_t sizeInBytes() const { return mSizeInBytes; }

	private:
		TemplatedPointsList();

		static void OnArrayDetached(pxr::Vt_ArrayForeignDataSource* source) {
			// Callback triggered by USD if the internal reference drops to 0
			// Often left empty if the wrapping class object fully dictates the std::vector lifecycle
		}

		void calcSizeInBytes() const;


		std::vector<PointType> 	mPoints;
		pxr::VtArray<PointType> 	mVtArray;
		pxr::Vt_ArrayForeignDataSource 	mForeignDataSource;

		mutable size_t mSizeInBytes;
};

using PointsList = TemplatedPointsList<std::vector<TemplatedPointsListBase::PointType>>;

} // namespace Piston

#endif // PISTON_LIB_POINTS_LIST_H_