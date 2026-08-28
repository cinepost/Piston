#ifndef PISTON_LIB_INSTANCE_LIST_H_
#define PISTON_LIB_INSTANCE_LIST_H_

#include "framework.h"
#include "points_list.h"

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

template <typename T>
class TemplatedInstanceList : public TemplatedPointsListBase {
	public:
		TemplatedInstanceList(size_t size);
		TemplatedInstanceList(Piston::TemplatedInstanceList<T>&& other);

		size_t size() const;

		PointType& operator [](size_t idx);
		const PointType& operator [](size_t idx) const;

		PointType* data();
		const PointType* data() const;

		const pxr::VtArray<PointType>& getVtArray() const { return mVtArray.AsConst(); }

		void resize(size_t new_size);

		void fillWithZero();

		size_t sizeInBytes() const { return mSizeInBytes; }

	private:
		TemplatedInstanceList();

		static void OnArrayDetached(pxr::Vt_ArrayForeignDataSource* source) {
			// Callback triggered by USD if the internal reference drops to 0
			// Often left empty if the wrapping class object fully dictates the std::vector lifecycle
		}

		void calcSizeInBytes() const;


		std::vector<PointType> 	mPoints;
		pxr::VtArray<PointType> mVtArray;
		pxr::Vt_ArrayForeignDataSource 	mForeignDataSource;

		mutable size_t mSizeInBytes;
};

using InstanceList = TemplatedInstanceList<std::vector<TemplatedPointsListBase::PointType>>;

} // namespace Piston

#endif // PISTON_LIB_INSTANCE_LIST_H_