#include "common.h"
#include "points_list.h"
#include "logging.h"

#include <pxr/base/vt/value.h>
#include <pxr/base/vt/dictionary.h>

#include <stdio.h>
#include <stdint.h>

	
namespace Piston {

template <typename T, typename O>
TemplatedPointsList<T, O>::TemplatedPointsList(): mPointsForeignDataSource(&TemplatedPointsList<T, O>::OnArrayDetached), mOrientationsForeignDataSource(&TemplatedPointsList<T, O>::OnArrayDetached) {
	static_assert(std::is_same_v<T, std::vector<PointType>> || std::is_same_v<T, pxr::VtArray<PointType>>, "Only std::vector<PointType> and pxr::VtArray<PointType> types are permitted!");
	static_assert(std::is_same_v<O, std::vector<OrientationType>> || std::is_same_v<O, pxr::VtArray<OrientationType>>, "Only std::vector<OrientationType> and pxr::VtArray<OrientationType> types are permitted!");
}

template <typename T, typename O>
TemplatedPointsList<T, O>::TemplatedPointsList(size_t size, bool with_orientations): TemplatedPointsList() {
	assert(size > 0);

	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		mPoints.resize(size);
		mPointsVtArray = pxr::VtArray<PointType>(&mPointsForeignDataSource, mPoints.data(), mPoints.size(), true /* addRef */);
	} else {
		mPointsVtArray.resize(size);
	}

	if(with_orientations) {
		if constexpr (std::is_same_v<O, std::vector<OrientationType>>) {
			mOrientations.resize(mPoints.size());
			mOrientationsVtArray  = pxr::VtArray<PointType>(&mOrientationsForeignDataSource, mPoints.data(), mPoints.size(), true /* addRef */);
		} else {
			mOrientationsVtArray.resize(mPointsVtArray.size());
		}
	}

	calcSizeInBytes();
}

template <typename T, typename O>
TemplatedPointsList<T, O>::TemplatedPointsList(Piston::TemplatedPointsList<T, O>&& other): TemplatedPointsList() {
	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		mPoints = std::move(other.mPoints);
		mPointsVtArray = pxr::VtArray<PointType>(&mPointsForeignDataSource, mPoints.data(), mPoints.size(), true /* addRef */);
		other.mPointsVtArray = pxr::VtArray<PointType>(); 
	} else {
		mPointsVtArray = std::move(other.mPointsVtArray);
	}

	if constexpr (std::is_same_v<O, std::vector<OrientationType>>) {
		mOrientations = std::move(other.mOrientations);
		mOrientationsVtArray = pxr::VtArray<PointType>(&mOrientationsForeignDataSource, mPoints.data(), mPoints.size(), true /* addRef */);
	} else {
		mOrientationsVtArray = std::move(other.mOrientationsVtArray);
	}

	mSizeInBytes = other.mSizeInBytes;
}

template <typename T, typename O>
size_t TemplatedPointsList<T, O>::size() const { 
	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		assert(mPoints.size() == mPointsVtArray.AsConst().size());
		return mPoints.size();
	} else {
		return mPointsVtArray.AsConst().size();
	}
}

template <typename T, typename O>
void TemplatedPointsList<T, O>::resize(size_t new_size) {
	if(size() == new_size) return;

	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		mPoints.resize(new_size);
		mPointsVtArray = pxr::VtArray<PointType>(&mPointsForeignDataSource, mPoints.data(), mPoints.size(), true /* addRef */);
	} else {
		mPointsVtArray.resize(new_size);
	}
	calcSizeInBytes();
}

template <typename T, typename O>
TemplatedPointsListBase::PointType* TemplatedPointsList<T, O>::points() {
	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		return mPoints.data();
	} else {
		return mPointsVtArray.data();
	}
}

template <typename T, typename O>
const TemplatedPointsListBase::PointType* TemplatedPointsList<T, O>::points() const { 
	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		return mPoints.data();
	} else {
		return mPointsVtArray.AsConst().data();
	}
}

template <typename T, typename O>
TemplatedPointsListBase::OrientationType* TemplatedPointsList<T, O>::orientations() {
	if constexpr (std::is_same_v<O, std::vector<OrientationType>>) {
		return mOrientations.data();
	} else {
		return mOrientationsVtArray.data();
	}
}

template <typename T, typename O>
const TemplatedPointsListBase::OrientationType* TemplatedPointsList<T, O>::orientations() const { 
	if constexpr (std::is_same_v<O, std::vector<OrientationType>>) {
		return mOrientations.data();
	} else {
		return mOrientationsVtArray.AsConst().data();
	}
}

template <typename T, typename O>
void TemplatedPointsList<T, O>::fillWithZero() {
	static const PointType p_zero = {0.0, 0.0, 0.0};
	static const OrientationType o_zero = pxr::GfQuath(0.0);

	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		std::fill(mPoints.begin(), mPoints.end(), p_zero);
	} else {
		std::fill(mPointsVtArray.begin(), mPointsVtArray.end(), p_zero);
	}

	if constexpr (std::is_same_v<O, std::vector<OrientationType>>) {
		std::fill(mOrientations.begin(), mOrientations.end(), o_zero);
	} else {
		std::fill(mOrientationsVtArray.begin(), mOrientationsVtArray.end(), o_zero);
	}
}

template <typename T, typename O>
void TemplatedPointsList<T, O>::calcSizeInBytes() const {
	mSizeInBytes = 0;

	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		mSizeInBytes += mPoints.size() * sizeof(PointType);
	} else {
		mSizeInBytes += mPointsVtArray.size() * sizeof(PointType);
	}

	if constexpr (std::is_same_v<O, std::vector<OrientationType>>) {
		mSizeInBytes += mOrientations.size() * sizeof(OrientationType);
	} else {
		mSizeInBytes += mOrientationsVtArray.size() * sizeof(OrientationType);
	}
}

template class Piston::TemplatedPointsList<pxr::VtArray<Piston::TemplatedPointsListBase::PointType>, pxr::VtArray<Piston::TemplatedPointsListBase::OrientationType>>;
template class Piston::TemplatedPointsList<std::vector<Piston::TemplatedPointsListBase::PointType>,pxr::VtArray<Piston::TemplatedPointsListBase::OrientationType>>;

} // namespace Piston
