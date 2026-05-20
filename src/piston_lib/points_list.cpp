#include "common.h"
#include "points_list.h"
#include "logging.h"

#include <pxr/base/vt/value.h>
#include <pxr/base/vt/dictionary.h>

#include <stdio.h>
#include <stdint.h>

	
namespace Piston {

template <typename T>
TemplatedPointsList<T>::TemplatedPointsList(): mForeignDataSource(&TemplatedPointsList<T>::OnArrayDetached) {
	static_assert(std::is_same_v<T, std::vector<PointType>> || std::is_same_v<T, pxr::VtArray<PointType>>, "Only std::vector<PointType> and pxr::VtArray<PointType> types are permitted!");
}

template <typename T>
TemplatedPointsList<T>::TemplatedPointsList(size_t size): TemplatedPointsList() {
	assert(size > 0);

	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		mPoints.resize(size);
		mVtArray = pxr::VtArray<PointType>(&mForeignDataSource, mPoints.data(), mPoints.size(), true /* addRef */);
	} else {
		mVtArray.resize(size);
	}

	calcSizeInBytes();
}

template <typename T>
TemplatedPointsList<T>::TemplatedPointsList(Piston::TemplatedPointsList<T>&& other): TemplatedPointsList() {
	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		mPoints = std::move(other.mPoints);
		mVtArray = pxr::VtArray<PointType>(&mForeignDataSource, mPoints.data(), mPoints.size(), true /* addRef */);
		other.mVtArray = pxr::VtArray<PointType>(); 
	} else {
		mVtArray = std::move(other.mVtArray);
	}
	mSizeInBytes = other.mSizeInBytes;
}

template <typename T>
size_t TemplatedPointsList<T>::size() const { 
	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		assert(mPoints.size() == mVtArray.AsConst().size());
		return mPoints.size();
	} else {
		return mVtArray.AsConst().size();
	}

return mPoints.size(); }

template <typename T>
void TemplatedPointsList<T>::resize(size_t new_size) {
	if(size() == new_size) return;

	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		mPoints.resize(new_size);
		mVtArray = pxr::VtArray<PointType>(&mForeignDataSource, mPoints.data(), mPoints.size(), true /* addRef */);
	} else {
		mVtArray.resize(new_size);
	}
	calcSizeInBytes();
}

template <typename T>
TemplatedPointsListBase::PointType& TemplatedPointsList<T>::operator [](size_t idx) { 
	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		assert(idx < mPoints.size());
		return mPoints[idx]; 
	} else {
		assert(idx < mVtArray.size());
		return mVtArray[idx];
	}
}

template <typename T>
const TemplatedPointsListBase::PointType& TemplatedPointsList<T>::operator [](size_t idx) const { 
	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		assert(idx < mPoints.size());
		return mPoints[idx]; 
	} else {
		assert(idx < mVtArray.size());
		return mVtArray.AsConst()[idx];
	}
}

template <typename T>
TemplatedPointsListBase::PointType* TemplatedPointsList<T>::data() {
	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		return mPoints.data();
	} else {
		return mVtArray.data();
	}
}

template <typename T>
const TemplatedPointsListBase::PointType* TemplatedPointsList<T>::data() const { 
	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		return mPoints.data();
	} else {
		return mVtArray.AsConst().data();
	}
}

template <typename T>
void TemplatedPointsList<T>::fillWithZero() {
	static const PointType zero = {0.0, 0.0, 0.0};

	if constexpr (std::is_same_v<T, std::vector<PointType>>) {
		std::fill(mPoints.begin(), mPoints.end(), zero);
	} else {
		std::fill(mVtArray.begin(), mVtArray.end(), zero);
	}

}

template <typename T>
void TemplatedPointsList<T>::calcSizeInBytes() const {
	mSizeInBytes = size() * sizeof(PointType);
}

template class Piston::TemplatedPointsList<pxr::VtArray<Piston::TemplatedPointsListBase::PointType>>;
template class Piston::TemplatedPointsList<std::vector<Piston::TemplatedPointsListBase::PointType>>;

} // namespace Piston
