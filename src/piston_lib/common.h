#ifndef PISTON_LIB_COMMON_H_
#define PISTON_LIB_COMMON_H_

#include "framework.h"

#include <nlohmann/json.hpp>

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>

#include <memory>
#include <string>
#include <sstream>
#include <vector>


using json = nlohmann::json;


namespace Piston {

std::string bson_to_hex_string(const std::vector<uint8_t>& vec, bool truncate = false);
std::vector<uint8_t> hex_string_to_bson(const std::string& str);

bool isMeshGeoPrim(const pxr::UsdPrim& pGeoPrim);
bool isCurvesGeoPrim(const pxr::UsdPrim& getPrim);

class SerializableDeformerDataBase;

class UsdPrimHandle {
	public:
		UsdPrimHandle();
		UsdPrimHandle(const pxr::UsdPrim& pPrim);

		const pxr::UsdPrim& getPrim() const;

		bool isValid() const { return mPrim.IsValid(); }

		bool isMeshGeoPrim() const { return mPrim.GetTypeName() == "Mesh"; }
		bool isHairGeoPrim() const { return mPrim.GetTypeName() == "BasisCurves"; }

		std::string  getName() const { return getPath().GetName(); }
		pxr::SdfPath getPath() const { return mPrim.GetPath(); }
		pxr::UsdStageWeakPtr getStage() const { return mPrim.GetStage(); }

		bool getDataFromBson(SerializableDeformerDataBase* pDeformerData) const;
		bool writeDataToBson(SerializableDeformerDataBase* pDeformerData) const;

		bool getBsonFromPrim(const std::string& identifier, std::vector<std::uint8_t>& v_bson) const;
		bool setBsonToPrim(const std::string& identifier, const std::vector<std::uint8_t>& v_bson) const;
		void clearPrimBson(const std::string& identifier) const;

		pxr::UsdGeomPrimvarsAPI getPrimvarsAPI() const { return pxr::UsdGeomPrimvarsAPI::Get(getStage(), getPath()); }

		double getStageFPS() const;
		double getStageTimeCodesPerSecond() const;

		/* Invalidate handle */
		void clear();

		bool operator==(const pxr::UsdPrim& prim) const;
		explicit operator bool() const { return mPrim.IsValid(); };

	private:
		pxr::UsdPrim            mPrim;

};

class PointsList {
	public:
		PointsList(size_t size);

		PointsList(Piston::PointsList&& other);

		size_t size() const { return mPoints.size(); }

		pxr::GfVec3f& operator [](size_t idx) { return mPoints[idx]; }
		const pxr::GfVec3f& operator [](size_t idx) const { return mPoints[idx]; }

		pxr::GfVec3f* data() { return mPoints.data(); }
		const pxr::GfVec3f* data() const { return mPoints.data(); }

		std::vector<pxr::GfVec3f>& getVector() { return mPoints; }
		const std::vector<pxr::GfVec3f>& getVector() const { return mPoints; }

		const pxr::VtArray<pxr::GfVec3f>& getVtArray() const { return mVtArray; }

		void resize(size_t size);

		void fillWithZero();

		size_t sizeInBytes() const { return mSizeInBytes; }

	private:
		std::vector<pxr::GfVec3f> 	mPoints;
		pxr::VtArray<pxr::GfVec3f> 	mVtArray;
		pxr::Vt_ArrayForeignDataSource 	mForeignDataSource;

		void calcSizeInBytes() const;

		mutable size_t mSizeInBytes;
};

} // namespace Piston

#endif // PISTON_LIB_COMMON_H_