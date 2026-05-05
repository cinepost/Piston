#ifndef PISTON_LIB_COMMON_H_
#define PISTON_LIB_COMMON_H_

#include "framework.h"

#include <nlohmann/json.hpp>

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


using json = nlohmann::json;


namespace Piston {

class BaseCurvesDeformer;

using PxrTopologyVariant = std::variant<pxr::HdMeshTopology, pxr::HdBasisCurvesTopology>;

struct Topology {
	size_t             topology_hash;
	PxrTopologyVariant topology_variant;
	pxr::UsdTimeCode   time_code;

	Topology(size_t hash, PxrTopologyVariant&& variant, const pxr::UsdTimeCode time = pxr::UsdTimeCode::Default()) {
		topology_hash = hash;
		topology_variant = std::move(variant);
		time_code = time;
	}

	bool operator==(const Topology& other) const {
		return topology_hash == other.topology_hash && topology_variant == other.topology_variant;
	}

	bool operator!=(const Topology& other) const {
		return topology_hash != other.topology_hash || !(topology_variant == other.topology_variant);
	}
};

const char *stringifyMemSize(size_t bytes);

std::string bson_to_hex_string(const BSON& bson);
void hex_string_to_bson(const std::string& str, BSON& bson);

std::string getStageName(pxr::UsdStageRefPtr pStage);

inline bool isMeshGeoPrim(const pxr::UsdPrim& prim) { return prim.IsValid() && prim.IsA<pxr::UsdGeomMesh>(); }
inline bool isBasisCurvesGeoPrim(const pxr::UsdPrim& prim) { return prim.IsValid() && prim.IsA<pxr::UsdGeomBasisCurves>(); }

inline bool isSameType(const pxr::UsdPrim& prim_l, const pxr::UsdPrim& prim_r) {
	return prim_l.GetTypeName() == prim_r.GetTypeName();
}

bool clearPistonDataFromStage(pxr::UsdStageRefPtr pStage);
bool clearPistonDataFromPrim(pxr::UsdStageRefPtr pStage, const pxr::SdfPath& prim_path);

class SerializableDeformerDataBase;

class UsdPrimHandle {
	public:
		UsdPrimHandle();
		UsdPrimHandle(const pxr::UsdPrim& pPrim);
		UsdPrimHandle(const std::shared_ptr<BaseCurvesDeformer>& pDeformer);
		UsdPrimHandle(UsdPrimHandle&& other) noexcept;

		UsdPrimHandle& operator=(UsdPrimHandle&& other) noexcept;

		const pxr::UsdPrim& getPrim() const;

		bool isValid() const { return getPrim().IsValid(); }

		bool isMeshGeoPrim() const { return Piston::isMeshGeoPrim(getPrim()); }
		bool isBasisCurvesGeoPrim() const { return Piston::isBasisCurvesGeoPrim(getPrim()); }

		std::string  getFullName() const { return getPath().GetText(); }
		std::string  getName() const { return getPath().GetName(); }
		pxr::SdfPath getPath() const { return getPrim().GetPath(); }
		pxr::UsdStageWeakPtr getStage() const { return getPrim().GetStage(); }

		bool getDataFromBson(const pxr::SdfPath& prim_path, SerializableDeformerDataBase* pDeformerData) const;
		bool writeDataToBson(const pxr::SdfPath& prim_path, SerializableDeformerDataBase* pDeformerData) const;

		bool getBsonFromPrim(const pxr::SdfPath& prim_path, const std::string& identifier, BSON& v_bson) const;
		bool setBsonToPrim(const pxr::SdfPath& prim_path, const std::string& identifier, const BSON& v_bson) const;
		void clearPrimBson(const pxr::SdfPath& prim_path, const std::string& identifier) const;

		template<typename T>
		bool fetchAttributeValues(const std::string& attribute_name, pxr::VtArray<T>& array, pxr::UsdTimeCode time_code=pxr::UsdTimeCode::Default()) const;

		template<typename T>
		bool fetchAttributeValues(const std::string& attribute_name, std::vector<T>& vec, pxr::UsdTimeCode time_code=pxr::UsdTimeCode::Default()) const;

		bool getPoints(pxr::VtArray<pxr::GfVec3f>& array, pxr::UsdTimeCode time_code=pxr::UsdTimeCode::Default()) const;

		pxr::UsdGeomPrimvarsAPI getPrimvarsAPI() const { return pxr::UsdGeomPrimvarsAPI::Get(getStage(), getPath()); }

		double getStageFPS() const;
		double getStageTimeCodesPerSecond() const;

		const Topology& getTopology(pxr::UsdTimeCode time_code=pxr::UsdTimeCode::Default()) const;
		size_t getTopologyHash(pxr::UsdTimeCode time_code=pxr::UsdTimeCode::Default()) const;

		/* Invalidate handle */
		void clear();

		bool operator==(const pxr::UsdPrim& prim) const;
		explicit operator bool() const { return isValid(); };

		friend std::ostream& operator<<( std::ostream& os, const UsdPrimHandle& prim_handle ) {
        	os << prim_handle.getPath();
        	return os;
  		}

  	private:
  		bool prepareDataIfNeeded(pxr::UsdTimeCode time_code=pxr::UsdTimeCode::Default()) const;

	private:
		pxr::UsdPrim     mPrim;
		std::shared_ptr<BaseCurvesDeformer> mpDeformer;
		mutable std::unique_ptr<Topology> mpTopology;

};

class PointsList {
	public:
		PointsList(size_t size);

		PointsList(Piston::PointsList&& other);

		size_t size() const { assert(mPoints.size() == mVtArray.size()); return mPoints.size(); }

		pxr::GfVec3f& operator [](size_t idx) { return mPoints[idx]; }
		const pxr::GfVec3f& operator [](size_t idx) const { return mPoints[idx]; }

		pxr::GfVec3f* data() { return mPoints.data(); }
		const pxr::GfVec3f* data() const { return mPoints.data(); }

		std::vector<pxr::GfVec3f>& getVector() { return mPoints; }
		const std::vector<pxr::GfVec3f>& getVector() const { return mPoints; }

		const pxr::VtArray<pxr::GfVec3f>& getVtArray() const { return mVtArray; }

		void resize(size_t new_size);

		void fillWithZero();

		size_t sizeInBytes() const { return mSizeInBytes; }

	private:
		std::vector<pxr::GfVec3f> 	mPoints;
		pxr::VtArray<pxr::GfVec3f> 	mVtArray;
		pxr::Vt_ArrayForeignDataSource 	mForeignDataSource;

		void calcSizeInBytes() const;

		mutable size_t mSizeInBytes;
};

inline std::ostream& operator<<( std::ostream& os, const pxr::UsdPrim& prim ) {
	os << prim.GetPath();
	return os;
}

inline size_t getTopologyHash(const PxrTopologyVariant& topology) {
    return std::visit([](const auto& t) -> size_t {
        return static_cast<size_t>(t.ComputeHash());
    }, topology);
}

} // namespace Piston

#endif // PISTON_LIB_COMMON_H_