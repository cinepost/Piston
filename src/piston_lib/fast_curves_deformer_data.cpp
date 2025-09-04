#include "fast_curves_deformer_data.h"
#include "pxr_json.h"

namespace Piston {

static const SerializableDeformerDataBase::DataVersion kFastBindingDataVersion( 0u, 0u, 0u);

void FastCurvesDeformerData::clearData() {
	mCurveBinds.clear();
	mRestVertexNormals.clear();
	mPerBindRestNormals.clear();
	mPerBindRestTBs.clear();
}

size_t FastCurvesDeformerData::calcHash() const {
	size_t hash = 0;

	for(const auto& bind: mCurveBinds) {
		hash += static_cast<size_t>(bind.face_id * (bind.u + bind.v));
	}
	hash += mCurveBinds.size();

	for(const auto& n: mRestVertexNormals) {
		hash += static_cast<size_t>(n[0] + n[1] + n[2]);
	}
	hash += mRestVertexNormals.size();

	for(const auto& n: mPerBindRestNormals) {
		hash += static_cast<size_t>(n[0] + n[1] + n[2]);
	}
	hash += mPerBindRestNormals.size();

	for(const auto& [t, b]: mPerBindRestTBs) {
		hash += static_cast<size_t>(t[0]*b[0] + t[1]*b[1] + t[2]*b[2]);
	}
	hash += mPerBindRestTBs.size();

	return hash;
}

static constexpr const char* kJCurveBinds = "curvebinds";
static constexpr const char* kJRestVertexNormals = "restvtxnormals";
static constexpr const char* kPerBindRestNormals = "perbindvtxnormals";
static constexpr const char* kPerBindrBindRestTBs = "perbindtbs";

static constexpr const char* kJDataHash = "data_hash";


bool FastCurvesDeformerData::dumpToJSON(json& j) const {
	j[kJCurveBinds] = mCurveBinds;
	
	to_json(j[kJRestVertexNormals], mRestVertexNormals);
	to_json(j[kPerBindRestNormals], mPerBindRestNormals);
	to_json(j[kPerBindrBindRestTBs], mPerBindRestTBs);

	j[kJDataHash] = calcHash();

	return true;
}

bool FastCurvesDeformerData::readFromJSON(const json& j) {
	mCurveBinds = j[kJCurveBinds].template get<std::vector<CurveBindData>>();

	from_json(j[kJRestVertexNormals], mRestVertexNormals);
	from_json(j[kPerBindRestNormals], mPerBindRestNormals);
	from_json(j[kPerBindrBindRestTBs], mPerBindRestTBs);

	if(j[kJDataHash] != calcHash()) {
		std::cerr << typeName() << " json data hash mismatch !";
		return false;
	}

	dbg_printf("FastCurvesDeformerData data read from json payload !\n");

	return true;
}

const std::string& FastCurvesDeformerData::typeName() const {
	static const std::string kTypeName = "FastCurvesDeformerData";
	return kTypeName;
}

const std::string& FastCurvesDeformerData::jsonDataKey() const {
	static const std::string kDataKey = "piston_fast_deformer_data";
	return kDataKey;
}

const SerializableDeformerDataBase::DataVersion& FastCurvesDeformerData::jsonDataVersion() const {
	return kFastBindingDataVersion;
}

void to_json(json& j, const FastCurvesDeformerData::CurveBindData& bind) {
	j = {bind.face_id, bind.u, bind.v};
	//to_json(j["normal"], face.restNormal);
}

void from_json(const json& j, FastCurvesDeformerData::CurveBindData& bind) {
	bind.face_id = j.at(0).template get<uint32_t>();
	bind.u = j.at(1).template get<float>();
	bind.v = j.at(2).template get<float>();
}

} // namespace Piston