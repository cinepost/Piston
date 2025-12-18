#include "guides_curves_deformer_data.h"
#include "pxr_json.h"

namespace Piston {

static const SerializableDeformerDataBase::DataVersion kGuidesBindingDataVersion( 0u, 0u, 0u);

void GuidesCurvesDeformerData::clearData() {
	mPointBinds.clear();
}

size_t GuidesCurvesDeformerData::calcHash() const {
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

static constexpr const char* kJPointBinds = "pointbinds";
static constexpr const char* kJDataHash = "data_hash";


bool GuidesCurvesDeformerData::dumpToJSON(json& j) const {
	j[kJPointBinds] = mPointBinds;
	j[kJDataHash] = calcHash();

	return true;
}

bool GuidesCurvesDeformerData::readFromJSON(const json& j) {
	mPointBinds = j[kJPointBinds].template get<std::vector<PointBindData>>();

	if(j[kJDataHash].template get<size_t>() != calcHash()) {
		std::cerr << typeName() << " json data hash mismatch !";
		return false;
	}

	dbg_printf("GuidesCurvesDeformerData data read from json payload !\n");
	return true;
}

const std::string& GuidesCurvesDeformerData::typeName() const {
	static const std::string kTypeName = "GuidesCurvesDeformerData";
	return kTypeName;
}

const std::string& GuidesCurvesDeformerData::jsonDataKey() const {
	static const std::string kDataKey = "piston_guides_deformer_data";
	return kDataKey;
}

const SerializableDeformerDataBase::DataVersion& GuidesCurvesDeformerData::jsonDataVersion() const {
	return kFastBindingDataVersion;
}

void to_json(json& j, const GuidesCurvesDeformerData::CurveBindData& bind) {
	j = {bind.face_id, bind.u, bind.v};
}

void from_json(const json& j, GuidesCurvesDeformerData::CurveBindData& bind) {
	bind.face_id = j.at(0).template get<uint32_t>();
	bind.u = j.at(1).template get<float>();
	bind.v = j.at(2).template get<float>();
}

} // namespace Piston