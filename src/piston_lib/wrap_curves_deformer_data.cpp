#include "wrap_curves_deformer_data.h"
#include "pxr_json.h"

namespace Piston {

static const SerializableDeformerDataBase::DataVersion kWrapBindingDataVersion( 0u, 0u, 0u);

void  WrapCurvesDeformerData::setBindMode(const WrapCurvesDeformerData::BindMode& mode) {
	if(mBindMode == mode) return;

	mBindMode = mode;
	clear();
}

void WrapCurvesDeformerData::clearData() {
	mPointBinds.clear();
}

size_t WrapCurvesDeformerData::calcHash() const {
	size_t hash = 0;

	for(const auto& bind: mPointBinds) {
		hash += static_cast<size_t>(bind.face_id * (bind.u + bind.v + bind.dist));
	}
	hash += mPointBinds.size();

	return hash;
}

static constexpr const char* kJPointBinds = "pointbinds";
static constexpr const char* kJMode = "mode";
static constexpr const char* kJDataHash = "data_hash";


bool WrapCurvesDeformerData::dumpToJSON(json& j) const {
	j[kJPointBinds] = mPointBinds;
	j[kJMode] = to_string(mBindMode);
	j[kJDataHash] = calcHash();

	return true;
}

bool WrapCurvesDeformerData::readFromJSON(const json& j) {
	mPointBinds = j[kJPointBinds].template get<std::vector<PointBindData>>();
	const BindMode bind_mode = from_string(j[kJMode].template get<std::string>());

	if(bind_mode != mBindMode) {
		std::cerr << typeName() << " json data bind mode mismatch !";
		return false;
	}

	if(j[kJDataHash].template get<size_t>() != calcHash()) {
		std::cerr << typeName() << " json data hash mismatch !";
		return false;
	}

	dbg_printf("WrapCurvesDeformerData data read from json payload !\n");

	return true;
}

const std::string& WrapCurvesDeformerData::typeName() const {
	static const std::string kTypeName = "WrapCurvesDeformerData";
	return kTypeName;
}

const std::string& WrapCurvesDeformerData::jsonDataKey() const {
	static const std::string kDataKey = "piston_wrap_deformer_data";
	return kDataKey;
}

const SerializableDeformerDataBase::DataVersion& WrapCurvesDeformerData::jsonDataVersion() const {
	return kWrapBindingDataVersion;
}

void to_json(json& j, const WrapCurvesDeformerData::PointBindData& bind) {
	j = {bind.face_id, bind.u, bind.v, bind.dist};
}

void from_json(const json& j, WrapCurvesDeformerData::PointBindData& bind) {
	bind.face_id = j.at(0).template get<uint32_t>();
	bind.u = j.at(1).template get<float>();
	bind.v = j.at(2).template get<float>();
	bind.dist = j.at(3).template get<float>();
}

} // namespace Piston