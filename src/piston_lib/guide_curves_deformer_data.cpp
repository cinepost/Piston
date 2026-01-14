#include "guide_curves_deformer_data.h"
#include "pxr_json.h"

namespace Piston {

static const SerializableDeformerDataBase::DataVersion kGuidesBindingDataVersion( 0u, 0u, 0u);

void GuideCurvesDeformerData::clearData() {
	mPointBinds.clear();
}

size_t GuideCurvesDeformerData::calcHash() const {
	size_t hash = 0;

	for(const auto& bind: mPointBinds) {
		hash += bind.hash();
	}
	hash += mPointBinds.size();

	return hash;
}

static constexpr const char* kJMode = "mode";
static constexpr const char* kJPointBinds = "pointbinds";
static constexpr const char* kJDataHash = "data_hash";


bool GuideCurvesDeformerData::dumpToJSON(json& j) const {
	j[kJMode] = static_cast<uint8_t>(mBindMode);
	j[kJPointBinds] = mPointBinds;
	j[kJDataHash] = calcHash();

	return true;
}

bool GuideCurvesDeformerData::readFromJSON(const json& j) {
	const BindMode bind_mode = static_cast<GuideCurvesDeformerData::BindMode>(j[kJMode].template get<uint8_t>());

	if(bind_mode != mBindMode) {
		std::cerr << typeName() << " json data bind mode mismatch !";
		return false;
	}

	mPointBinds = j[kJPointBinds].template get<std::vector<PointBindData>>();

	if(j[kJDataHash].template get<size_t>() != calcHash()) {
		std::cerr << typeName() << " json data hash mismatch !";
		return false;
	}

	dbg_printf("GuideCurvesDeformerData data read from json payload !\n");
	return true;
}

void  GuideCurvesDeformerData::setBindMode(const GuideCurvesDeformerData::BindMode& mode) {
	if(mBindMode == mode) return;

	mBindMode = mode;
	clear();
}

const std::string& GuideCurvesDeformerData::typeName() const {
	static const std::string kTypeName = "GuideCurvesDeformerData";
	return kTypeName;
}

const std::string& GuideCurvesDeformerData::jsonDataKey() const {
	static const std::string kDataKey = "piston_guides_deformer_data";
	return kDataKey;
}

const SerializableDeformerDataBase::DataVersion& GuideCurvesDeformerData::jsonDataVersion() const {
	return kGuidesBindingDataVersion;
}

void to_json(json& j, const GuideCurvesDeformerData::PointBindData& bind) {
	j = {bind.guide_id, bind.vtx, bind.vec[0], bind.vec[1], bind.vec[2]};
}

void from_json(const json& j, GuideCurvesDeformerData::PointBindData& bind) {
	bind.guide_id = j.at(0).template get<uint16_t>();
	bind.vtx = j.at(1).template get<uint16_t>();
	bind.vec = {j.at(2).template get<float>(), j.at(3).template get<float>(), j.at(4).template get<float>()};
}

} // namespace Piston