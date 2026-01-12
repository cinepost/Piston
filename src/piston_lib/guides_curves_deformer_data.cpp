#include "guides_curves_deformer_data.h"
#include "pxr_json.h"

namespace Piston {

static const SerializableDeformerDataBase::DataVersion kGuidesBindingDataVersion( 0u, 0u, 0u);

void GuidesCurvesDeformerData::clearData() {
	mPointBinds.clear();
}

size_t GuidesCurvesDeformerData::calcHash() const {
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


bool GuidesCurvesDeformerData::dumpToJSON(json& j) const {
	j[kJMode] = static_cast<uint8_t>(mBindMode);
	j[kJPointBinds] = mPointBinds;
	j[kJDataHash] = calcHash();

	return true;
}

bool GuidesCurvesDeformerData::readFromJSON(const json& j) {
	const BindMode bind_mode = static_cast<GuidesCurvesDeformerData::BindMode>(j[kJMode].template get<uint8_t>());

	if(bind_mode != mBindMode) {
		std::cerr << typeName() << " json data bind mode mismatch !";
		return false;
	}

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
	return kGuidesBindingDataVersion;
}

void to_json(json& j, const GuidesCurvesDeformerData::PointBindData& bind) {
	j = {bind.guide_id, bind.vtx, bind.vec[0], bind.vec[1], bind.vec[2]};
}

void from_json(const json& j, GuidesCurvesDeformerData::PointBindData& bind) {
	bind.guide_id = j.at(0).template get<uint16_t>();
	bind.vtx = j.at(1).template get<uint16_t>();
	bind.vec = {j.at(2).template get<float>(), j.at(3).template get<float>(), j.at(4).template get<float>()};
}

} // namespace Piston