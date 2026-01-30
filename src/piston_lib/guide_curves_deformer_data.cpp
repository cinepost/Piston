#include "guide_curves_deformer_data.h"
#include "pxr_json.h"

namespace Piston {

static const SerializableDeformerDataBase::DataVersion kGuidesBindingDataVersion( 0u, 0u, 0u);

void GuideCurvesDeformerData::clearData() {
	mPointBinds.clear();
	mSkinPrimPath = "";
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
static constexpr const char* kJSkinPrimPath = "skin_prim_path";

bool GuideCurvesDeformerData::dumpToJSON(json& j) const {
	j[kJMode] = static_cast<uint8_t>(mBindMode);
	j[kJPointBinds] = mPointBinds;
	j[kJSkinPrimPath] = mSkinPrimPath;
	j[kJDataHash] = calcHash();

	return true;
}

bool GuideCurvesDeformerData::readFromJSON(const json& j) {
	const BindMode bind_mode = static_cast<GuideCurvesDeformerData::BindMode>(j[kJMode].template get<uint8_t>());

	if(bind_mode != mBindMode) {
		std::cerr << typeName() << " json data bind mode mismatch !";
		return false;
	}

	const std::string skin_prim_path = j[kJSkinPrimPath].template get<std::string>();
	if((skin_prim_path != mSkinPrimPath) && (mBindMode == BindMode::NTB)) {
		std::cerr << typeName() << " json data skin primitive path mismatch !";
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

void GuideCurvesDeformerData::setBindMode(const GuideCurvesDeformerData::BindMode& mode) {
	if(mBindMode == mode) return;
	mBindMode = mode;
	clear();
}

uint32_t GuideCurvesDeformerData::PointBindData::encodeID_modeANGLE(uint32_t guide_id, uint8_t guide_vertex_id) {
	// We use 23 bits for guide curve id and 8 bits for guide curve vertex

	static const uint32_t kMaxGuideID = 8388607; // 23 bit

	assert(guide_id <= kMaxGuideID);

	return ((guide_id & 0x007FFFFF) << 8) | (uint32_t)guide_vertex_id; 
}

void GuideCurvesDeformerData::PointBindData::decodeID_modeANGLE(uint32_t encoded_id, uint32_t& guide_id, uint8_t& guide_vertex_id) {
	guide_id = (encoded_id >> 8) & 0x007FFFFF;
	guide_vertex_id = encoded_id & 0x000000FF;
}

uint32_t GuideCurvesDeformerData::PointBindData::encodeID_modeSPACE(uint32_t id,) {
	return id & 0x7FFFFFFF;
}

void GuideCurvesDeformerData::PointBindData::decodeID_modeSPACE(uint32_t encoded_id, uint32_t& id) {
	id = encoded_id & 0x7FFFFFFFF;
}

void GuideCurvesDeformerData::setSkinPrimPath(const std::string& prim_path) {
	if(mSkinPrimPath == prim_path) return;
	mSkinPrimPath = prim_path;
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
	j = {bind.encoded_id, bind.vec[0], bind.vec[1], bind.vec[2]};
}

void from_json(const json& j, GuideCurvesDeformerData::PointBindData& bind) {
	bind.encoded_id = j.at(0).template get<uint32_t>();
	bind.vec = {j.at(2).template get<float>(), j.at(3).template get<float>(), j.at(4).template get<float>()};
}

} // namespace Piston