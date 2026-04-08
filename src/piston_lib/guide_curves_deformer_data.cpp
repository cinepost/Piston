#include "guide_curves_deformer_data.h"
#include "pxr_json.h"
#include "logging.h"


namespace Piston {

static const SerializableDeformerDataBase::DataVersion kGuidesBindingDataVersion( 0u, 0u, 0u);

inline void to_json(json& j, const GuideCurvesDeformerData::PointBindData& bind) {
	j = {bind.encoded_id.raw_data, bind.data[0], bind.data[1], bind.data[2]};
}

inline void from_json(const json& j, GuideCurvesDeformerData::PointBindData& bind) {
	bind.encoded_id.raw_data = j.at(0).template get<uint32_t>();
	bind.data = {j.at(2).template get<float>(), j.at(3).template get<float>(), j.at(4).template get<float>()};
}

inline void to_json(json& j, const GuideCurvesDeformerData::PointSurfaceBindData& bind) {
	j = {bind.face_id, bind.point_id, static_cast<uint32_t>(bind.u.toBits()) | (static_cast<uint32_t>(bind.v.toBits()) << 16), static_cast<uint32_t>(bind.dist.toBits()) | (static_cast<uint32_t>(bind.weight.toBits()) << 16)};
}

inline void from_json(const json& j, GuideCurvesDeformerData::PointSurfaceBindData& bind) {
	bind.face_id = j.at(0).template get<uint32_t>();
	bind.point_id = j.at(1).template get<uint32_t>();

	uint32_t uv = j.at(2).template get<uint32_t>();
	uint32_t dw = j.at(3).template get<uint32_t>();

	bind.u.fromBits(uv & 0x00FF);
	bind.v.fromBits(uv >> 16);
	bind.dist.fromBits(dw & 0x00FF);
	bind.weight.fromBits(dw >> 16);
}

inline void to_json(json& j, const GuideCurvesDeformerData::GuideOrigin& o) {
	j = o.raw_data;
}

inline void from_json(const json& j, GuideCurvesDeformerData::GuideOrigin& o) {
	o.raw_data = j.at(0).template get<uint32_t>();
}

void GuideCurvesDeformerData::clearData() {
	mPointBinds.clear();
	mPointSurfaceBinds.clear();
	mGuideOrigins.clear();
	mSkinPrimPath = "";
	setPopulated(false);
}

size_t GuideCurvesDeformerData::calcHash() const {
	size_t hash = 0;

	for(const auto& bind: mPointBinds) {
		hash += bind.hash();
	}
	hash += mPointBinds.size();

	for(const auto& bind: mPointSurfaceBinds) {
		hash += bind.hash();
	}
	hash += mPointSurfaceBinds.size();

	for(int idx: mSkinPrimIndices) {
		hash += static_cast<size_t>(idx);
	}
	hash += mSkinPrimIndices.size();

	for(uint32_t id: mGuideOrigins) {
		hash += id;
	}
	hash += mGuideOrigins.size();

	return hash;
}

static const char* kJMode = "mode";
static const char* kJPointBinds = "pointbinds";
static const char* kJPointSurfaceBinds = "pointsurfacebinds";
static const char* kJGuideOrigins = "guideorigs";
static const char* kJDataHash = "data_hash";
static const char* kJSkinPrimPath = "skin_prim_path";
static const char* kJSkinPrimIndices = "skin_prim_indices";

bool GuideCurvesDeformerData::dumpToJSON(json& j) const {
	static const std::vector<GuideOrigin> kEmptyGuideOrigins;
	j[kJMode] = static_cast<uint8_t>(mBindMode);
	j[kJPointBinds] = mPointBinds;
	j[kJPointSurfaceBinds] = mPointSurfaceBinds;
	j[kJGuideOrigins] = (mBindMode == BindMode::NTB) ? mGuideOrigins : kEmptyGuideOrigins;
	j[kJSkinPrimPath] = mSkinPrimPath;
	j[kJSkinPrimPath] = mSkinPrimIndices;
	j[kJSkinPrimIndices] = mSkinPrimIndices;
	j[kJDataHash] = calcHash();

	return true;
}

/*
		std::vector<PointBindData> 			mPointBinds;
		std::vector<GuideOrigin> 			mGuideOrigins;
		std::vector<PointSurfaceBindData> 	mPointSurfaceBinds;
		BindMode                    		mBindMode = BindMode::NTB;
		std::string                 		mSkinPrimPath;
		std::vector<int> 					mSkinPrimIndices;

		bool                                mKeepRootsOnSurface = true;
*/

bool GuideCurvesDeformerData::readFromJSON(const json& j) {
	clearData();

	const BindMode bind_mode = static_cast<GuideCurvesDeformerData::BindMode>(j[kJMode].template get<uint8_t>());

	if(bind_mode != mBindMode) {
		LOG_ERR << typeName() << " json data bind mode mismatch !";
		return false;
	}

	const std::string skin_prim_path = j[kJSkinPrimPath].template get<std::string>();
	if((skin_prim_path != mSkinPrimPath) && (mBindMode == BindMode::NTB)) {
		LOG_ERR << typeName() << " json data skin primitive path mismatch !";
		return false;
	}

	mPointBinds = j[kJPointBinds].template get<std::vector<PointBindData>>();

	if(mBindMode == BindMode::NTB) {
		mGuideOrigins = j[kJGuideOrigins].template get<std::vector<GuideOrigin>>();
	}

	mPointSurfaceBinds = j[kJPointSurfaceBinds].template get<std::vector<PointSurfaceBindData>>();
	mSkinPrimIndices = j[kJSkinPrimIndices].template get<std::vector<int>>();

	if(j[kJDataHash].template get<size_t>() != calcHash()) {
		LOG_ERR << typeName() << " json data hash mismatch !";
		return false;
	}

	LOG_DBG << "GuideCurvesDeformerData data read from json payload !";

	setPopulated(true);
	return true;
}

void GuideCurvesDeformerData::setBindMode(const GuideCurvesDeformerData::BindMode& mode) {
	if(mBindMode == mode) return;
	mBindMode = mode;
	clear();
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

} // namespace Piston