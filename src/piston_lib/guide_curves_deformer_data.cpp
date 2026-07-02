#include "guide_curves_deformer_data.h"
#include "pxr_json.h"
#include "logging.h"


namespace Piston {

static_assert(sizeof(GuideCurvesDeformerData::PointBindData::EncodedID) == 4);
static_assert(sizeof(GuideCurvesDeformerData::PointBindData) == 16);

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

inline void to_json(json& j, const GuideCurvesDeformerData::PointBindDataLHS& bind) {
	j = {
		bind.curveIndices[0], //u32 
		bind.curveIndices[1], //u32
		bind.curveIndices[2], //u32

		static_cast<uint32_t>(bind.v[0]) | (static_cast<uint32_t>(bind.v[1]) << 8) | (static_cast<uint32_t>(bind.v[2]) << 16), //u32
		static_cast<uint32_t>(bind.v[3]) | (static_cast<uint32_t>(bind.v[4]) << 8) | (static_cast<uint32_t>(bind.v[5]) << 16), //u32

		static_cast<uint32_t>(bind.w[0].toBits()) | (static_cast<uint32_t>(bind.w[1].toBits()) << 16), //u32
		static_cast<uint32_t>(bind.w[2].toBits()) | (static_cast<uint32_t>(bind.w[3].toBits()) << 16), //u32
		static_cast<uint32_t>(bind.w[4].toBits()) | (static_cast<uint32_t>(bind.w[5].toBits()) << 16) //u32

	};
/*
	uint32_t curveIndices[3]; 	// TODO: use three 24 or 30 bit indices and rest bits are for flags
	uint8_t v[6];            	// TODO: use relative 8bit vertex indices
	float16_t w[6];				// TODO: float16_t weight
*/	
}

inline void from_json(const json& j, GuideCurvesDeformerData::PointBindDataLHS& bind) {
	bind.curveIndices[0] = j.at(0).template get<uint32_t>();
	bind.curveIndices[1] = j.at(1).template get<uint32_t>();
	bind.curveIndices[2] = j.at(2).template get<uint32_t>();

	uint32_t v012 = j.at(3).template get<uint32_t>();
	bind.v[0] = v012 & 0x000000FF;
	bind.v[1] = (v012 >> 8 ) & 0x000000FF;
	bind.v[2] = v012 >> 16;

	uint32_t v345 = j.at(4).template get<uint32_t>();
	bind.v[3] = v345 & 0x000000FF;
	bind.v[4] = (v345 >> 8 ) & 0x000000FF;
	bind.v[5] = v345 >> 16;

	uint32_t w01 = j.at(5).template get<uint32_t>();
	bind.w[0].fromBits(w01 & 0x0000FFFF); 
	bind.w[1].fromBits(w01 >> 16);

	uint32_t w23 = j.at(6).template get<uint32_t>();
	bind.w[2].fromBits(w23 & 0x0000FFFF); 
	bind.w[3].fromBits(w23 >> 16);

	uint32_t w45 = j.at(7).template get<uint32_t>();
	bind.w[4].fromBits(w45 & 0x0000FFFF); 
	bind.w[5].fromBits(w45 >> 16);
}

inline void to_json(json& j, const GuideCurvesDeformerData::BlendedNTBData& bind) {
	j = {
		bind.guide_id[0], //u32 
		bind.guide_id[1], //u32
		bind.guide_id[2], //u32
		static_cast<uint32_t>(bind.v[0]) | (static_cast<uint32_t>(bind.v[1]) << 8) | (static_cast<uint32_t>(bind.v[2]) << 16), //u32
		static_cast<uint32_t>(bind.w[0].toBits()) | (static_cast<uint32_t>(bind.w[1].toBits()) << 16), //u32
		static_cast<uint16_t>(bind.w[2].toBits()), // u16
		bind.coords[0][0], bind.coords[0][1], bind.coords[0][2],
		bind.coords[1][0], bind.coords[1][1], bind.coords[1][2],
		bind.coords[2][0], bind.coords[2][1], bind.coords[2][2]
	};
/*
	uint32_t 	guide_id[3];
	uint8_t  	v[3]; 		// vertices. 2 vertices per guide
	float16_t   w[3]; 		// weights. 2 weights per guide
	pxr::GfVec3f coords[3]; // ntb coords
*/
}

inline void from_json(const json& j, GuideCurvesDeformerData::BlendedNTBData& bind) {
	bind.guide_id[0] = j.at(0).template get<uint32_t>();
	bind.guide_id[1] = j.at(1).template get<uint32_t>();
	bind.guide_id[2] = j.at(2).template get<uint32_t>();

	uint32_t v012 = j.at(3).template get<uint32_t>();
	bind.v[0] = v012 & 0x000000FF;
	bind.v[1] = (v012 >> 8 ) & 0x000000FF;
	bind.v[2] = v012 >> 16;

	uint32_t w01 = j.at(4).template get<uint32_t>();
	bind.w[0].fromBits(w01 & 0x0000FFFF); 
	bind.w[1].fromBits(w01 >> 16);
	bind.w[2].fromBits(j.at(5).template get<uint16_t>());

	bind.coords[0] = {j.at(6).template get<float>(), j.at(7).template get<float>(), j.at(8).template get<float>()};
	bind.coords[1] = {j.at(9).template get<float>(), j.at(10).template get<float>(), j.at(11).template get<float>()};
	bind.coords[2] = {j.at(12).template get<float>(), j.at(13).template get<float>(), j.at(14).template get<float>()};
}

void GuideCurvesDeformerData::clearData() {
	const std::lock_guard<std::mutex> lock(mMutex);

	mPointBinds.clear();
	mBlendNTBPointBinds.clear();
	mLHSPointBinds.clear();

	mPointSurfaceBinds.clear();
	mGuideOrigins.clear();
	mSkinPrimPath = "";
	mIsValid = false;
}

size_t GuideCurvesDeformerData::calcHash() const {
	size_t hash = 0;

	for(const auto& bind: mPointBinds) {
		hash += bind.hash();
	}
	hash += mPointBinds.size();

	for(const auto& bind: mBlendNTBPointBinds) {
		hash += bind.hash();
	}
	hash += mBlendNTBPointBinds.size();

	for(const auto& bind: mLHSPointBinds) {
		hash += bind.hash();
	}
	hash += mLHSPointBinds.size();

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

static const char* kJLHSPointBinds = "lhs_pointbinds";
static const char* kJBlendNTBPointBinds = "ntb_blend_pointbinds";

static const char* kJPointSurfaceBinds = "pointsurfacebinds";
static const char* kJGuideOrigins = "guideorigs";
static const char* kJDataHash = "data_hash";
static const char* kJSkinPrimPath = "skin_prim_path";
static const char* kJSkinPrimIndices = "skin_prim_indices";

bool GuideCurvesDeformerData::dumpToJSON(json& j) const {
	const std::lock_guard<std::mutex> lock(mMutex);

	static const std::vector<GuideOrigin> kEmptyGuideOrigins;
	static const std::vector<PointBindDataLHS> kEmptyLHSBinds;
	static const std::vector<BlendedNTBData> kEmptyBlendNTBBinds;

	j[kJMode] = static_cast<uint8_t>(mBindMode);
	
	j[kJPointBinds] = mPointBinds;

	j[kJLHSPointBinds] = (mBindMode == BindMode::LHS) ? mLHSPointBinds : kEmptyLHSBinds;
	j[kJBlendNTBPointBinds] = (mBindMode == BindMode::BLEND) ? mBlendNTBPointBinds : kEmptyBlendNTBBinds;

	j[kJPointSurfaceBinds] = mPointSurfaceBinds;
	j[kJGuideOrigins] = (mBindMode == BindMode::NTB) ? mGuideOrigins : kEmptyGuideOrigins;
	j[kJSkinPrimPath] = mSkinPrimPath;
	j[kJSkinPrimPath] = mSkinPrimIndices;
	j[kJSkinPrimIndices] = mSkinPrimIndices;
	j[kJDataHash] = calcHash();

	return true;
}

bool GuideCurvesDeformerData::readFromJSON(const json& j) {
	const std::lock_guard<std::mutex> lock(mMutex);

	mIsValid = false;

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
	} else if(mBindMode == BindMode::LHS) {
		mLHSPointBinds = j[kJLHSPointBinds].template get<std::vector<PointBindDataLHS>>();
	} else if(mBindMode == BindMode::BLEND) {
		mBlendNTBPointBinds = j[kJBlendNTBPointBinds].template get<std::vector<BlendedNTBData>>();
	}

	mPointSurfaceBinds = j[kJPointSurfaceBinds].template get<std::vector<PointSurfaceBindData>>();
	mSkinPrimIndices = j[kJSkinPrimIndices].template get<std::vector<int>>();

	if(j[kJDataHash].template get<size_t>() != calcHash()) {
		LOG_ERR << typeName() << " json data hash mismatch !";
		return false;
	}

	LOG_DBG << "GuideCurvesDeformerData data read from json payload !";

	mIsValid = true;
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