#include "point_instancer_deformer_data.h"
#include "pxr_json.h"
#include "logging.h"



namespace Piston {

static const SerializableDeformerDataBase::DataVersion kPointInstancerDeformerDataVersion( 0u, 0u, 1u);

PointInstancerDeformerData::PointInstancerDeformerData(): mBindMode(BindMode::SIMPLE), mIsValid(false) {

};

void PointInstancerDeformerData::setBindMode(BindMode mode) {
	if(mBindMode == mode) return;
	mBindMode = mode;
	clear();
}

void PointInstancerDeformerData::clearData() {
	const std::lock_guard<std::mutex> lock(mMutex);

	mPointBinds.clear();
	mIsValid = false;
}

size_t PointInstancerDeformerData::calcHash() const {
	assert(false && "PointInstancerDeformerData calcHash NOT_IMPLEMENTED");

	size_t hash = 0;

	if(mBindMode == BindMode::SIMPLE) {
		for(const auto& bind: mPointBinds) {
			hash += bind.getHash();
		}
		hash += mPointBinds.size();
	} else {
		hash += mMPPPointBindings.size();
	}

	return hash;
}

static constexpr const char* kJPointBinds = "pointbinds";
static constexpr const char* kJMode = "mode";
static constexpr const char* kJDataHash = "data_hash";


bool PointInstancerDeformerData::dumpToJSON(json& j) const {
	assert(false && "PointInstancerDeformerData dumpToJSON NOT_IMPLEMENTED");

	const std::lock_guard<std::mutex> lock(mMutex);
	
	j[kJPointBinds] = mPointBinds;
	j[kJMode] = to_string(mBindMode);
	j[kJDataHash] = calcHash();

	return true;
}

bool PointInstancerDeformerData::readFromJSON(const json& j) {
	assert(false && "PointInstancerDeformerData readFromJSON NOT_IMPLEMENTED");

	const std::lock_guard<std::mutex> lock(mMutex);

	mIsValid = false;

	BindMode bind_mode = BindMode::SIMPLE;
	from_string(j[kJMode].get<std::string>(), bind_mode);

	if(bind_mode != mBindMode) {
		LOG_ERR << typeName() << " json data bind mode mismatch !";
		return false;
	}

	mPointBinds = j[kJPointBinds].get<std::vector<PointBindData>>();
	if(j[kJDataHash].get<size_t>() != calcHash()) {
		LOG_ERR << typeName() << " json data hash mismatch !";
		return false;
	}

	LOG_DBG << "PointInstancerDeformerData data read from json payload !";

	mIsValid = true;
	return true;
}


const std::string& PointInstancerDeformerData::typeName() const {
	static const std::string kTypeName = "PointInstancerDeformerData";
	return kTypeName;
}

const std::string& PointInstancerDeformerData::jsonDataKey() const {
	static const std::string kDataKey = "piston_point_instatncer_deformer_data";
	return kDataKey;
}

const SerializableDeformerDataBase::DataVersion& PointInstancerDeformerData::jsonDataVersion() const {
	return kPointInstancerDeformerDataVersion;
}

void to_json(json& j, const PointInstancerDeformerData::PointBindData& bind) {
	assert(false && "PointInstancerDeformerData to_json NOT_IMPLEMENTED");
	j = {
		bind.point_indices[0],
		bind.point_indices[1],
		bind.point_indices[2],
		bind.point_indices[3],
		bind.edge_id,
		bind.flags,
		bind.localPos, 
		bind.restNormal,
		bind.restTangent,
		bind.restBinormal,
		bind.u,
		bind.v
	};
}

void from_json(const json& j, PointInstancerDeformerData::PointBindData& bind) {
	assert(false && "PointInstancerDeformerData from_json NOT_IMPLEMENTED");

	bind.point_indices[0] = j.at(0).get<uint32_t>();
	bind.point_indices[1] = j.at(1).get<uint32_t>();
	bind.point_indices[2] = j.at(2).get<uint32_t>();
	bind.point_indices[3] = j.at(3).get<uint32_t>();

	bind.edge_id = j.at(4).get<int8_t>();
	bind.flags = static_cast<PointInstancerDeformerData::PointBindData::Flags>(j.at(5).get<uint8_t>());

	bind.localPos 	  = j.at(6).get<pxr::GfVec3f>();
	bind.restNormal   = j.at(7).get<pxr::GfVec3f>();
	bind.restTangent  = j.at(8).get<pxr::GfVec3f>();
	bind.restBinormal = j.at(9).get<pxr::GfVec3f>();

	bind.u = j.at(10).get<float>();
	bind.v = j.at(11).get<float>();
}

} // namespace Piston