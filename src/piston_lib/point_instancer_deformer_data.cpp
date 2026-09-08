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

	for(const auto& bind: mPointBinds) {
		std::size_t raw_bits;
		double _tmp = static_cast<double>(bind.localPos[0] + bind.localPos[1] + bind.localPos[2]);

    	std::memcpy(&raw_bits, &_tmp, sizeof(double));
		hash += static_cast<size_t>(bind.edge_id) + raw_bits;
	}

	if(mBindMode == BindMode::SIMPLE) {
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
	from_string(j[kJMode].template get<std::string>(), bind_mode);

	if(bind_mode != mBindMode) {
		LOG_ERR << typeName() << " json data bind mode mismatch !";
		return false;
	}

	mPointBinds = j[kJPointBinds].template get<std::vector<PointBindData>>();
	if(j[kJDataHash].template get<size_t>() != calcHash()) {
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
	j = {bind.localPos[0], bind.localPos[1], bind.localPos[2], bind.edge_id};
}

void from_json(const json& j, PointInstancerDeformerData::PointBindData& bind) {
	
	assert(false && "PointInstancerDeformerData from_json NOT_IMPLEMENTED");

	bind.localPos[0] = j.at(0).template get<float>();
	bind.localPos[1] = j.at(1).template get<float>();
	bind.localPos[2] = j.at(2).template get<float>();
	bind.edge_id = j.at(4).template get<float>();
}

} // namespace Piston