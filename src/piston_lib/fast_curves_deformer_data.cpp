#include "fast_curves_deformer_data.h"

namespace Piston {

static const SerializableDeformerDataBase::DataVersion kFastBindingDataVersion( 0u, 0u, 0u);

void FastCurvesDeformerData::clearData() {
	mCurveBinds.clear();

	mRestVertexNormals.clear();
	mLiveVertexNormals.clear();

	mPerBindRestNormals.clear();
	mPerBindLiveNormals.clear();

	mPerBindRestTBs.clear();
	mPerBindLiveTBs.clear();
}

bool FastCurvesDeformerData::dumpToJSON(json& j) const {
	return true;
}

bool FastCurvesDeformerData::readFromJSON(const json& j) {
	return true;
}

const std::string& FastCurvesDeformerData::typeName() const {
	static const std::string kTypeName = "FastCurvesDeformerData";
	return kTypeName;
}

const std::string& FastCurvesDeformerData::jsonDataKey() const {
	static const std::string kDataKey = "_piston_deformer_fast_binding_data_";
	return kDataKey;
}

const SerializableDeformerDataBase::DataVersion& FastCurvesDeformerData::jsonDataVersion() const {
	return kFastBindingDataVersion;
}

} // namespace Piston