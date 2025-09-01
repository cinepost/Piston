#include "pxr_json.h"

namespace Piston {

void to_json(json& j, const pxr::VtArray<pxr::GfVec3f>& vec) {
	j = json::array();
	
	for(const auto& v: vec) {
    	j.push_back({(float)v[0], (float)v[1], (float)v[2]});
    }	
}

void from_json(const json& j, pxr::VtArray<pxr::GfVec3f>& vec) {
    for (auto& e : j) {
 		vec.emplace_back(pxr::GfVec3f(
 			e.at(0).template get<float>(),
 			e.at(1).template get<float>(),
 			e.at(2).template get<float>()
 		));
	}
}

void to_json(json& j, const pxr::GfVec3f& p) {
	j =  json{(float)p[0], (float)p[1], (float)p[2]};
}

void from_json(const json& j, pxr::GfVec3f& p) {
	p[0] = j.at(0).template get<float>();
	p[1] = j.at(1).template get<float>();
	p[2] = j.at(2).template get<float>();
}

} // namespace Piston
