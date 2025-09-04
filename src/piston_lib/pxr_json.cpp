#include "pxr_json.h"

namespace Piston {

void to_json(json& j, const std::vector<std::pair<pxr::GfVec3f,pxr::GfVec3f>>& vec) {
	for(const auto& [a, b]: vec) {
    	j.push_back({(float)a[0], (float)a[1], (float)a[2], (float)b[0], (float)b[1], (float)b[2]});
    }
}

void from_json(const json& j, std::vector<std::pair<pxr::GfVec3f,pxr::GfVec3f>>& vec) {
	for (auto& e : j) {
 		vec.emplace_back(std::make_pair(
 			pxr::GfVec3f(
 				e.at(0).template get<float>(),
 				e.at(1).template get<float>(),
 				e.at(2).template get<float>()
 			),
 			pxr::GfVec3f(
 				e.at(3).template get<float>(),
 				e.at(4).template get<float>(),
 				e.at(5).template get<float>()
 			)
 		));
	}
}

void to_json(json& j, const std::vector<pxr::GfVec3f>& vec) {
	for(const auto& v: vec) {
    	j.push_back({(float)v[0], (float)v[1], (float)v[2]});
    }
}

void from_json(const json& j, std::vector<pxr::GfVec3f>& vec) {
	for (auto& e : j) {
 		vec.emplace_back(pxr::GfVec3f(
 			e.at(0).template get<float>(),
 			e.at(1).template get<float>(),
 			e.at(2).template get<float>()
 		));
	}
}

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
