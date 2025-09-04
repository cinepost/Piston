#ifndef PISTON_LIB_PXR_JSON_H_
#define PISTON_LIB_PXR_JSON_H_

#include "framework.h"

#include <memory>
#include <string>
#include <vector>

#include <pxr/base/gf/vec3f.h>
#include <pxr/base/vt/array.h>

#include <nlohmann/json.hpp>


using json = nlohmann::json;

namespace Piston {

void to_json(json& j, const std::vector<std::pair<pxr::GfVec3f,pxr::GfVec3f>>& vec);
void from_json(const json& j, std::vector<std::pair<pxr::GfVec3f,pxr::GfVec3f>>& vec);

void to_json(json& j, const std::vector<pxr::GfVec3f>& vec);
void from_json(const json& j, std::vector<pxr::GfVec3f>& vec);

void to_json(json& j, const pxr::VtArray<pxr::GfVec3f>& vec);
void from_json(const json& j, pxr::VtArray<pxr::GfVec3f>& vec);

void to_json(json& j, const pxr::GfVec3f& p);
void from_json(const json& j, pxr::GfVec3f& p);

} // namespace Piston

#endif // PISTON_LIB_PXR_JSON_H_