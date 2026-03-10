#ifndef PISTON_LIB_DEBUG_DRAWING_H_
#define PISTON_LIB_DEBUG_DRAWING_H_

#include "framework.h"
#include "common.h"
#include "adjacency.h"
#include "phantom_trimesh.h"


#include <memory>
#include <limits>
#include <string>
#include <mutex>


namespace Piston {

class DebugGeo {
	public:
		using UniquePtr = std::unique_ptr<DebugGeo>;

		struct Line {
			pxr::GfVec3f p0, p1;
			pxr::GfVec3f c0, c1;
			float w0, w1;

			Line(): p0(0.0f), p1(0.0f), c0(1.0f), c1(1.0f), w0(0.01f), w1(0.01f) {}
			Line(const pxr::GfVec3f& _p0, const pxr::GfVec3f& _p1): p0(_p0), p1(_p1) {}

			void setColor(const pxr::GfVec3f& c) { c0 = c; c1 = c; }
			void setColor(const pxr::GfVec3f& _c0, const pxr::GfVec3f& _c1) { c0 = _c0; c1 = _c1; }

			void setWidth(const float w) { w0 = w; w1 = w; }
			void setWidth(const float _w0, const float _w1) { w0 = _w0; w1 = _w1; }
		};

		void addLine(const Line& l) { mLines.push_back(l); }
		void clear();

		static UniquePtr create(const std::string& name);

		DebugGeo(const std::string& name): mName(name) {}
		bool build(const std::string& path, pxr::UsdStageRefPtr pStage);

	private:
		std::string mName;
		std::vector<Line> mLines;

		std::mutex  mMutex;

};

} // namespace Piston

#endif // PISTON_LIB_DEBUG_DRAWING_H_