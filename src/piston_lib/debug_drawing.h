#ifndef PISTON_LIB_DEBUG_DRAWING_H_
#define PISTON_LIB_DEBUG_DRAWING_H_

#include "framework.h"
#include "common.h"
#include "adjacency.h"
#include "phantom_trimesh.h"
#include "geometry_tools.h"


#include <memory>
#include <limits>
#include <string>
#include <mutex>


namespace Piston {

class DebugGeo {
	public:
		using UniquePtr = std::unique_ptr<DebugGeo>;

		struct Pt {
			pxr::GfVec3f pos;
			pxr::GfVec3f col;
			float w;

			Pt(): pos(0.0f), col(1.0f), w(0.01f) {}
			Pt(const pxr::GfVec3f& p): pos(p), col(1.0f), w(0.01f) {}
			Pt(const pxr::GfVec3f& p, const pxr::GfVec3f& _c, float _w): pos(p), col(_c), w(_w) {}

			void setColor(const pxr::GfVec3f& c) { col = c; }
			void setWidth(const float _w) { w = _w; }
		};

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

		struct WireframeBox {
			pxr::GfVec3f min, max;
			pxr::GfVec3f color;
			float width;

			WireframeBox(): min(0.0f), max(0.0f), color(1.0f), width(0.01f) {}
			WireframeBox(const AABB& aabb): min(aabb.min), max(aabb.max), color(1.0f), width(0.01f) {}

			void setColor(const pxr::GfVec3f& c) { color = c; }
			void setWidth(const float w) { width = w; }
		};

		struct WireTetra {
			pxr::GfVec3f p0, p1, p2, p3;
			pxr::GfVec3f color;
			float width;

			WireTetra(): p0(0.0f), p1(0.0f), p2(0.0f), p3(0.0f) {}
			WireTetra(const pxr::GfVec3f& _p0, const pxr::GfVec3f& _p1, const pxr::GfVec3f& _p2, const pxr::GfVec3f& _p3): p0(_p0), p1(_p1), p2(_p2), p3(_p3) {}

			void setColor(const pxr::GfVec3f& c) { color = c; }
			void setWidth(const float w) { width = w; }
		};

		void addPoint(const Pt& pt) { mPoints.push_back(pt); }
		void addLine(const Line& l) { mLines.push_back(l); }
		void addWireBox(const WireframeBox& b) { mWireBoxes.push_back(b); }
		void addWireTetra(const WireTetra& t) { mWireTetras.push_back(t); }
		void clear();

		static UniquePtr create(const std::string& name);

		DebugGeo(const std::string& name): mName(name) {}
		bool build(const std::string& path, pxr::UsdStageRefPtr pStage);

	private:
		std::string mName;
		std::vector<Pt> mPoints;
		std::vector<Line> mLines;
		std::vector<WireframeBox> mWireBoxes;
		std::vector<WireTetra> mWireTetras;

		std::mutex  mMutex;

};

} // namespace Piston

#endif // PISTON_LIB_DEBUG_DRAWING_H_