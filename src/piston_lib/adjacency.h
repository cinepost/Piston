#ifndef PISTON_LIB_ADJACENCY_H_
#define PISTON_LIB_ADJACENCY_H_

#include "framework.h"

#include <memory>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/mesh.h>


namespace Piston {

class UsdGeomMeshFaceAdjacency {
	public:
		UsdGeomMeshFaceAdjacency();
		static UsdGeomMeshFaceAdjacency create(const pxr::UsdGeomMesh& mesh, pxr::UsdTimeCode timeCode = pxr::UsdTimeCode::Default());

		bool isValid() const;
		void invalidate();

		size_t getHash() const;

		std::string toString() const;

	private:
    	std::vector<uint32_t> mCounts;   	// per vertex neighbor faces counts
    	std::vector<uint32_t> mOffsets;  	// per vertex neighbor offsets in data array
    	std::vector<uint32_t> mFaceData;    // neighbor face indices
    	std::vector<uint32_t> mVertexData;	// neighbor vertex indices
    
    	std::vector<std::vector<uint32_t>> mPointToVerticesMap; // point index to vertices map

    	bool mValid; // Set by Adjacency data builder!
    
    	mutable size_t mHash;
};

inline std::string to_string(const UsdGeomMeshFaceAdjacency& a) {
	return a.toString();
}


} // namespace Piston

#endif // PISTON_LIB_ADJACENCY_H_