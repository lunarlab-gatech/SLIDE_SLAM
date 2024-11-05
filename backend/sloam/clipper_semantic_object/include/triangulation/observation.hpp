// the code is adapted from https://github.com/gnardari/urquhart
#pragma once

#include <triangulation/distance.hpp>
#include <triangulation/polygon.hpp>
#include <map>

#include "libqhullcpp/RboxPoints.h"
#include "libqhullcpp/QhullError.h"
#include "libqhullcpp/QhullQh.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullFacetSet.h"
#include "libqhullcpp/QhullLinkedList.h"
#include "libqhullcpp/QhullPoint.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/QhullVertexSet.h"
#include "libqhullcpp/Qhull.h"

namespace DelaunayTriangulation{
    class Observation {
        public:
            explicit Observation(PointVector& landmarks);
            // Computes a Delaunay triangulation using QHull from a set of landmarks.
            void delaunayTriangulation(PointVector& points, std::vector<Polygon>& polygons);
            std::vector<DelaunayTriangulation::Polygon> triangles;
        private:
            PointVector landmarks;
            
            // // Uses the triangles of the delaunay triangulation to compute an urquhart tessellation
            // void urquhartTesselation_();
            
            // std::vector<EdgeT>::iterator findEdge_(Polygon& x, EdgeT commonEdge);
            // // Merges two polygons into a new one, this is used inside the urquhart tessellation computation
            // // p will be merged with n, which shares the longest edge of p, indexed (relative to p) by commonEdgeIdx
            // Polygon combinePolygonData_(const Polygon& p, const Polygon& n);
            // Polygon mergePolygons_(Polygon& p, Polygon& n, EdgeT commonEdge);
    };
}