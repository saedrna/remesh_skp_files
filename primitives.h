#pragma once

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <unordered_map>

#include <sketchup/Sketchup.hpp>

template <typename T>
struct SketchupHasher{
    size_t operator()(const T& k) const{
        return std::hash<void *>()(k.ref().ptr);
    }
};

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Plane_3 Plane3d;
typedef Kernel::Point_3 Point3d;
typedef Kernel::Point_2 Point2d;

typedef CGAL::Surface_mesh<Kernel::Point_3> SurfaceMesh;
typedef SurfaceMesh::Vertex_index vertex_t;
typedef SurfaceMesh::Edge_index edge_t;
typedef SurfaceMesh::Face_index face_t;
typedef SurfaceMesh::halfedge_index hedge_t;

typedef uint32_t plane_idx_t;
typedef uint32_t vertex_idx_t;

///\brief 一个plane可以对应多个face_t
typedef std::unordered_map<face_t, plane_idx_t > FacePlaneMap;

///\brief 一个su vertex只能对应一个mesh vertex
typedef std::unordered_map<CW::Vertex, vertex_idx_t, SketchupHasher<CW::Vertex>> VertexIdxMap;

typedef std::unordered_map<CW::Face, plane_idx_t, SketchupHasher<CW::Face>> PlaneIdxMap;
