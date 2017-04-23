//
// Created by han on 22/4/17.
//

#pragma once

#include "primitives.h"

#include <memory>

class TextureMapping {
public:
    TextureMapping() {}

    ~TextureMapping() {}

    void read_skp_mesh(const std::string &path);
    void remesh();

protected:
    void add_face_to_mesh(const CW::Face& su_face);
    std::vector<std::vector<vertex_idx_t>>
        triangulate_polygon_with_holes(const CW::Face& su_face);
    void debug_mesh(const std::string& path);
protected:
    std::shared_ptr<CW::Model> model_;
    std::vector<CW::Vertex> su_vertices_;
    std::vector<CW::Face> su_faces_;
    PlaneIdxMap su_face_idx_;
    VertexIdxMap su_vertex_idx_;

    std::vector<Point3d> mesh_vertices_;
    std::vector<Plane3d> planes_;

    SurfaceMesh mesh_;
    double target_length_ = 0.1 * 30;
};
