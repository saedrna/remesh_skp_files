//
// Created by han on 22/4/17.
//

#include "texture_mapping.h"

#include <fstream>

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_id_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/barycenter.h>
#include <cgal/Polygon_mesh_processing/remesh.h>
#include <cgal/Polygon_mesh_processing/triangulate_faces.h>

void TextureMapping::read_skp_mesh(const std::string &path) {
    using namespace CW;
    model_ = std::make_shared<CW::Model>(path);

    su_faces_ = model_->entities().faces();

    int id_vertex = 0;
    int id_face = 0;
    for(const auto &face : su_faces_) {
        su_face_idx_.insert({face, id_face++});
        auto plane = face.plane();
        planes_.emplace_back(plane.a, plane.b, plane.c, plane.d);

        auto vertices = face.vertices();
        for(const auto &vert : vertices) {
            if(su_vertex_idx_.count(vert) == 0) {
                su_vertex_idx_.insert({vert, id_vertex++});
                su_vertices_.push_back(vert);

                Point3d mesh_vertex = {vert.position().x, vert.position().y, vert.position().z};
                mesh_vertices_.push_back(mesh_vertex);
                mesh_.add_vertex(mesh_vertex);
            }
        }
    }

    for(const auto &face : su_faces_) {
        add_face_to_mesh(face);
    }
    auto edges = model_->entities().edges(false);
    debug_mesh("mesh.off");
}

void TextureMapping::add_face_to_mesh(const CW::Face &face) {
    int num_holes = face.inner_loops().size();
    if(num_holes == 0) {
        auto vertices = face.outer_loop().vertices();
        std::vector<vertex_t> mesh_vertices;
        for(const auto &vert: vertices) {
            mesh_vertices.emplace_back(su_vertex_idx_[vert]);
        }
        mesh_.add_face(mesh_vertices);
    } else {
        auto triangles = triangulate_polygon_with_holes(face);
        for(const auto &triangle : triangles) {
            std::vector<vertex_t> tri = {
                vertex_t(triangle[0]), vertex_t(triangle[1]), vertex_t(triangle[2])
            };
            mesh_.add_face(tri);
        }
    }
}

void TextureMapping::debug_mesh(const std::string &path) {
    std::ofstream ofile(path);
    ofile << mesh_;
}

std::vector<std::vector<vertex_idx_t>> TextureMapping::triangulate_polygon_with_holes(const CW::Face &su_face) {
    std::vector<std::vector<vertex_idx_t>> mesh_facets;

    int id_face = su_face_idx_[su_face];
    Plane3d plane = planes_[id_face];

    std::vector<vertex_idx_t> global_vertex_id;
    std::vector<Point2d> point2d;

    auto su_vertices = su_face.vertices();
    for(const auto &vert : su_vertices) {
        Point3d point3d = {vert.position().x, vert.position().y, vert.position().z};
        global_vertex_id.push_back(su_vertex_idx_[vert]);
        point2d.push_back(plane.to_2d(point3d));
    }

    typedef CGAL::Triangulation_vertex_base_with_id_2<Kernel> Tvb;
    typedef CGAL::Triangulation_data_structure_2<Tvb> Tds;
    typedef CGAL::Delaunay_triangulation_2<Kernel, Tds> Delaunay;
    typedef CGAL::Polygon_with_holes_2<Kernel> PolygonWithHoles;
    typedef PolygonWithHoles::General_polygon_2 PolygonGenral;
    typedef CGAL::Polygon_2<Kernel> Polygon2d;

    PolygonWithHoles polygon2d;
    {
        auto loops = su_face.loops();
        for(int i = 0; i < loops.size(); ++i) {
            std::vector<Point2d> loop_points;
            for(const auto &vert : loops[i].vertices()) {
                Point3d point3d = {vert.position().x, vert.position().y, vert.position().z};
                loop_points.push_back(plane.to_2d(point3d));
            }
            Polygon2d polygon(begin(loop_points), end(loop_points));
            if(i == 0) {
                if(polygon.is_clockwise_oriented()) {
                    polygon.reverse_orientation();
                }
                polygon2d.outer_boundary() = polygon;
            } else {
                if(polygon.is_counterclockwise_oriented()) {
                    polygon.reverse_orientation();
                }
                polygon2d.add_hole(polygon);
            }
        }
    }
    Delaunay delaunay;
    for(int i = 0; i < point2d.size(); ++i) {
        int id_vert = global_vertex_id[i];
        auto vh = delaunay.insert(point2d[i]);
        vh->id() = id_vert;
    }

    for(auto tri = delaunay.faces_begin(); tri != delaunay.faces_end(); ++tri) {

        Point2d barycenter = CGAL::barycenter(
            tri->vertex(0)->point(), 1.0,
            tri->vertex(1)->point(), 1.0,
            tri->vertex(2)->point(), 1.0
        );
        bool in_polygon = true;
        {
            if(!(polygon2d.outer_boundary().bounded_side(barycenter) == CGAL::ON_BOUNDED_SIDE)) {
                in_polygon = false;
            }
            for(auto hole = polygon2d.holes_begin(); hole != polygon2d.holes_end(); ++hole) {
                if(hole->bounded_side(barycenter) == CGAL::ON_BOUNDED_SIDE) {
                    in_polygon = false;
                }
            }
        }

        if(in_polygon) {
            mesh_facets.push_back({(vertex_idx_t) tri->vertex(0)->id(),
                                   (vertex_idx_t) tri->vertex(1)->id(),
                                   (vertex_idx_t) tri->vertex(2)->id()});
        }
    }
    return mesh_facets;
}

void TextureMapping::remesh() {
    namespace PMP = CGAL::Polygon_mesh_processing;
    auto constrained_edges = mesh_.add_property_map<edge_t, bool>("e:is_constrained", false).first;
    auto edges = model_->entities().edges(false);

    std::set<std::pair<vertex_idx_t, vertex_idx_t>> origin_edges;
    for(const auto &edge : edges) {
        vertex_idx_t id_start, id_end;
        auto vertex_start = edge.start();
        auto vertex_end = edge.end();
        id_start = su_vertex_idx_[edge.start()];
        id_end = su_vertex_idx_[edge.end()];
        if(id_start < id_end)
            origin_edges.insert({id_start, id_end});
        else
            origin_edges.insert({id_end, id_start});
    }

    for(const auto &edge : CGAL::edges(mesh_)) {
        vertex_idx_t id_start = mesh_.vertex(edge, 0);
        vertex_idx_t id_end = mesh_.vertex(edge, 1);

        auto edge_pair = std::make_pair(std::min(id_start, id_end), std::max(id_start, id_end));
        if(origin_edges.count(edge_pair))
            constrained_edges[edge] = true;
    }

    PMP::triangulate_faces(mesh_);
    debug_mesh("triangulate.off");

    PMP::isotropic_remeshing(
        faces(mesh_),
        3.0,
        mesh_,
        PMP::parameters::number_of_iterations(1).edge_is_constrained_map(constrained_edges)
    );

    debug_mesh("remesh.off");
}
