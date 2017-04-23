//
// Created by han on 23/4/17.
//

#pragma once

#include "Face.hpp"
#include <SketchupAPI/model/mesh_helper.h>

namespace CW {
class MeshHelper {
public:
    MeshHelper();
    MeshHelper(const Face& face, bool destroy_on_release = true);
    ~MeshHelper();

private:
    SUMeshHelperRef m_mesh_helper;
    bool m_attacehd;
};
}


