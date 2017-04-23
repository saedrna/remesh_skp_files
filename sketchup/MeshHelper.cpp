//
// Created by han on 23/4/17.
//
#include <cassert>
#include "MeshHelper.hpp"

namespace CW
{

MeshHelper::MeshHelper() {
    m_attacehd = false;


}

MeshHelper::MeshHelper(const Face &face, bool destroy_on_release) {
    SU_RESULT res = SUMeshHelperCreate(&m_mesh_helper, face.ref());
    assert(res == SU_ERROR_NONE);

    m_attacehd = !destroy_on_release;
}

MeshHelper::~MeshHelper() {
    if(!m_attacehd && SUIsInvalid(m_mesh_helper)){
        auto res = SUMeshHelperRelease(&m_mesh_helper);
        assert(res == SU_ERROR_NONE);
    }
}
}//end CW