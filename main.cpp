#include <iostream>
#include <fstream>

#include "texture_mapping.h"
int main() {
    TextureMapping mapper;
    std::string skp_path = "box.skp";
    mapper.read_skp_mesh(skp_path);
    mapper.remesh();
    return 0;
}