#ifndef ADAPTIVE_REMESH_HPP
#define ADAPTIVE_REMESH_HPP

#include <iostream>
#include <vector>
#include <utility>
#include <string>
#include <cstdio>

#include "cloth_mesh.hpp"

using namespace std;

#define ADAPTIVE_REMESHING_DEBUG 0
#define ADAPTIVE_REMESHING_DEBUG_SAVE_OBJ_SPLIT 0
#define ADAPTIVE_REMESHING_DEBUG_SAVE_OBJ_FLIP 0
#define ADAPTIVE_REMESHING_DEBUG_SAVE_OBJ_COLLAPSE 0
#define ADAPTIVE_REMESHING_DEBUG_PRINT_SPLIT 0
#define ADAPTIVE_REMESHING_DEBUG_PRINT_FLIP 0
#define ADAPTIVE_REMESHING_DEBUG_PRINT_COLLAPSE 0

void ClothAR_StaticRemesh(ClothMesh &mesh);

#endif
