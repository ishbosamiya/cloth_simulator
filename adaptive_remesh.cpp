#include "adaptive_remesh.hpp"

static void ClothAR_splitEdges(ClothMesh &mesh)
{
}

static void ClothAR_collapseEdges(ClothMesh &mesh)
{
}

void ClothAR_Remesh(ClothMesh &mesh)
{
  ClothAR_splitEdges(mesh);
  ClothAR_collapseEdges(mesh);
}
