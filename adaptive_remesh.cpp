#include "adaptive_remesh.hpp"

static void maximalIndependentSetOfSplittableEdges(ClothMesh &mesh, vector<ClothEdge *> &r_E)
{
  /* Due to the check mesh.exists(e) in splitEdges(), it is possible
   * to just return edges in decreasing order of their size */
  int edges_size = mesh.edges.size();
  vector<pair<ClothEdge *, double>> edge_with_size;
  for (int i = 0; i < edges_size; i++) {
    ClothEdge *e = static_cast<ClothEdge *>(mesh.edges[i]);

    double size = e->ClothAR_size();
    if (size > 1.0) {
      edge_with_size.push_back(make_pair(e, size));
    }
  }

  sort(edge_with_size.begin(),
       edge_with_size.end(),
       [](pair<ClothEdge *, double> a, pair<ClothEdge *, double> b) {
         return b.second < a.second;
       });
  int edge_with_size_size = edge_with_size.size();
  r_E.reserve(edge_with_size_size);
  for (int i = 0; i < edge_with_size_size; i++) {
    r_E.push_back(edge_with_size[i].first);
  }
}

static void getModifiedFaces(EditedElements &ee, vector<ClothFace *> &r_modified_faces)
{
  int added_faces_size = ee.added_faces.size();
  int r_modified_faces_size = r_modified_faces.size();
  r_modified_faces.reserve(r_modified_faces_size + added_faces_size);
  for (int i = 0; i < added_faces_size; i++) {
    r_modified_faces.push_back(static_cast<ClothFace *>(ee.added_faces[i]));
  }
}

static void ClothAR_flipEdges(vector<ClothFace *> modified_faces, EditedElements &r_ee)
{
  /* TODO(ish): flipEdges method */
}

static void setMeanParams(ClothNode *n0, ClothNode *n1, EditedElements &ee)
{
  int added_verts_size = ee.added_verts.size();
  for (int i = 0; i < added_verts_size; i++) {
    ClothVert *vnew = static_cast<ClothVert *>(ee.added_verts[i]);
    ClothVert *v0 = n0->adjacent(vnew);
    ClothVert *v1 = n1->adjacent(vnew);

    vnew->sizing = (v0->sizing + v1->sizing) * 0.5;
    ClothNode *nvnew = static_cast<ClothNode *>(vnew->node);
    ClothNode *nv0 = static_cast<ClothNode *>(v0->node);
    ClothNode *nv1 = static_cast<ClothNode *>(v1->node);
    nvnew->x0 = (nv0->x0 + nv1->x0) * 0.5;
    nvnew->v = (nv0->v + nv1->v) * 0.5;
    nvnew->mass = (nv0->mass + nv1->mass) * 0.5;
  }
}

static void ClothAR_splitEdges(ClothMesh &mesh)
{
  vector<ClothEdge *> E;
  do {
    E.clear();
    maximalIndependentSetOfSplittableEdges(mesh, E);
    for (int i = 0; i < E.size(); i++) {
      ClothEdge *e = E[i];
      assert(e != NULL);
      if (mesh.exists(e)) { /* TODO(ish): might be possible to
                             * optimize this */
        EditedElements ee;
        e->split(ee);
        ee.apply(mesh);
        setMeanParams(static_cast<ClothNode *>(e->n[0]), static_cast<ClothNode *>(e->n[1]), ee);
        for (int j = 0; j < ee.removed_edges.size(); j++) {
          exclude(static_cast<ClothEdge *>(ee.removed_edges[j]), E);
        }
        /* Need to get the modified faces during the splitting
         * operation so that they can be flipped if needed */
        vector<ClothFace *> modified_faces;
        getModifiedFaces(ee, modified_faces);
        /* Delete removed elements from memory */
        ee.deleteElements();

        /* Run flip edges on the modified faces */
        ClothAR_flipEdges(modified_faces, ee);
        ee.apply(mesh);
        ee.deleteElements();
      }
    }
  } while (E.size() > 0);
}

static bool collapsible(ClothEdge *e, int &r_remove_index)
{
  /* TODO(ish): check if the edges is collapsible or not */
  return false;
}

static void update(const EditedElements &ee, vector<ClothFace *> &r_F)
{
  int removed_faces_size = ee.removed_faces.size();
  int added_faces_size = ee.added_faces.size();
  for (int i = 0; i < removed_faces_size; i++) {
    exclude(static_cast<ClothFace *>(ee.removed_faces[i]), r_F);
  }
  for (int i = 0; i < added_faces_size; i++) {
    include(static_cast<ClothFace *>(ee.added_faces[i]), r_F);
  }
}

static vector<ClothFace *> deepCopy(const vector<Face *> &faces)
{
  vector<ClothFace *> r_faces;
  int faces_size = faces.size();
  r_faces.reserve(faces_size);
  for (int i = 0; i < faces_size; i++) {
    r_faces.push_back(static_cast<ClothFace *>(faces[i]));
  }
  return r_faces;
}

static void ClothAR_collapseEdges(ClothMesh &mesh)
{
  vector<ClothFace *> F = deepCopy(mesh.faces);
  while (F.size() > 0) {
    for (int i = 0; i < F.size(); i++) {
      ClothFace *f = F[i];

      bool no_collapse = true;
      for (int j = 0; j < 3; j++) {
        ClothEdge *e = static_cast<ClothEdge *>(f->adj_e[j]);
        int remove_index;

        if (collapsible(e, remove_index)) {
          EditedElements ee;
          e->collapse(remove_index, ee);
          ee.apply(mesh);

          /* Get modified faces for flip edges */
          vector<ClothFace *> modified_faces;
          getModifiedFaces(ee, modified_faces);
          /* Update F with ee */
          update(ee, F);
          ee.deleteElements();

          /* Run flip edges on the modified faces */
          ClothAR_flipEdges(modified_faces, ee);
          ee.apply(mesh);
          ee.deleteElements();

          no_collapse = false;
        }
      }

      if (no_collapse) {
        remove(i, F);
        /* i is decremented because remove(i, F) replaces F[i] with
         * F.back() and then F.pop_back() */
        i--;
      }
    }
  }
}

static void ClothAR_Remesh(ClothMesh &mesh)
{
  ClothAR_splitEdges(mesh);
  ClothAR_collapseEdges(mesh);
}

static void computeStaticVertSizing(ClothMesh &mesh, double min_edge_len)
{
  int verts_size = mesh.verts.size();
  for (int i = 0; i < verts_size; i++) {
    ClothVert *vert = static_cast<ClothVert *>(mesh.verts[i]);
    vert->sizing = Mat2x2(1.0 / sqr(min_edge_len));
  }
}

void ClothAR_StaticRemesh(ClothMesh &mesh)
{
  computeStaticVertSizing(mesh, 0.1);
  ClothAR_Remesh(mesh);
}
