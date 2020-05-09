#include "adaptive_remesh.hpp"

static void maximalIndependentSetOfSplittableEdges(ClothMesh &mesh, vector<ClothEdge *> &r_E)
{
  /* TODO(ish): get the maximal independent set of splittable edges */
}

static void getModifiedFaces(EditedElements &ee, vector<ClothFace *> &r_modified_faces)
{
  int added_faces_size = ee.added_faces.size();
  int r_modified_faces_size = r_modified_faces.size();
  r_modified_faces.reserve(r_modified_faces_size + added_faces_size);
  for (int i = 0; i < added_faces_size; i++) {
    r_modified_faces[r_modified_faces_size + i] = static_cast<ClothFace *>(ee.added_faces[i]);
  }
}

static void ClothAR_flipEdges(vector<ClothFace *> modified_faces, EditedElements &r_ee)
{
  /* TODO(ish): flipEdges method */
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
        /* TODO(ish): set sizing and other Cloth Parameters for the
         * newly created vert/node */
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
    r_faces[i] = static_cast<ClothFace *>(faces[i]);
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

void ClothAR_Remesh(ClothMesh &mesh)
{
  ClothAR_splitEdges(mesh);
  ClothAR_collapseEdges(mesh);
}
