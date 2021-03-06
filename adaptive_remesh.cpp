#include "adaptive_remesh.hpp"

static int obj_num = 0;

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

static bool flippable(ClothEdge *e)
{
  if (e->isOnSeamOrBoundary()) {
    return false;
  }
  ClothVert *i = e->getVert(0, 0);
  if (i == NULL) {
    return false;
  }
  ClothVert *j = e->getVert(0, 1);
  if (j == NULL) {
    return false;
  }
  ClothVert *k = e->getOtherVertOfFace(0);
  if (k == NULL) {
    return false;
  }
  ClothVert *l = e->getOtherVertOfFace(1);
  if (l == NULL) {
    return false;
  }

  /* TODO(ish): This is not part of the paper "Adaptive Anisotropic
   * Remeshing for Cloth Simulation" but it seems necessary to not get triangles
   * with very small aspect ratios, it may be possible to remove this
   * after implementing collapse edges completely */
  if (k->ClothAR_size(l) > 1.0) {
    return false;
  }

  Vec2 ujk = j->uv - k->uv;
  Vec2 uik = i->uv - k->uv;
  Vec2 uil = i->uv - l->uv;
  Vec2 ujl = j->uv - l->uv;

  Mat2x2 Mavg = (i->sizing + j->sizing + k->sizing + l->sizing) * 0.25;
  if (wedge(ujk, uik) * dot(uil, Mavg * ujl) + dot(ujk, Mavg * uik) * wedge(uil, ujl) < 0.0) {
    return true;
  }
  return false;
}

static void maximalIndependentSetOfFlippableEdges(const vector<ClothEdge *> &E_dash,
                                                  vector<ClothEdge *> &r_E)
{
  vector<ClothEdge *> flippable_edges;
  int E_dash_size = E_dash.size();
  for (int i = 0; i < E_dash_size; i++) {
    ClothEdge *e = E_dash[i];

    if (flippable(e)) {
      flippable_edges.push_back(e);
    }
  }
  vector<ClothNode *> selected_nodes;
  int flippable_edges_size = flippable_edges.size();
  for (int i = 0; i < flippable_edges_size; i++) {
    ClothEdge *e = flippable_edges[i];
    ClothNode *n0 = static_cast<ClothNode *>(e->n[0]);
    ClothNode *n1 = static_cast<ClothNode *>(e->n[1]);
    if (!is_in(n0, selected_nodes) && !is_in(n1, selected_nodes)) {
      selected_nodes.push_back(n0);
      selected_nodes.push_back(n1);

      r_E.push_back(e);
    }
  }
}

static void updateFlippedFaces(EditedElements &ee, vector<ClothFace *> &r_F)
{
  int added_faces_size = ee.added_faces.size();
  int removed_faces_size = ee.removed_faces.size();
  for (int i = 0; i < added_faces_size; i++) {
    include(static_cast<ClothFace *>(ee.added_faces[i]), r_F);
  }
  for (int i = 0; i < removed_faces_size; i++) {
    exclude(static_cast<ClothFace *>(ee.removed_faces[i]), r_F);
  }
}

static bool inverted(const ClothFace *f)
{
  if (0.5 * wedge(f->v[1]->uv - f->v[0]->uv, f->v[2]->uv - f->v[0]->uv) <
      numeric_limits<double>::epsilon()) {
    return true;
  }
  return false;
}

static bool inverted(const EditedElements &ee)
{
  int added_faces_size = ee.added_faces.size();
  for (int i = 0; i < added_faces_size; i++) {
    ClothFace *f = static_cast<ClothFace *>(ee.added_faces[i]);

    if (inverted(f)) {
      return true;
    }
  }
  return false;
}

static void ClothAR_flipEdges(ClothMesh &mesh, vector<ClothFace *> &modified_faces)
{
  vector<ClothEdge *> E;
  int count = 0;
  static int func_count = 0;
  do {
    vector<ClothEdge *> E_dash;
    for (int i = 0; i < modified_faces.size(); i++) {
      ClothFace *f = modified_faces[i];
      for (int j = 0; j < 3; j++) {
        include(static_cast<ClothEdge *>(f->adj_e[j]), E_dash);
      }
    }
    E.clear();
    maximalIndependentSetOfFlippableEdges(E_dash, E);
#if ADAPTIVE_REMESHING_DEBUG
#  if ADAPTIVE_REMESHING_DEBUG_PRINT_FLIP
    cout << __func__ << " run " << func_count << " do_while: " << count << endl;
    cout << __func__ << " mesh.edges.size(): " << mesh.edges.size()
         << " E_dash.size(): " << E_dash.size() << " E.size(): " << E.size() << endl;
#  endif
#  if ADAPTIVE_REMESHING_DEBUG_SAVE_OBJ_FLIP
    char file[64];
    snprintf(file, 64, "temp/temp/temp_%04d.obj", obj_num++);
    mesh.saveObj(string(file));
    cout << __func__ << " saved " << obj_num - 1 << endl;
#  endif
#endif
    for (int i = 0; i < E.size(); i++) {
      ClothEdge *e = E[i];
      EditedElements ee;
      if (e->flip(ee)) {
        if (inverted(ee)) {
          ee.clear();
          remove(i, E);
          i--;
#if ADAPTIVE_REMESHING_DEBUG
#  if ADAPTIVE_REMESHING_DEBUG_PRINT_FLIP
          cout << __func__ << " continued due to inverted! current E.size(): " << E.size() << endl;
#  endif
#endif
          continue;
        }
        ee.apply(mesh);
        updateFlippedFaces(ee, modified_faces);
        ee.deleteElements();
      }
      else {
#if ADAPTIVE_REMESHING_DEBUG
#  if ADAPTIVE_REMESHING_DEBUG_PRINT_FLIP
        cout << __func__ << " couldn't flip predicted flip!" << endl;
#  endif
#endif
        remove(i, E);
        i--;
      }
    }
    count++;
  } while (E.size() > 0);
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
  int count = 30;
  int previous[count];
  for (int i = 0; i < count; i++) {
    previous[i] = -1;
  }
  int loop_count = 0;
  do {
    E.clear();
    maximalIndependentSetOfSplittableEdges(mesh, E);
    bool break_out = true;
    for (int i = 0; i < count; i++) {
      if (E.size() != previous[i]) {
        break_out = false;
      }
    }
    if (break_out) {
      mesh.saveObj("temp/temp.obj");
      cout << "warning: had to break out of " << __func__
           << " might have been an infinite loop otherwise!" << endl;
      return;
    }
    for (int i = count - 1; i > 0; i--) {
      previous[i] = previous[i - 1];
    }
    previous[0] = E.size();
#if ADAPTIVE_REMESHING_DEBUG
#  if ADAPTIVE_REMESHING_DEBUG_SAVE_OBJ_SPLIT
    {
      char file[64];
      snprintf(file, 64, "temp/temp/temp_%04d.obj", obj_num++);
      mesh.saveObj(string(file));
      cout << __func__ << " saved " << obj_num - 1 << endl;
    }
#  endif
#endif
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
#if ADAPTIVE_REMESHING_DEBUG
#  if ADAPTIVE_REMESHING_DEBUG_PRINT_SPLIT
        cout << __func__ << " do_while_count: " << loop_count << " for_loop_count: " << i << endl;
#  endif
#endif

        /* Run flip edges on the modified faces */
        ClothAR_flipEdges(mesh, modified_faces);
      }
    }
    loop_count++;
  } while (E.size() > 0);
}

static bool collapsibleAspectEdgeSizeCheck(ClothEdge *e, int remove_index)
{
  for (int i = 0; i < 2; i++) {
    ClothVert *v0 = e->getVert(i, remove_index);
    ClothVert *v1 = e->getVert(i, 1 - remove_index);
    /* v0 should exist and it shouldn't have be the same as the
     * previously removed v0 */
    if (!v0 || (i == 1 && v0 == e->getVert(0, remove_index))) {
      continue;
    }
    int adj_f_size = v0->adj_f.size();
    for (int i = 0; i < adj_f_size; i++) {
      const ClothFace *f = static_cast<ClothFace *>(v0->adj_f[i]);
      ClothVert *vs[3] = {static_cast<ClothVert *>(f->v[0]),
                          static_cast<ClothVert *>(f->v[1]),
                          static_cast<ClothVert *>(f->v[2])};
      if (is_in(v1, vs)) {
        continue;
      }
      replace(v0, v1, vs);
      double area = wedge(vs[1]->uv - vs[0]->uv, vs[2]->uv - vs[0]->uv) * 0.5;
      double perimeter = norm(vs[0]->uv - vs[1]->uv) + norm(vs[1]->uv - vs[2]->uv) +
                         norm(vs[2]->uv - vs[0]->uv);
      double aspect_ratio = 12.0 * sqrt(3) * area / sqr(perimeter);
      if (area < numeric_limits<double>::epsilon()) {
        return false;
      }
      /* TODO(ish): make aspect min part of remeshing parameters */
      double aspect_min = 1e-2;
      if (aspect_ratio < aspect_min) {
        return false;
      }
      for (int j = 0; j < 3; j++) {
        double h = 0.2;
        if (vs[j] != v1 && vs[NEXT(j)]->ClothAR_size(vs[PREV(j)]) > (1.0 - h)) {
          return false;
        }
      }
    }
  }
  return true;
}

static bool collapsible(ClothEdge *e, int &r_remove_index)
{
  for (r_remove_index = 0; r_remove_index < 2; r_remove_index++) {
    ClothNode *n0 = static_cast<ClothNode *>(e->n[r_remove_index]); /* Node that will be removed */
    ClothNode *n1 = static_cast<ClothNode *>(e->n[1 - r_remove_index]);

    /* The node that will be removed shouldn't along be on the seam
     * or boundary */
    if (n0->isOnSeamOrBoundary() && !e->isOnSeamOrBoundary()) {
      continue;
    }
    /* TODO(ish): Ensure that corners are not collapsed, but
     * this may or may not lead to problems, will need to test */
    if (collapsibleAspectEdgeSizeCheck(e, r_remove_index)) {
      return true;
    }
  }
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
#if ADAPTIVE_REMESHING_DEBUG
#  if ADAPTIVE_REMESHING_DEBUG_SAVE_OBJ_COLLAPSE
    {
      char file[64];
      snprintf(file, 64, "temp/temp/temp_%04d.obj", obj_num++);
      mesh.saveObj(string(file));
      cout << __func__ << " saved " << obj_num - 1 << endl;
    }
#  endif
#endif
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

#if ADAPTIVE_REMESHING_DEBUG
#  if ADAPTIVE_REMESHING_DEBUG_PRINT_COLLAPSE
          cout << "collapsed an edge!" << endl;
#  endif
#endif

          /* Run flip edges on the modified faces */
          ClothAR_flipEdges(mesh, modified_faces);
          no_collapse = false;
          break;
        }
      }

      if (no_collapse) {
        remove(i, F);
        /* i is decremented because remove(i, F) replaces F[i] with
         * F.back() and then F.pop_back() */
        i--;
      }
      else {
        /* This is to restart the for loop over F, some face might have changed */
        break;
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
