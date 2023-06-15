/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editors
 */

#pragma once

#define MAX_CLIPPLANE_LEN 3
#define SCE_SNAP_MODE_POINTS \
  (SCE_SNAP_MODE_VERTEX | SCE_SNAP_MODE_EDGE_MIDPOINT | SCE_SNAP_MODE_EDGE_PERPENDICULAR)

struct SnapData_EditMesh;

struct SnapObjectContext {
  struct Scene *scene;

  blender::Map<const BMEditMesh *, std::unique_ptr<SnapData_EditMesh>> editmesh_caches;

  /* Filter data, returns true to check this value */
  struct {
    struct {
      bool (*test_vert_fn)(BMVert *, void *user_data);
      bool (*test_edge_fn)(BMEdge *, void *user_data);
      bool (*test_face_fn)(BMFace *, void *user_data);
      void *user_data;
    } edit_mesh;
  } callbacks;

  struct {
    Depsgraph *depsgraph;
    const RegionView3D *rv3d;
    const View3D *v3d;

    eSnapMode snap_to_flag;
    SnapObjectParams params;

    float ray_start[3];
    float ray_dir[3];
    float mval[2];

    float init_co[3];
    float curr_co[3];

    float win_size[2]; /* win x and y */
    float clip_plane[MAX_CLIPPLANE_LEN][4];
    int clip_plane_len;

    /* read/write */
    uint object_index;

    bool has_occlusion_plane; /* Ignore plane of occlusion in curves. */
    bool use_occlusion_test_edit;
  } runtime;

  /* Output. */

  struct {
    BVHTreeNearest nearest;

    /* Matrix of target object (may not be #Object.object_to_world with dupli-instances). */
    float obmat[4][4];
    /* Snapped object. */
    Object *ob;
    /* Snapped data. */
    const ID *data;

    eSnapMode elem;
  } poly, edge, point;

  /* List of #SnapObjectHitDepth (caller must free). */
  ListBase *hit_list;

  float dist_px_sq_to_edge_center;
};

struct RayCastAll_Data {
  void *bvhdata;

  /* internal vars for adding depths */
  BVHTree_RayCastCallback raycast_callback;

  const float (*obmat)[4];

  float len_diff;
  float local_scale;

  uint ob_uuid;

  /* output data */
  ListBase *hit_list;
};

struct Nearest2dUserData {
 public:
  /**
   * Constructor.
   */
  Nearest2dUserData(SnapObjectContext *sctx,
                    Object *ob_eval,
                    const ID *data_eval,
                    const float obmat[4][4] = nullptr,
                    bool skip_occlusion_plane = false);

  float dist_px_sq(void);
  float snap_dist_px_sq(const float co[3]);
  bool snap_boundbox(float min[3], float max[3]);
  bool snap_point(int index, const float co[3]);
  bool snap_edge(
      int edge_index, int v0_index, int v1_index, const float v0_co[3], const float v1_co[3]);
  bool confirm(SnapObjectContext *sctx);

  eSnapMode m_snap_to_flag;

  Object *m_ob;
  const ID *m_data;
  float m_obmat[4][4];

  DistProjectedAABBPrecalc m_nearest_precalc;
  float m_pmat_local[4][4];
  float m_clip_plane[MAX_CLIPPLANE_LEN][4];
  int m_clip_plane_len;

  float m_prev_co[3];
  float m_normal_fallback[3];

  bool m_use_backface_culling;

  void (*get_vert_co)(const int index, const Nearest2dUserData *data, const float **r_co);
  void (*get_edge_verts_index)(const int index, const Nearest2dUserData *data, int r_v_index[2]);
  void (*get_tri_verts_index)(const int index, const Nearest2dUserData *data, int r_v_index[3]);
  void (*get_tri_edges_index)(const int index, const Nearest2dUserData *data, int r_e_index[3]);
  void (*copy_vert_no)(const int index, const Nearest2dUserData *data, float r_no[3]);

  union {
    struct {
      BMesh *bm;
    };
    struct {
      const blender::float3 *vert_positions;
      const blender::float3 *vert_normals;
      const blender::int2 *edges;
      const int *corner_verts;
      const int *corner_edges;
      const MLoopTri *looptris;
    };
  };

 protected:
  BVHTreeNearest m_nearest_edge;
  BVHTreeNearest m_nearest_point;
  eSnapMode m_point_type;
  float m_dist_px_sq_to_edge_center;
  bool m_is_persp;
};

/* transform_snap_object.cc */

void raycast_all_cb(void *userdata, int index, const BVHTreeRay *ray, BVHTreeRayHit *hit);

bool raycast_tri_backface_culling_test(
    const float dir[3], const float v0[3], const float v1[3], const float v2[3], float no[3]);

void cb_snap_vert(void *userdata,
                  int index,
                  const DistProjectedAABBPrecalc *precalc,
                  const float (*clip_plane)[4],
                  const int clip_plane_len,
                  BVHTreeNearest *nearest);

void cb_snap_edge(void *userdata,
                  int index,
                  const DistProjectedAABBPrecalc *precalc,
                  const float (*clip_plane)[4],
                  const int clip_plane_len,
                  BVHTreeNearest *nearest);

bool nearest_world_tree(SnapObjectContext *sctx,
                        BVHTree *tree,
                        BVHTree_NearestPointCallback nearest_cb,
                        void *treedata,
                        const float (*obmat)[4],
                        float *r_dist_sq,
                        float *r_loc,
                        float *r_no,
                        int *r_index);

/* transform_snap_object_editmesh.cc */

struct SnapData_EditMesh {
  /* Verts, Edges. */
  BVHTree *bvhtree[2];
  bool cached[2];

  /* BVH tree from #BMEditMesh.looptris. */
  BVHTreeFromEditMesh treedata_editmesh;

  blender::bke::MeshRuntime *mesh_runtime;
  float min[3], max[3];

  void clear();

  ~SnapData_EditMesh()
  {
    this->clear();
  }

#ifdef WITH_CXX_GUARDEDALLOC
  MEM_CXX_CLASS_ALLOC_FUNCS("SnapData_EditMesh")
#endif
};

void snap_object_editmesh(
    SnapObjectContext *sctx, Object *ob_eval, ID *id, const float obmat[4][4], bool use_hide);

void snap_polygon_editmesh(
    SnapObjectContext *sctx, Object *ob_eval, const ID *id, const float obmat[4][4], int polygon);

/* transform_snap_object_mesh.cc */

void snap_object_mesh(
    SnapObjectContext *sctx, Object *ob_eval, ID *id, const float obmat[4][4], bool use_hide);

void snap_polygon_mesh(
    SnapObjectContext *sctx, Object *ob_eval, const ID *id, const float obmat[4][4], int polygon);
