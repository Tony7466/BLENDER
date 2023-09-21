/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

/**
 * variables on the UI for now
 * <pre>
 * float mediafrict;  friction to env
 * float nodemass;    softbody mass of *vertex*
 * float grav;        softbody amount of gravitation to apply
 *
 * float goalspring;  softbody goal springs
 * float goalfrict;   softbody goal springs friction
 * float mingoal;     quick limits for goal
 * float maxgoal;
 *
 * float inspring;    softbody inner springs
 * float infrict;     softbody inner springs friction
 * </pre>
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "CLG_log.h"

#include "MEM_guardedalloc.h"

/* types */
#include "DNA_collection_types.h"
#include "DNA_curve_types.h"
#include "DNA_lattice_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_force_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BLI_ghash.h"
#include "BLI_listbase.h"
#include "BLI_math_geom.h"
#include "BLI_math_matrix.h"
#include "BLI_math_vector.h"
#include "BLI_threads.h"
#include "BLI_utildefines.h"

#include "BKE_collection.h"
#include "BKE_collision.h"
#include "BKE_curve.h"
#include "BKE_deform.h"
#include "BKE_effect.h"
#include "BKE_global.h"
#include "BKE_layer.h"
#include "BKE_mesh.h"
#include "BKE_modifier.h"
#include "BKE_pointcache.h"
#include "BKE_scene.h"
#include "BKE_softbody.h"

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_query.h"

#include "PIL_time.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { SB_EDGE = 1, SB_BEND = 2, SB_STIFFQUAD = 3, SB_HANDLE = 4 } type_spring;
typedef struct BodySpring {
  int v1, v2;
  float len, cf, load;
  float ext_force[3]; /* edges colliding and sailing */
  type_spring springtype;
  short flag;
} BodySpring;
typedef struct BodyFace {
  int v1, v2, v3;
  float ext_force[3]; /* faces colliding */
  short flag;
} BodyFace;
typedef struct ReferenceVert {
  float pos[3]; /* position relative to com */
  float mass;   /* node mass */
} ReferenceVert;
typedef struct ReferenceState {
  float com[3];         /* Center of mass. */
  ReferenceVert *ivert; /* List of initial values. */
} ReferenceState;
typedef struct SBScratch {
  GHash *colliderhash;
  short needstobuildcollider;
  short flag;
  BodyFace *bodyface;
  int totface;
  float aabbmin[3], aabbmax[3];
  ReferenceState Ref;
} SBScratch;
typedef struct SB_thread_context {
  Scene *scene;
  Object *ob;
  float forcetime;
  float timenow;
  int ifirst;
  int ilast;
  ListBase *effectors;
  int do_deflector;
  float fieldfactor;
  float windfactor;
  int nr;
  int tot;
} SB_thread_context;
#define MID_PRESERVE 1
#define SOFTGOALSNAP 0.999f
#define HEUNWARNLIMIT 1      /* 500 would be fine i think for detecting severe *stiff* stuff */
#define BSF_INTERSECT 1      /* edge intersects collider face */
#define SBF_DOFUZZY 1        /* Body-point do fuzzy. */
#define SBF_OUTOFCOLLISION 2 /* Body-point does not collide. */
#define BFF_INTERSECT 1      /* collider edge   intrudes face. */
#define BFF_CLOSEVERT 2      /* collider vertex repulses face. */
float sb_grav_force_scale(Object *ob);
float sb_fric_force_scale(Object *ob);
float sb_time_scale(Object *ob);
float _final_goal(Object *ob, BodyPoint *bp);
float _final_mass(Object *ob, BodyPoint *bp);
typedef struct ccdf_minmax {
  float minx, miny, minz, maxx, maxy, maxz;
} ccdf_minmax;
typedef struct ccd_Mesh {
  int mvert_num, tri_num;
  const float (*vert_positions)[3];
  const float (*vert_positions_prev)[3];
  const MVertTri *tri;
  int safety;
  ccdf_minmax *mima;
  float bbmin[3];
  float bbmax[3];
} ccd_Mesh;
ccd_Mesh *ccd_mesh_make(Object *ob);
void ccd_mesh_update(Object *ob, ccd_Mesh *pccd_M);
void ccd_mesh_free(ccd_Mesh *ccdm);
void ccd_build_deflector_hash_single(GHash *hash, Object *ob);
void ccd_build_deflector_hash(Depsgraph *depsgraph,
                              Collection *collection,
                              Object *vertexowner,
                              GHash *hash);

void ccd_update_deflector_hash_single(GHash *hash, Object *ob);
void ccd_update_deflector_hash(Depsgraph *depsgraph,
                               Collection *collection,
                               Object *vertexowner,
                               GHash *hash);
int count_mesh_quads(Mesh *me);
void add_mesh_quad_diag_springs(Object *ob);
void add_2nd_order_roller(Object *ob, float stiffness, int *counter, int addsprings);
void add_2nd_order_springs(Object *ob, float stiffness);
void add_bp_springlist(BodyPoint *bp, int springID);
void build_bps_springlist(Object *ob);
void calculate_collision_balls(Object *ob);
void renew_softbody(Object *ob, int totpoint, int totspring);
void free_softbody_baked(SoftBody *sb);
void free_scratch(SoftBody *sb);
void free_softbody_intern(SoftBody *sb);
int query_external_colliders(Depsgraph *depsgraph, Collection *collection);
int sb_detect_aabb_collisionCached(float force[3], Object *vertexowner, float time);
int sb_detect_face_pointCached(const float face_v1[3],
                               const float face_v2[3],
                               const float face_v3[3],
                               float *damp,
                               float force[3],
                               Object *vertexowner,
                               float time);
int sb_detect_face_collisionCached(const float face_v1[3],
                                   const float face_v2[3],
                                   const float face_v3[3],
                                   float *damp,
                                   float force[3],
                                   Object *vertexowner,
                                   float time);
void scan_for_ext_face_forces(Object *ob, float timenow);
int sb_detect_edge_collisionCached(const float edge_v1[3],
                                   const float edge_v2[3],
                                   float *damp,
                                   float force[3],
                                   Object *vertexowner,
                                   float time);
void _scan_for_ext_spring_forces(
    Scene *scene, Object *ob, float timenow, int ifirst, int ilast, ListBase *effectors);
void *exec_scan_for_ext_spring_forces(void *data);
void sb_sfesf_threads_run(Depsgraph *depsgraph,
                          Scene *scene,
                          Object *ob,
                          float timenow,
                          int totsprings,
                          int *ptr_to_break_func(void));
int choose_winner(
    float *w, float *pos, float *a, float *b, float *c, float *ca, float *cb, float *cc);
int sb_detect_vertex_collisionCached(float opco[3],
                                     float facenormal[3],
                                     float *damp,
                                     float force[3],
                                     Object *vertexowner,
                                     float time,
                                     float vel[3],
                                     float *intrusion);
int sb_deflect_face(Object *ob,
                    float *actpos,
                    float *facenormal,
                    float *force,
                    float *cf,
                    float time,
                    float *vel,
                    float *intrusion);
void sb_spring_force(Object *ob, int bpi, BodySpring *bs, float iks, float forcetime);
int _softbody_calc_forces_slice_in_a_thread(Scene *scene,
                                            Object *ob,
                                            float forcetime,
                                            float timenow,
                                            int ifirst,
                                            int ilast,
                                            int *ptr_to_break_func(void),
                                            ListBase *effectors,
                                            int do_deflector,
                                            float fieldfactor,
                                            float windfactor);
void *exec_softbody_calc_forces(void *data);
void sb_cf_threads_run(Scene *scene,
                       Object *ob,
                       float forcetime,
                       float timenow,
                       int totpoint,
                       int *ptr_to_break_funcvoid(void),
                       ListBase *effectors,
                       int do_deflector,
                       float fieldfactor,
                       float windfactor);
void softbody_calc_forces(
    Depsgraph *depsgraph, Scene *scene, Object *ob, float forcetime, float timenow);
void softbody_apply_forces(Object *ob, float forcetime, int mode, float *err, int mid_flags);
void softbody_restore_prev_step(Object *ob);
void softbody_apply_goalsnap(Object *ob);
void apply_spring_memory(Object *ob);
void interpolate_exciter(Object *ob, int timescale, int time);
void springs_from_mesh(Object *ob);
void mesh_to_softbody(Object *ob);
void mesh_faces_to_scratch(Object *ob);
void reference_to_scratch(Object *ob);
float globallen(float *v1, float *v2, Object *ob);
void makelatticesprings(Lattice *lt, BodySpring *bs, int dostiff, Object *ob);
void lattice_to_softbody(Object *ob);
void curve_surf_to_softbody(Object *ob);
void softbody_to_object(Object *ob, float (*vertexCos)[3], int numVerts, int local);
void sb_new_scratch(SoftBody *sb);
bool object_has_edges(const Object *ob);
void softbody_update_positions(Object *ob, SoftBody *sb, float (*vertexCos)[3], int numVerts);
void softbody_reset(Object *ob, SoftBody *sb, float (*vertexCos)[3], int numVerts);
void softbody_step(Depsgraph *depsgraph, Scene *scene, Object *ob, SoftBody *sb, float dtime);
void sbStoreLastFrame(Depsgraph *depsgraph, Object *object, float framenr);

#ifdef __cplusplus
}
#endif
