/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bmesh
 *
 * Cut meshes along intersections.
 *
 * Boolean-like modeling operation (without calculating inside/outside).
 *
 * Supported:
 * - Concave faces.
 * - Non-planar faces.
 * - Custom-data (UVs etc).
 *
 * Unsupported:
 * - Intersecting between different meshes.
 * - No support for holes (cutting a hole into a single face).
 */

#include "MEM_guardedalloc.h"

#include "BLI_alloca.h"
#include "BLI_buffer.h"
#include "BLI_edgehash.h"
#include "BLI_kdopbvh.h"
#include "BLI_linklist.h"
#include "BLI_math.h"
#include "BLI_memarena.h"
#include "BLI_sort.h"
#include "BLI_sort_utils.h"

#include "bmesh.h"

#include "bmesh_intersect.h" /* own include */

#include "tools/bmesh_edgesplit.h"

#define RAYCAST_DEPTH_EPSILON (10 * FLT_EPSILON)

#ifndef NDEBUG
#  define DEBUG_BOOLEAN
#endif

#define VERT_DISSOLVE_FLAG BM_ELEM_TAG_ALT
#define EDGE_ISECT_FLAG BM_ELEM_TAG

/* remove verts created by intersecting triangles */
#define USE_DISSOLVE

#ifdef DEBUG_BOOLEAN
static void debug_ghash_free_key_fn(void *key)
{
  printf("Removed Ghash Intersect Face Key: %p\n", key);
}
#else
static void *debug_ghash_free_key_fn = NULL;
#endif

typedef enum {
  VERT,
  EDGE,
  TRI,
  NONE,
} ISectElemType;

typedef struct IsectPair {
  void *elem_a;
  void *elem_b;
  short type_a;
  short type_b;
  union {
    struct {
      float lambda_a;
      float lambda_b;
      uint sort_vote; /* Unused. */
    };
    float co[3];
  };
} IsectPair;

typedef struct IsectEdge {
  union {
    struct {
      IsectPair *pair_a;
      IsectPair *pair_b;
    };
    BMEdge *edge;
    void *next;
  };
} IsectEdge;

typedef struct ISectState {
  BMesh *bm;

  /* Point to an #IsectPair using an array of indices as a key. */
  EdgeHash *vert_vert;
  GHash *iso_vert;
  GHash *vert_edge;
  GHash *edge_tri;
  GHash *edge_edge;

  /* BMFace-index: LinkList(of edges), only original faces */
  GSet *isect_edges;
  GHash *face_links;

  MemArena *mem_arena;
  LinkNode *removed_nodes;
  // LinkNode *removed_isect_edges;

  float eps;
  float eps_sq;
#ifdef DEBUG_BOOLEAN
  float eps_cube;
#endif
} ISectState;

/* Stitch hash initialization functions */
static uint inthash_v2_p(const void *key)
{
  const int *v = key;
  return (BLI_ghashutil_uinthash(v[0]) + BLI_ghashutil_uinthash(v[1]));
}

static bool inthash_v2_cmp(const void *a, const void *b)
{
  return (memcmp(a, b, sizeof(int[2])) != 0);
}

static uint inthash_v3_p(const void *key)
{
  const int *v = key;
  return (inthash_v2_p(v) + BLI_ghashutil_uinthash(v[2]));
}

static bool inthash_v3_cmp(const void *a, const void *b)
{
  return (memcmp(a, b, sizeof(int[3])) != 0);
}

static uint isect_edge_hash_p(const void *key)
{
  const IsectEdge *isect_edge = key;
  uint k0 = (uint)(intptr_t)isect_edge->pair_a;
  uint k1 = (uint)(intptr_t)isect_edge->pair_b;
  BLI_assert(k0 <= k1);

  return (k0 << 8) ^ k1;
}

static bool isect_edge_hash_cmp(const void *a, const void *b)
{
  return (memcmp(a, b, sizeof(IsectPair *) * 2) != 0);
}

static void isect_state_init(ISectState *r_s, BMesh *bm, const float eps)
{
  r_s->bm = bm;

  r_s->vert_vert = BLI_edgehash_new(__func__);
  r_s->iso_vert = BLI_ghash_int_new(__func__);
  r_s->vert_edge = BLI_ghash_new(inthash_v2_p, inthash_v2_cmp, __func__);
  r_s->edge_tri = BLI_ghash_new(inthash_v3_p, inthash_v3_cmp, __func__);
  r_s->edge_edge = BLI_ghash_new(
      BLI_ghashutil_inthash_v4_p, BLI_ghashutil_inthash_v4_cmp, __func__);

  r_s->isect_edges = BLI_gset_new(isect_edge_hash_p, isect_edge_hash_cmp, __func__);
  r_s->face_links = BLI_ghash_ptr_new(__func__);

  r_s->mem_arena = BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE, __func__);
  r_s->removed_nodes = NULL;

  /* setup epsilon from base */
  r_s->eps = eps;
  r_s->eps_sq = eps * eps;
#ifdef DEBUG_BOOLEAN
  r_s->eps_cube = r_s->eps_sq * eps;
#endif
}

static void isect_state_free(ISectState *s)
{
  /* cleanup */
  BLI_edgehash_free(s->vert_vert, NULL);
  BLI_ghash_free(s->iso_vert, NULL, NULL);
  BLI_ghash_free(s->vert_edge, NULL, NULL);
  BLI_ghash_free(s->edge_tri, NULL, NULL);
  BLI_ghash_free(s->edge_edge, NULL, NULL);

  BLI_gset_free(s->isect_edges, NULL);
  BLI_ghash_free(s->face_links, NULL, NULL);

  BLI_memarena_free(s->mem_arena);
}

static bool ghash_ensure_p_alloc_key(
    GHash *gh, const void *key, const size_t key_size, struct MemArena *ma, void ***r_val)
{
  void **r_key;
  if (!BLI_ghash_ensure_p_ex(gh, key, &r_key, r_val)) {
    *r_key = BLI_memarena_alloc(ma, key_size);
    memcpy(*r_key, key, key_size);
    return false;
  }
  return true;
}

static IsectPair *iso_vert_ensure(ISectState *s, BMVert *v)
{
  void *v_index_key = POINTER_FROM_INT(BM_elem_index_get(v));
  void **val_ptr;

  if (!BLI_ghash_ensure_p(s->iso_vert, v_index_key, &val_ptr)) {
    IsectPair *pair = *val_ptr = BLI_memarena_alloc(s->mem_arena, sizeof(*pair));
    pair->elem_a = v;
    pair->type_a = VERT;
    pair->type_b = NONE;
  }

  return (IsectPair *)*val_ptr;
}

static IsectPair *vert_vert_pair_ensure(ISectState *s, BMVert *va, BMVert *vb)
{
  int indexes[2] = {BM_elem_index_get(va), BM_elem_index_get(vb)};

  void **val_ptr;
  if (!BLI_edgehash_ensure_p(s->vert_vert, UNPACK2(indexes), &val_ptr)) {
    IsectPair *pair = *val_ptr = BLI_memarena_alloc(s->mem_arena, sizeof(*pair));
    pair->elem_a = va;
    pair->elem_b = vb;
    pair->type_a = VERT;
    pair->type_b = VERT;
  }

  return (IsectPair *)*val_ptr;
}

static IsectPair *vert_edge_pair_ensure(
    ISectState *s, BMVert *v, BMVert *ev_a, BMVert *ev_b, const float fac)
{
  BMEdge *e = BM_edge_exists(ev_a, ev_b);
  if (e) {
    int indexes[2] = {BM_elem_index_get(v), BM_elem_index_get(e)};

    void **val_ptr;
    if (!ghash_ensure_p_alloc_key(
            s->vert_edge, indexes, sizeof(indexes), s->mem_arena, &val_ptr)) {
      IsectPair *pair = *val_ptr = BLI_memarena_alloc(s->mem_arena, sizeof(*pair));
      pair->elem_a = v;
      pair->type_a = VERT;
      pair->elem_b = e;
      pair->type_b = EDGE;
      pair->lambda_b = e->v1 == ev_a ? fac : 1.0f - fac;
    }

    return (IsectPair *)*val_ptr;
  }
  else {
    return iso_vert_ensure(s, v);
  }
}

static IsectPair *vert_tri_pair_ensure(ISectState *s,
                                       BMVert *v,
                                       const int UNUSED(loop_tri),
                                       const float UNUSED(co[3]))
{
  return iso_vert_ensure(s, v);
}

static IsectPair *edge_edge_pair_ensure(ISectState *s,
                                        BMVert *ea_va,
                                        BMVert *ea_vb,
                                        BMVert *eb_va,
                                        BMVert *eb_vb,
                                        const float fac_a,
                                        const float fac_b,
                                        const float co_a[3],
                                        const float co_b[3])
{
  int indexes[4] = {
      BM_elem_index_get(ea_va),
      BM_elem_index_get(ea_vb),
      BM_elem_index_get(eb_va),
      BM_elem_index_get(eb_vb),
  };

  if (indexes[0] > indexes[1]) {
    SWAP(int, indexes[0], indexes[1]);
  }

  if (indexes[2] > indexes[3]) {
    SWAP(int, indexes[2], indexes[3]);
  }

  if (indexes[0] > indexes[2]) {
    SWAP(int, indexes[0], indexes[2]);
    SWAP(int, indexes[1], indexes[3]);
  }

  void **val_ptr;
  if (!ghash_ensure_p_alloc_key(s->edge_edge, indexes, sizeof(indexes), s->mem_arena, &val_ptr)) {
    IsectPair *pair = *val_ptr = BLI_memarena_alloc(s->mem_arena, sizeof(*pair));
    BMEdge *ea = BM_edge_exists(ea_va, ea_vb);
    BMEdge *eb = BM_edge_exists(eb_va, eb_vb);
    if (ea) {
      pair->elem_a = ea;
      pair->type_a = EDGE;
      pair->lambda_a = ea->v1 == ea_va ? fac_a : 1.0f - fac_a;
    }
    else {
      pair->elem_a = pair->co;
      pair->type_a = NONE;
    }

    if (eb) {
      pair->elem_b = eb;
      pair->type_b = EDGE;
      pair->lambda_b = eb->v1 == eb_va ? fac_b : 1.0f - fac_b;
    }
    else {
      pair->elem_b = pair->co;
      pair->type_b = NONE;
    }

    if (!ea && !eb) {
      mid_v3_v3v3(pair->co, co_a, co_b);
    }
  }

  return (IsectPair *)*val_ptr;
}

static IsectPair *edge_tri_pair_ensure(
    ISectState *s, BMVert *ev_a, BMVert *ev_b, const int loop_tri, const float fac)
{
  int indexes[3] = {BM_elem_index_get(ev_a), BM_elem_index_get(ev_b), loop_tri};
  if (indexes[0] > indexes[1]) {
    SWAP(int, indexes[0], indexes[1]);
  }
  void **val_ptr;
  if (!ghash_ensure_p_alloc_key(s->edge_tri, indexes, sizeof(indexes), s->mem_arena, &val_ptr)) {
    IsectPair *pair = *val_ptr = BLI_memarena_alloc(s->mem_arena, sizeof(*pair));
    BMEdge *e = BM_edge_exists(ev_a, ev_b);
    if (e) {
      pair->elem_a = e;
      pair->type_a = EDGE;
      pair->lambda_a = e->v1 == ev_a ? fac : 1.0f - fac;
    }
    else {
      pair->elem_a = pair->co;
      pair->type_a = NONE;
      interp_v3_v3v3(pair->co, ev_a->co, ev_b->co, fac);
    }

    pair->elem_b = pair->co;
    pair->type_b = NONE;
  }

  return (IsectPair *)*val_ptr;
}

/**
 * Store as value in GHash so we can get list-length without counting every time.
 * Also means we don't need to update the GHash value each time.
 */
typedef struct FaceLinkLists {
  LinkNode *edge_list;          /* #IsectEdge first, #BMEdge after. */
  LinkNode *face_coplanar_list; /* #BMFace. */
  uint edge_list_len;
  bool is_face_a;
} FaceLinkLists;

static FaceLinkLists *ghash_face_link_lists_ensure(GHash *gh,
                                                   BMFace *f,
                                                   MemArena *mem_arena,
                                                   const bool is_face_a)
{
  void **f_lists_p;

  if (!BLI_ghash_ensure_p(gh, f, &f_lists_p)) {
    FaceLinkLists *f_lists;
    f_lists = *f_lists_p = BLI_memarena_calloc(mem_arena, sizeof(*f_lists));
    f_lists->is_face_a = is_face_a;
  }

  return (FaceLinkLists *)*f_lists_p;
}

static LinkNode *isect_node_alloc(ISectState *s)
{
  LinkNode *node = s->removed_nodes;
  if (node) {
    s->removed_nodes = node->next;
  }
  else {
    node = BLI_memarena_alloc(s->mem_arena, sizeof(*node));
  }
  return node;
}

static void isect_node_remove(ISectState *s, LinkNode **node_ptr, LinkNode *node)
{
  *node_ptr = node->next;
  node->next = s->removed_nodes;
  s->removed_nodes = node;
}

static IsectEdge *isect_edge_alloc(struct ISectState *s)
{
  IsectEdge *isect_edge = BLI_memarena_alloc(s->mem_arena, sizeof(*isect_edge));
  return isect_edge;
}

static IsectEdge *isect_edge_ensure(struct ISectState *s, IsectPair *pair_a, IsectPair *pair_b)
{
  BLI_assert(pair_a != pair_b);
  if ((uint)(intptr_t)pair_a > (uint)(intptr_t)pair_b) {
    SWAP(IsectPair *, pair_a, pair_b);
  }

  IsectEdge isect_edge_stack = {.pair_a = pair_a, .pair_b = pair_b};
  void **key_ptr;
  if (!BLI_gset_ensure_p_ex(s->isect_edges, &isect_edge_stack, &key_ptr)) {
    IsectEdge *isect_edge = *key_ptr = isect_edge_alloc(s);
    *isect_edge = isect_edge_stack;
  }

  return (IsectEdge *)*key_ptr;
}

static void face_isect_edge_add(ISectState *s,
                                BMFace *f,
                                IsectEdge *isect_edge,
                                const bool is_face_a)
{
  FaceLinkLists *f_lists = ghash_face_link_lists_ensure(s->face_links, f, s->mem_arena, is_face_a);
  if (BLI_linklist_index(f_lists->edge_list, isect_edge) == -1) {
    LinkNode *node = isect_node_alloc(s);
    node->next = f_lists->edge_list;
    node->link = isect_edge;
    f_lists->edge_list = node;
    f_lists->edge_list_len++;
  }
}

static void face_coplanar_add(ISectState *s, BMFace *f, BMFace *f_coplanar, const bool is_face_a)
{
  FaceLinkLists *f_lists = ghash_face_link_lists_ensure(s->face_links, f, s->mem_arena, is_face_a);

  if (BLI_linklist_index(f_lists->face_coplanar_list, f_coplanar) == -1) {
    // BLI_assert(compare_ff(fabsf(dot_v3v3(f->no, f_coplanar->no)), 1.0f, s->eps));
    LinkNode *node = isect_node_alloc(s);
    node->next = f_lists->face_coplanar_list;
    node->link = f_coplanar;
    f_lists->face_coplanar_list = node;
  }
}

static bool is_tri_degenerate(ISectState *s, const float tri[3][3])
{
  float normal[3];
  cross_tri_v3(normal, UNPACK3(tri));
  if (len_squared_v3(normal) <= s->eps_sq) {
    return true;
  }

  return false;
}

#ifdef DEBUG_BOOLEAN
static bool is_coplanar(ISectState *s, const float tri_a[3][3], const float tri_b[3][3])
{
#  if 0
  float normal_a[3], normal_b[3];
  cross_tri_v3(normal_a, UNPACK3(tri_a));
  cross_tri_v3(normal_b, UNPACK3(tri_b));

  float cross[3];
  cross_v3_v3v3(cross, normal_a, normal_b);

  return len_v3(cross) < s->eps_cube;
#  else
  UNUSED_VARS(s, tri_a, tri_b);
  return true;
#  endif
}
#endif

typedef enum {
  NO_ISECT = 0,
  ISECT,
  COPLANAR,
  DEGENERATED,
} IsectTriTriMode;
static IsectTriTriMode isect_tri_tri_impl(ISectState *s,
                                          const float tri_a[3][3],
                                          const float tri_b[3][3],
                                          BMVert *fv_a[3],
                                          BMVert *fv_b[3],
                                          int a_index,
                                          int b_index,
                                          BMFace *f_a,
                                          BMFace *f_b)
{
  struct {
    uint i;
    ISectElemType type;
    float lambda;
    float ix[3];
  } isect_elem[3][2];
  uint isect_elem_len = 0;

  bool visit_vert_a[3] = {false, false, false};
  bool visit_vert_b[3] = {false, false, false};
  int vert_a_vert_b[3] = {-1, -1, -1};
  int vert_b_vert_a[3] = {-1, -1, -1};
  int vert_a_edge_b[3] = {-1, -1, -1};
  int vert_b_edge_a[3] = {-1, -1, -1};

  /* Vert x Vert. */
  for (uint i_a = 0; i_a < 3; i_a++) {
    for (uint i_b = 0; i_b < 3; i_b++) {
      if (vert_b_vert_a[i_b] != -1) {
        continue;
      }
      const float dist_sq = len_squared_v3v3(tri_a[i_a], tri_b[i_b]);
      if (dist_sq <= s->eps_sq) {
        if (isect_elem_len == 2) {
          BLI_assert(is_coplanar(s, tri_a, tri_b));
          return COPLANAR;
        }
        visit_vert_a[i_a] = true;
        visit_vert_b[i_b] = true;
        vert_a_vert_b[i_a] = i_b;
        vert_b_vert_a[i_b] = i_a;

        isect_elem[isect_elem_len][0].i = i_a;
        isect_elem[isect_elem_len][0].type = VERT;
        isect_elem[isect_elem_len][1].i = i_b;
        isect_elem[isect_elem_len][1].type = VERT;
        isect_elem_len++;
      }
    }
  }

  /* Vert x Edge. */
  for (uint i_a = 0; i_a < 3; i_a++) {
    if (visit_vert_a[i_a]) {
      continue;
    }
    for (uint i_b = 0, i_b_next = 1; i_b < 3; i_b++, i_b_next = (i_b + 1) % 3) {
      if (visit_vert_a[i_a]) {
        continue;
      }
      const float lambda = closest_to_line_v3(
          isect_elem[isect_elem_len][1].ix, tri_a[i_a], tri_b[i_b], tri_b[i_b_next]);
      if (!IN_RANGE(lambda, 0.0f, 1.0f)) {
        continue;
      }
      const float dist_sq = len_squared_v3v3(tri_a[i_a], isect_elem[isect_elem_len][1].ix);
      if (dist_sq <= s->eps_sq) {
        if (isect_elem_len == 2) {
          BLI_assert(is_coplanar(s, tri_a, tri_b));
          return COPLANAR;
        }
        visit_vert_a[i_a] = true;
        vert_a_edge_b[i_a] = i_b;

        isect_elem[isect_elem_len][0].i = i_a;
        isect_elem[isect_elem_len][0].type = VERT;
        isect_elem[isect_elem_len][1].i = i_b;
        isect_elem[isect_elem_len][1].type = EDGE;
        isect_elem[isect_elem_len][1].lambda = lambda;
        isect_elem_len++;
      }
    }
  }

  for (uint i_b = 0; i_b < 3; i_b++) {
    if (visit_vert_b[i_b]) {
      continue;
    }
    for (uint i_a = 0, i_a_next = 1; i_a < 3; i_a++, i_a_next = (i_a + 1) % 3) {
      if (visit_vert_b[i_b]) {
        continue;
      }
      const float lambda = closest_to_line_v3(
          isect_elem[isect_elem_len][0].ix, tri_b[i_b], tri_a[i_a], tri_a[i_a_next]);
      if (!IN_RANGE(lambda, 0.0f, 1.0f)) {
        continue;
      }
      const float dist_sq = len_squared_v3v3(tri_b[i_b], isect_elem[isect_elem_len][0].ix);
      if (dist_sq <= s->eps_sq) {
        if (isect_elem_len == 2) {
          BLI_assert(is_coplanar(s, tri_a, tri_b));
          return COPLANAR;
        }
        visit_vert_b[i_b] = true;
        vert_b_edge_a[i_b] = i_a;

        isect_elem[isect_elem_len][0].i = i_a;
        isect_elem[isect_elem_len][0].type = EDGE;
        isect_elem[isect_elem_len][0].lambda = lambda;
        isect_elem[isect_elem_len][1].i = i_b;
        isect_elem[isect_elem_len][1].type = VERT;
        isect_elem_len++;
      }
    }
  }

  /* Vert x Tri. */
  for (uint i_a = 0; i_a < 3; i_a++) {
    if (visit_vert_a[i_a]) {
      continue;
    }
    if (isect_point_tri_v3(tri_a[i_a], UNPACK3(tri_b), isect_elem[isect_elem_len][1].ix)) {
      const float dist_sq = len_squared_v3v3(tri_a[i_a], isect_elem[isect_elem_len][1].ix);
      if (dist_sq <= s->eps_sq) {
        if (isect_elem_len == 2) {
          BLI_assert(is_coplanar(s, tri_a, tri_b));
          return COPLANAR;
        }
        visit_vert_a[i_a] = true;

        isect_elem[isect_elem_len][0].i = i_a;
        isect_elem[isect_elem_len][0].type = VERT;
        isect_elem[isect_elem_len][1].type = TRI;
        isect_elem_len++;
      }
    }
  }

  for (uint i_b = 0; i_b < 3; i_b++) {
    if (visit_vert_b[i_b]) {
      continue;
    }
    if (isect_point_tri_v3(tri_b[i_b], UNPACK3(tri_a), isect_elem[isect_elem_len][0].ix)) {
      const float dist_sq = len_squared_v3v3(tri_b[i_b], isect_elem[isect_elem_len][0].ix);
      if (dist_sq <= s->eps_sq) {
        if (isect_elem_len == 2) {
          BLI_assert(is_coplanar(s, tri_a, tri_b));
          return COPLANAR;
        }
        visit_vert_b[i_b] = true;

        isect_elem[isect_elem_len][0].type = TRI;
        isect_elem[isect_elem_len][1].i = i_b;
        isect_elem[isect_elem_len][1].type = VERT;
        isect_elem_len++;
      }
    }
  }

  /* Edge x Edge. */
  for (uint i_a = 0, i_a_next = 1; i_a < 3; i_a++, i_a_next = (i_a + 1) % 3) {
    if (visit_vert_a[i_a] && visit_vert_a[i_a_next]) {
      continue;
    }
    for (uint i_b = 0, i_b_next = 1; i_b < 3; i_b++, i_b_next = (i_b + 1) % 3) {
      if (visit_vert_b[i_b] && visit_vert_b[i_b_next]) {
        continue;
      }

      if (ELEM(vert_a_vert_b[i_a], i_b, i_b_next)) {
        continue;
      }
      if (ELEM(vert_a_vert_b[i_a_next], i_b, i_b_next)) {
        continue;
      }
      if (ELEM(i_b, vert_a_edge_b[i_a], vert_a_edge_b[i_a_next])) {
        continue;
      }
      if (ELEM(i_a, vert_b_edge_a[i_b], vert_b_edge_a[i_b_next])) {
        continue;
      }
      float a_dir[3], b_dir[3];
      sub_v3_v3v3(a_dir, tri_a[i_a_next], tri_a[i_a]);
      sub_v3_v3v3(b_dir, tri_b[i_b_next], tri_b[i_b]);

      float lambda_a, lambda_b;
      if (isect_ray_ray_v3(tri_a[i_a], a_dir, tri_b[i_b], b_dir, &lambda_a, &lambda_b)) {
        if (!IN_RANGE(lambda_a, 0.0f, 1.0f)) {
          continue;
        }
        if (!IN_RANGE(lambda_b, 0.0f, 1.0f)) {
          continue;
        }
        madd_v3_v3v3fl(isect_elem[isect_elem_len][0].ix, tri_a[i_a], a_dir, lambda_a);
        madd_v3_v3v3fl(isect_elem[isect_elem_len][1].ix, tri_b[i_b], b_dir, lambda_b);

        const float dist_sq = len_squared_v3v3(isect_elem[isect_elem_len][0].ix,
                                               isect_elem[isect_elem_len][1].ix);
        if (dist_sq <= s->eps_sq) {
          if (isect_elem_len == 2) {
            if (is_tri_degenerate(s, tri_a) || is_tri_degenerate(s, tri_b)) {
              goto add_pairs;
            }
            BLI_assert(is_coplanar(s, tri_a, tri_b));
            return COPLANAR;
          }

          isect_elem[isect_elem_len][0].i = i_a;
          isect_elem[isect_elem_len][0].type = EDGE;
          isect_elem[isect_elem_len][0].lambda = lambda_a;
          isect_elem[isect_elem_len][1].i = i_b;
          isect_elem[isect_elem_len][1].type = EDGE;
          isect_elem[isect_elem_len][1].lambda = lambda_b;
          isect_elem_len++;
        }
      }
    }
  }

  /* Edge x Tri. */
  for (uint i_a = 0, i_a_next = 1; i_a < 3; i_a++, i_a_next = (i_a + 1) % 3) {
    if (visit_vert_a[i_a] || visit_vert_a[i_a_next]) {
      continue;
    }
    if (isect_elem_len) {
      if ((isect_elem[0][0].type == EDGE) && isect_elem[0][0].i == i_a) {
        continue;
      }
    }
    if (isect_elem_len == 2) {
      if ((isect_elem[1][0].type == EDGE) && isect_elem[1][0].i == i_a) {
        continue;
      }
    }
    float lambda;
    if (isect_line_segment_tri_v3(tri_a[i_a], tri_a[i_a_next], UNPACK3(tri_b), &lambda, NULL)) {
      if (isect_elem_len == 2) {
        BLI_assert(is_coplanar(s, tri_a, tri_b));
        return COPLANAR;
      }
      interp_v3_v3v3(isect_elem[isect_elem_len][0].ix, tri_a[i_a], tri_a[i_a_next], lambda);
      isect_elem[isect_elem_len][0].i = i_a;
      isect_elem[isect_elem_len][0].type = EDGE;
      isect_elem[isect_elem_len][0].lambda = lambda;
      isect_elem[isect_elem_len][1].type = TRI;
      isect_elem_len++;
    }
  }

  for (uint i_b = 0, i_b_next = 1; i_b < 3; i_b++, i_b_next = (i_b + 1) % 3) {
    if (visit_vert_b[i_b] || visit_vert_b[i_b_next]) {
      continue;
    }
    if (isect_elem_len) {
      if ((isect_elem[0][1].type == EDGE) && isect_elem[0][1].i == i_b) {
        continue;
      }
    }
    if (isect_elem_len == 2) {
      if ((isect_elem[1][1].type == EDGE) && isect_elem[1][1].i == i_b) {
        continue;
      }
    }
    float lambda;
    if (isect_line_segment_tri_v3(tri_b[i_b], tri_b[i_b_next], UNPACK3(tri_a), &lambda, NULL)) {
      if (isect_elem_len == 2) {
        BLI_assert(is_coplanar(s, tri_a, tri_b));
        return COPLANAR;
      }
      isect_elem[isect_elem_len][0].type = TRI;
      isect_elem[isect_elem_len][1].i = i_b;
      isect_elem[isect_elem_len][1].type = EDGE;
      isect_elem[isect_elem_len][1].lambda = lambda;
      interp_v3_v3v3(isect_elem[isect_elem_len][1].ix, tri_b[i_b], tri_b[i_b_next], lambda);
      isect_elem_len++;
    }
  }

  if (isect_elem_len < 2) {
    return NO_ISECT;
  }

  IsectPair *pairs[2];

add_pairs:
  for (uint i = 0; i < 2; i++) {
    float *ix_a = isect_elem[i][0].ix;
    float *ix_b = isect_elem[i][1].ix;
    uint i_a = isect_elem[i][0].i;
    uint i_b = isect_elem[i][1].i;
    float lambda_a = isect_elem[i][0].lambda;
    float lambda_b = isect_elem[i][1].lambda;
    ISectElemType elem_a = isect_elem[i][0].type;
    ISectElemType elem_b = isect_elem[i][1].type;

    uint i_b_next;
    if (elem_b == EDGE) {
      i_b_next = (i_b + 1) % 3;
    }

    IsectPair *pair;
    if (elem_a == VERT) {
      if (elem_b == VERT) {
        if (fv_a[i_a] == fv_b[i_b]) {
          pair = iso_vert_ensure(s, fv_a[i_a]);
        }
        else {
          pair = vert_vert_pair_ensure(s, fv_a[i_a], fv_b[i_b]);
        }
      }
      else if (elem_b == EDGE) {
        pair = vert_edge_pair_ensure(s, fv_a[i_a], fv_b[i_b], fv_b[i_b_next], lambda_b);
      }
      else {
        pair = vert_tri_pair_ensure(s, fv_a[i_a], b_index, ix_b);
      }
    }
    else if (elem_a == EDGE) {
      uint i_a_next = (i_a + 1) % 3;
      if (elem_b == VERT) {
        pair = vert_edge_pair_ensure(s, fv_b[i_b], fv_a[i_a], fv_a[i_a_next], lambda_a);
      }
      else if (elem_b == EDGE) {
        pair = edge_edge_pair_ensure(s,
                                     fv_a[i_a],
                                     fv_a[i_a_next],
                                     fv_b[i_b],
                                     fv_b[i_b_next],
                                     lambda_a,
                                     lambda_b,
                                     ix_a,
                                     ix_b);
      }
      else {
        pair = edge_tri_pair_ensure(s, fv_a[i_a], fv_a[i_a_next], b_index, lambda_a);
      }
    }
    else {
      if (elem_b == VERT) {
        pair = vert_tri_pair_ensure(s, fv_b[i_b], a_index, ix_a);
      }
      else if (elem_b == EDGE) {
        pair = edge_tri_pair_ensure(s, fv_b[i_b], fv_b[i_b_next], a_index, lambda_b);
      }
      else {
        return NO_ISECT;
      }
    }
    pairs[i] = pair;
  }

  if (pairs[0] == pairs[1]) {
    return NO_ISECT;
  }

  IsectEdge *isect_edge = isect_edge_ensure(s, pairs[0], pairs[1]);
  face_isect_edge_add(s, f_a, isect_edge, true);
  face_isect_edge_add(s, f_b, isect_edge, false);
  return ISECT;
}

/**
 * Return true if we have any intersections.
 */
static void bm_isect_tri_tri(
    struct ISectState *s, int a_index, int b_index, BMLoop **a, BMLoop **b)
{
  BMFace *f_a = (*a)->f;
  BMFace *f_b = (*b)->f;
  if (UNLIKELY(BM_face_share_edge_check(f_a, f_b))) {
    return;
  }

  BMVert *fv_a[3] = {UNPACK3_EX(, a, ->v)};
  BMVert *fv_b[3] = {UNPACK3_EX(, b, ->v)};
  float f_a_cos[3][3];
  float f_b_cos[3][3];
  copy_v3_v3(f_a_cos[0], fv_a[0]->co);
  copy_v3_v3(f_a_cos[1], fv_a[1]->co);
  copy_v3_v3(f_a_cos[2], fv_a[2]->co);
  copy_v3_v3(f_b_cos[0], fv_b[0]->co);
  copy_v3_v3(f_b_cos[1], fv_b[1]->co);
  copy_v3_v3(f_b_cos[2], fv_b[2]->co);

  IsectTriTriMode isect = isect_tri_tri_impl(
      s, f_a_cos, f_b_cos, fv_a, fv_b, a_index, b_index, f_a, f_b);

  if (isect == NO_ISECT) {
    return;
  }

  if (isect == COPLANAR) {
    BLI_assert(is_coplanar(s, f_a_cos, f_b_cos));
    /* Coplanar Case. */
    /* Add coplanar faces. */
    face_coplanar_add(s, f_a, f_b, true);
    face_coplanar_add(s, f_b, f_a, false);
  }
}

typedef struct SplitFace {
  BMFace *f;
  BMLoop *l_a;
  BMLoop *l_b;
} SplitFace;
static bool share_face_fn(BMFace *f, BMLoop *l_a, BMLoop *l_b, void *userdata)
{
  SplitFace f_item;
  f_item.f = f;
  f_item.l_a = l_a;
  f_item.l_b = l_b;
  BLI_buffer_append((BLI_Buffer *)userdata, SplitFace, f_item);
  return false;
}

static void merge_verts_remove_edge_split_face(ISectState *s, BMVert *v_dst, BMVert *v_src)
{
  BLI_buffer_declare_static(SplitFace, split_faces, BLI_BUFFER_NOP, 2);
  BMLoop *l_dummy;
  BM_vert_pair_shared_face_cb(v_src, v_dst, true, share_face_fn, &split_faces, &l_dummy, &l_dummy);
  BMEdge *e_kill = BM_edge_exists(v_dst, v_src);
  if (split_faces.count) {
    /* Update the `GHash *face_links` for faces that will be deleted or added. */
    for (uint i = 0; i < split_faces.count; i++) {
      SplitFace *split_face = &((SplitFace *)split_faces.data)[i];
      FaceLinkLists *f_lists = BLI_ghash_lookup(s->face_links, split_face->f);
      if (BM_loop_is_adjacent(split_face->l_a, split_face->l_b)) {
        if (f_lists && (split_face->f->len <= 3)) {
          BLI_ghash_remove(s->face_links, split_face->f, debug_ghash_free_key_fn, NULL);
        }
      }
      else {
        BMLoop *l_del;
        BMFace *f_new = BM_face_split(
            s->bm, split_face->f, split_face->l_a, split_face->l_b, &l_del, NULL, false);
        if (e_kill) {
          BM_edge_splice(s->bm, e_kill, l_del->e);
        }
        else {
          e_kill = l_del->e;
        }
        if (f_lists) {
          if (f_new->len > 3) {
            void **f_lists_p;
            if (!BLI_ghash_ensure_p(s->face_links, f_new, &f_lists_p)) {
              *f_lists_p = f_lists;
            }
          }
          if (split_face->f->len <= 3) {
            BLI_ghash_remove(s->face_links, split_face->f, debug_ghash_free_key_fn, NULL);
          }
        }
      }
    }
    BLI_buffer_free(&split_faces);
  }
  if (e_kill) {
    bmesh_kernel_join_vert_kill_edge(s->bm, e_kill, v_src, true, false, true);
  }
  else {
    BM_vert_splice(s->bm, v_dst, v_src);
  }
}

static int sort_cmp_by_lambda_fn(const void *a, const void *b, void *data)
{
  const BMEdge *e = data;
  const IsectPair *pair_a = *(IsectPair **)a;
  const IsectPair *pair_b = *(IsectPair **)b;

  float lambda_a = (pair_a->elem_b == e) ? pair_a->lambda_b : pair_a->lambda_a;
  float lambda_b = (pair_b->elem_b == e) ? pair_b->lambda_b : pair_b->lambda_a;

  if (lambda_a > lambda_b) {
    return 1;
  }
  if (lambda_a < lambda_b) {
    return -1;
  }

  /* TODO: best way to sort equal lambda values. */
  return 0;
}

struct IsectEdgeDataIter {
  IsectPair **edge_pair_table;
  BMEdge **edge_affected;

  uint edge_pair_table_len;
  uint edge_in_pair_len;
  uint edge_affected_len;
};

static void split_edge_data_iter_set(struct IsectEdgeDataIter *isect_data, IsectPair *pair)
{
  if (ELEM(EDGE, pair->type_a, pair->type_b)) {
    isect_data->edge_pair_table[isect_data->edge_pair_table_len++] = pair;
  }
  if (pair->type_a == EDGE) {
    BMEdge *e = pair->elem_a;
    int e_index = BM_elem_index_get(e);
    BM_elem_index_set(e, e_index + 1);
    isect_data->edge_in_pair_len++;
    if (e_index == 0) {
      isect_data->edge_affected[isect_data->edge_affected_len++] = e;
    }
  }
  if (pair->type_b == EDGE) {
    BMEdge *e = pair->elem_b;
    int e_index = BM_elem_index_get(e);
    BM_elem_index_set(e, e_index + 1);
    isect_data->edge_in_pair_len++;
    if (e_index == 0) {
      isect_data->edge_affected[isect_data->edge_affected_len++] = e;
    }
  }
}

static void bm_split_edges(ISectState *s)
{
  BMesh *bm = s->bm;
  BMEdge *e;

  uint edge_pair_len_alloc = BLI_ghash_len(s->vert_edge) + BLI_ghash_len(s->edge_tri) +
                             BLI_ghash_len(s->edge_edge);

  struct IsectEdgeDataIter isect_data = {NULL};
  isect_data.edge_pair_table = MEM_mallocN(sizeof(IsectPair *) * edge_pair_len_alloc, __func__);
  isect_data.edge_affected = MEM_mallocN(
      sizeof(BMEdge *) * (edge_pair_len_alloc + BLI_ghash_len(s->edge_edge)), __func__);

  GHashIterator gh_iter;
  GHASH_ITER (gh_iter, s->vert_edge) {
    IsectPair *pair = BLI_ghashIterator_getValue(&gh_iter);
    split_edge_data_iter_set(&isect_data, pair);
  }

  GHASH_ITER (gh_iter, s->edge_tri) {
    IsectPair *pair = BLI_ghashIterator_getValue(&gh_iter);
    split_edge_data_iter_set(&isect_data, pair);
  }

  GHASH_ITER (gh_iter, s->edge_edge) {
    IsectPair *pair = BLI_ghashIterator_getValue(&gh_iter);
    split_edge_data_iter_set(&isect_data, pair);
  }

  int offset = 0;
  for (uint i = 0; i < isect_data.edge_affected_len; i++) {
    e = isect_data.edge_affected[i];
    int len = BM_elem_index_get(e);
    BM_elem_index_set(e, offset);
    offset += len;
  }

  IsectPair **edge_pair = MEM_mallocN(sizeof(*edge_pair) * isect_data.edge_in_pair_len, __func__);
  for (uint i = 0; i < isect_data.edge_pair_table_len; i++) {
    IsectPair *pair = isect_data.edge_pair_table[i];
    if (pair->type_a == EDGE) {
      e = pair->elem_a;
      uint slot = BM_elem_index_get(e);
      BM_elem_index_set(e, slot + 1);
      edge_pair[slot] = pair;
    }
    if (pair->type_b == EDGE) {
      e = pair->elem_b;
      uint slot = BM_elem_index_get(e);
      BM_elem_index_set(e, slot + 1);
      edge_pair[slot] = pair;
    }
  }

  MEM_freeN(isect_data.edge_pair_table);

  offset = 0;
  for (uint i = 0; i < isect_data.edge_affected_len; i++) {
    e = isect_data.edge_affected[i];
    int end = BM_elem_index_get(e);
    int len = end - offset;
    if (len > 1) {
      BLI_qsort_r(&edge_pair[offset], len, sizeof(*edge_pair), sort_cmp_by_lambda_fn, e);
    }

    float lambda, lambda_prev = 0.0f;
    do {
      IsectPair *pair = edge_pair[offset];
      float pair_lambda = (pair->elem_b == e) ? pair->lambda_b : pair->lambda_a;

      lambda = (pair_lambda - lambda_prev) / (1.0f - lambda_prev);
      lambda_prev = pair_lambda;

      BMEdge *e_new;
      BMVert *v_new = BM_edge_split(bm, e, e->v1, &e_new, lambda);
      // BM_elem_flag_enable(v_new, VERT_DISSOLVE_FLAG);
      BLI_assert(e_new->v2 == v_new);
      BLI_assert(e->v1 == v_new);

      BMEdge *e_dst_a = NULL;
      BMEdge *e_dst_b = NULL;
      if (pair->elem_b == e) {
        if (pair->type_a == VERT) {
          e_dst_a = BM_edge_exists(e_new->v1, pair->elem_a);
          e_dst_b = BM_edge_exists(pair->elem_a, e->v2);
          merge_verts_remove_edge_split_face(s, pair->elem_a, v_new);
          // pair->type_b = NONE;
#ifdef DEBUG_BOOLEAN
          pair->elem_b = NULL;
#endif
        }
        else {
          pair->elem_b = pair->elem_a;
          pair->type_b = pair->type_a;
          pair->lambda_b = pair->lambda_a;
          pair->elem_a = v_new;
          pair->type_a = VERT;
        }
      }
      else {
        pair->type_a = VERT;
        if (pair->type_b == VERT) {
          e_dst_a = BM_edge_exists(e_new->v1, pair->elem_b);
          e_dst_b = BM_edge_exists(pair->elem_b, e->v2);
          merge_verts_remove_edge_split_face(s, pair->elem_b, v_new);

          pair->elem_a = pair->elem_b;
          // pair->type_b = NONE;
#ifdef DEBUG_BOOLEAN
          pair->elem_b = NULL;
#endif
        }
        else {
          pair->elem_a = v_new;
        }
      }

      if (e_dst_a) {
        BM_edge_splice(bm, e_dst_a, e_new);
      }
      if (e_dst_b) {
        BM_edge_splice(bm, e, e_dst_b);
      }
    } while (++offset != end);
  }

  MEM_freeN(isect_data.edge_affected);
  MEM_freeN(edge_pair);
}

static BMVert *vert_find_merge_dst(IsectPair **vert_pair_table, BMVert *v)
{
  BMVert *v_iter = v;
  int pair_i = BM_elem_index_get(v);
  while (pair_i != -1) {
    v_iter = vert_pair_table[pair_i]->elem_a;
    pair_i = BM_elem_index_get(v_iter);
    BLI_assert(v_iter != v);
  }
  return v_iter;
}

static void bm_vert_splice_and_remove_doubles(ISectState *s, BMVert *v_a, BMVert *v_b)
{
  struct {
    void *next;
    BMEdge *e_dst;
    BMEdge *e_src;
  } *edge_splice_link = NULL;

  BMIter iter;
  BMEdge *e;
  BM_ITER_ELEM (e, &iter, v_a, BM_EDGES_OF_VERT) {
    BMVert *v_other = BM_edge_other_vert(e, v_a);
    if (v_other == v_b) {
      continue;
    }
    BMEdge *e_src = BM_edge_exists(v_other, v_b);
    if (e_src) {
      void *next = edge_splice_link;
      edge_splice_link = alloca(sizeof(*edge_splice_link));
      edge_splice_link->next = next;
      edge_splice_link->e_dst = e;
      edge_splice_link->e_src = e_src;
    }
  }

  merge_verts_remove_edge_split_face(s, v_a, v_b);
  while (edge_splice_link) {
    BM_edge_splice(s->bm, edge_splice_link->e_dst, edge_splice_link->e_src);
    edge_splice_link = edge_splice_link->next;
  }
}

static void bm_merge_verts(ISectState *s)
{
  EdgeHashIterator *ehi;
  GHashIterator gh_iter;

  BMesh *bm = s->bm;
  BMIter iter;
  BMVert *v;
  BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
    BM_elem_index_set(v, -1);
    BM_elem_flag_disable(v, VERT_DISSOLVE_FLAG);
  }
  bm->elem_index_dirty |= BM_VERT;

  uint vert_pair_table_len = BLI_edgehash_len(s->vert_vert);
  if (vert_pair_table_len == 0) {
    return;
  }

  IsectPair **vert_pair_table = MEM_mallocN(sizeof(*vert_pair_table) * vert_pair_table_len,
                                            __func__);
  vert_pair_table_len = 0;
  for (ehi = BLI_edgehashIterator_new(s->vert_vert); BLI_edgehashIterator_isDone(ehi) == false;
       BLI_edgehashIterator_step(ehi)) {
    IsectPair *pair = BLI_edgehashIterator_getValue(ehi);
    BLI_assert((pair->type_a == VERT) && (pair->type_b == VERT));
    vert_pair_table[vert_pair_table_len++] = pair;
  }
  BLI_edgehashIterator_free(ehi);
  BLI_assert(vert_pair_table_len == (uint)BLI_edgehash_len(s->vert_vert));

#ifdef DEBUG_BOOLEAN
  GHASH_ITER (gh_iter, s->edge_edge) {
    IsectPair *pair = BLI_ghashIterator_getValue(&gh_iter);
    BLI_assert(pair->type_b != VERT);
  }
#endif

  /* Rearrange merged vert chain to find the final vert for each group. */

  for (uint i = 0; i < vert_pair_table_len; i++) {
    IsectPair *pair = vert_pair_table[i];
    BMVert *v_b = pair->elem_b;
    int v_b_index = BM_elem_index_get(v_b);
    if (v_b_index == -1) {
      BM_elem_index_set(v_b, i);
    }
    else {
      BMVert *v_a_dst = vert_find_merge_dst(vert_pair_table, v_b);
      if (v_a_dst != pair->elem_a) {
        pair->elem_b = v_a_dst;
        BM_elem_index_set(v_a_dst, i);
      }
      else {
        pair->elem_b = NULL;
      }
    }
  }
  for (uint i = 0; i < vert_pair_table_len; i++) {
    IsectPair *pair = vert_pair_table[i];
    pair->elem_a = vert_find_merge_dst(vert_pair_table, pair->elem_a);
  }

#ifdef DEBUG_BOOLEAN
  for (uint i = 0; i < vert_pair_table_len; i++) {
    IsectPair *pair = vert_pair_table[i];
    BMVert *v_a = pair->elem_a;
    BLI_assert(BM_elem_index_get(v_a) == -1);
    if (pair->elem_b) {
      BMVert *v_b = pair->elem_b;
      BLI_assert(BM_elem_index_get(v_b) != -1);
    }
  }
#endif

  /* Avoid referencing vertices that will be deleted. */
  GHASH_ITER (gh_iter, s->iso_vert) {
    IsectPair *pair = BLI_ghashIterator_getValue(&gh_iter);
    BLI_assert(pair->type_a == VERT);
    pair->elem_a = vert_find_merge_dst(vert_pair_table, pair->elem_a);
  }
  GHASH_ITER (gh_iter, s->vert_edge) {
    IsectPair *pair = BLI_ghashIterator_getValue(&gh_iter);
    BLI_assert(pair->type_a == VERT);
    pair->elem_a = vert_find_merge_dst(vert_pair_table, pair->elem_a);
  }

  /* Now merge verts. */
  for (uint i = 0; i < vert_pair_table_len; i++) {
    IsectPair *pair = vert_pair_table[i];
    if (pair->elem_b) {
      bm_vert_splice_and_remove_doubles(s, pair->elem_a, pair->elem_b);
    }
#ifdef DEBUG_BOOLEAN
    pair->elem_b = NULL;
#endif
  }

  MEM_freeN(vert_pair_table);
}

static bool face_edges_split(BMesh *bm,
                             BMFace *f,
                             FaceLinkLists *f_lists,
                             bool use_island_connect,
                             bool use_partial_connect,
                             MemArena *mem_arena_edgenet,
                             BMFace ***r_face_arr,
                             int *r_face_arr_len)
{
  uint edge_arr_len = f_lists->edge_list_len;
  BMEdge **edge_arr = BLI_array_alloca(edge_arr, edge_arr_len);
  LinkNode *node;
  BLI_assert(f->head.htype == BM_FACE);

  uint i = 0;
  for (node = f_lists->edge_list; node; node = node->next) {
    edge_arr[i++] = ((IsectEdge *)node->link)->edge;
  }
  BLI_assert(i == f_lists->edge_list_len);

  if (use_island_connect) {
    uint edge_arr_holes_len;
    BMEdge **edge_arr_holes;
    if (BM_face_split_edgenet_connect_islands(bm,
                                              f,
                                              edge_arr,
                                              edge_arr_len,
                                              use_partial_connect,
                                              mem_arena_edgenet,
                                              &edge_arr_holes,
                                              &edge_arr_holes_len)) {
      edge_arr_len = edge_arr_holes_len;
      edge_arr = edge_arr_holes; /* owned by the arena */
    }
  }

  return BM_face_split_edgenet(bm, f, edge_arr, (int)edge_arr_len, r_face_arr, r_face_arr_len) &&
         *r_face_arr;
}

static void bm_split_faces_and_tag_edges(ISectState *s,
                                         const bool use_island_connect,
                                         const bool use_partial_connect)
{
  BMesh *bm = s->bm;

  /* Convert #IsectPairs to #BMEdge. */
  GSetIterator gs_iter;
  GSET_ITER (gs_iter, s->isect_edges) {
    IsectEdge *isect_edge = BLI_gsetIterator_getKey(&gs_iter);
    if (isect_edge->pair_a->elem_a == isect_edge->pair_b->elem_a) {
      /* It shouldn't happen. */
      isect_edge->edge = NULL;
      continue;
    }
    if (isect_edge->pair_a->type_a != VERT) {
      BMVert *v = BM_vert_create(bm, isect_edge->pair_a->co, NULL, 0);
      isect_edge->pair_a->elem_a = v;
      isect_edge->pair_a->type_a = VERT;
      BM_elem_flag_enable(v, VERT_DISSOLVE_FLAG);
    }
    if (isect_edge->pair_b->type_a != VERT) {
      BMVert *v = BM_vert_create(bm, isect_edge->pair_b->co, NULL, 0);
      isect_edge->pair_b->elem_a = v;
      isect_edge->pair_b->type_a = VERT;
      BM_elem_flag_enable(v, VERT_DISSOLVE_FLAG);
    }
    BMVert *v_a = isect_edge->pair_a->elem_a;
    BMVert *v_b = isect_edge->pair_b->elem_a;
    BMEdge *e = BM_edge_exists(v_a, v_b);
    if (e) {
      /* It shouldn't happen, but sometimes a pair with type NONE can form from one with type EDGE
       * even though it already exists. */
      // BLI_assert(BM_elem_index_get(e) != -2);
    }
    else {
      e = BM_edge_create(bm, v_a, v_b, NULL, 0);
      BM_elem_index_set(e, -2);
    }
    BM_elem_flag_enable(e, EDGE_ISECT_FLAG);
    isect_edge->edge = e;
  }

  /* Avoid mutable acess. */
  struct {
    BMFace *f;
    FaceLinkLists *f_lists;
  } *f_links_array = BLI_array_alloca(f_links_array, BLI_ghash_len(s->face_links));

  uint f_links_len = 0;
  GHashIterator gh_iter;
  GHASH_ITER (gh_iter, s->face_links) {
    BMFace *f = BLI_ghashIterator_getKey(&gh_iter);
    FaceLinkLists *f_lists = BLI_ghashIterator_getValue(&gh_iter);
    BLI_assert((uint)BLI_linklist_count(f_lists->edge_list) == f_lists->edge_list_len);
    if (f_lists->edge_list_len == 0) {
      continue;
    }

    f_links_array[f_links_len].f = f;
    f_links_array[f_links_len].f_lists = f_lists;
    f_links_len++;
  }

  if (f_links_len == 0) {
    return;
  }

  MemArena *mem_arena_edgenet = BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE, __func__);

  for (uint i = 0; i < f_links_len; i++) {
    BMFace *f = f_links_array[i].f;
    FaceLinkLists *f_lists = f_links_array[i].f_lists;
    LinkNode **node_ptr, *node, *node_next;

    /* Remove NULL. */
    node_ptr = &f_lists->edge_list;
    node = *node_ptr;
    do {
      node_next = node->next;
      BMEdge *e = ((IsectEdge *)node->link)->edge;
      if (!e) {
        /* Remove node. */
        isect_node_remove(s, node_ptr, node);
        f_lists->edge_list_len--;
        continue;
      }
      node_ptr = &node->next;
    } while ((node = node_next));

    if (f_lists->edge_list_len == 0) {
      continue;
    }

    /* Remove duplicated edges.
     * It should never happen, but it happens because of the imprecision that causes pairs that
     * should reference EDGEs to reference TRIS. */
    node_ptr = &f_lists->edge_list;
    node = *node_ptr;
    do {
      node_next = node->next;
      BMEdge *e = ((IsectEdge *)node->link)->edge;
      if (!BM_elem_flag_test(e, EDGE_ISECT_FLAG)) {
        /* Remove edge. */
        isect_node_remove(s, node_ptr, node);
        f_lists->edge_list_len--;
        continue;
      }
      BM_elem_flag_disable(e, EDGE_ISECT_FLAG);
      node_ptr = &node->next;
    } while ((node = node_next));

    /* Remove edges in face. */
    node_ptr = &f_lists->edge_list;
    node = *node_ptr;
    do {
      node_next = node->next;
      BMEdge *e = ((IsectEdge *)node->link)->edge;
      BM_elem_flag_enable(e, EDGE_ISECT_FLAG);
      if ((BM_elem_index_get(e) != -2) && BM_edge_in_face(e, f)) {
        /* Remove edge. */
        isect_node_remove(s, node_ptr, node);
        f_lists->edge_list_len--;
        continue;
      }
      node_ptr = &node->next;
    } while ((node = node_next));

    BLI_assert((uint)BLI_linklist_count(f_lists->edge_list) == f_lists->edge_list_len);
    if (f_lists->edge_list_len == 0) {
      continue;
    }

    BMFace **face_arr = NULL;
    int face_arr_len = 0;
    if (face_edges_split(bm,
                         f,
                         f_lists,
                         use_island_connect,
                         use_partial_connect,
                         mem_arena_edgenet,
                         &face_arr,
                         &face_arr_len)) {

      if (f_lists->face_coplanar_list) {
        for (uint j = 0; j < face_arr_len; j++) {
          BMFace *f_new = face_arr[j];
          if (f_new == f) {
            continue;
          }

          FaceLinkLists *f_new_lists;
          f_new_lists = ghash_face_link_lists_ensure(
              s->face_links, f_new, s->mem_arena, f_lists->is_face_a);
          f_new_lists->face_coplanar_list = f_lists->face_coplanar_list;

          for (node = f_lists->face_coplanar_list; node; node = node->next) {
            BMFace *f_coplanar = node->link;
            face_coplanar_add(s, f_coplanar, f_new, !f_lists->is_face_a);
          }
        }
      }
      MEM_freeN(face_arr);
    }

    BLI_memarena_clear(mem_arena_edgenet);
  }

  BLI_memarena_free(mem_arena_edgenet);
}

static bool bm_vert_inside_face_eps(BMVert *v, BMFace *f, const float eps_sq)
{
  BMLoop *l_iter, *l_first, *l_prev;
  l_iter = l_first = f->l_first;
  l_prev = l_first->prev;
  do {
    if (l_iter->v == v) {
      return true;
    }
    const float dist_sq = dist_squared_to_line_segment_v3(v->co, l_prev->v->co, l_iter->v->co);
    if (dist_sq <= eps_sq) {
      return true;
    }
    l_prev = l_iter;
  } while ((l_iter = l_iter->next) != l_first);

  return BM_face_point_inside_test(f, v->co);
}

static void bm_resolve_coplanar_faces(ISectState *s,
                                      const bool kill_face_a,
                                      const bool kill_face_b)
{
  /* For quick acess. */
  struct {
    BMFace *f;
    FaceLinkLists *f_lists;
    bool is_face_a;
  } *f_links_array = BLI_array_alloca(f_links_array, BLI_ghash_len(s->face_links));

  uint f_links_len = 0;
  GHashIterator gh_iter;
  GHASH_ITER (gh_iter, s->face_links) {
    BMFace *f = BLI_ghashIterator_getKey(&gh_iter);
    FaceLinkLists *f_lists = BLI_ghashIterator_getValue(&gh_iter);
    if (f_lists->face_coplanar_list == NULL) {
      continue;
    }

    f_links_array[f_links_len].f = f;
    f_links_array[f_links_len].f_lists = f_lists;
    f_links_array[f_links_len].is_face_a = f_lists->is_face_a;
    f_links_len++;

#ifdef DEBUG_BOOLEAN
    BMVert *v;
    BMIter iter;
    BM_ITER_ELEM (v, &iter, f, BM_VERTS_OF_FACE) {
      BLI_assert(BM_elem_index_get(v) != -2);
    }
#endif
  }

  if (f_links_len == 0) {
    return;
  }

  for (uint i = 0; i < f_links_len; i++) {
    BMFace *f = f_links_array[i].f;
    FaceLinkLists *f_lists = f_links_array[i].f_lists;
    BM_face_calc_normal(f, f->no);

    for (LinkNode *node = f_lists->face_coplanar_list; node; node = node->next) {
      BMFace *f_coplanar = node->link;

      BMVert *v;
      BMIter iter;
      BM_ITER_ELEM (v, &iter, f_coplanar, BM_VERTS_OF_FACE) {
        if (BM_elem_index_get(v) == -2) {
          continue;
        }
        if (bm_vert_inside_face_eps(v, f, s->eps_sq)) {
          BM_elem_index_set(v, -2);
        }
      }
    }
  }

  BMesh *bm = s->bm;
  bm->elem_index_dirty |= BM_VERT;
  for (uint i = 0; i < f_links_len; i++) {
    BMFace *f = f_links_array[i].f;
    BMVert *v;
    BMIter iter;
    bool is_overlap = true;
    BM_ITER_ELEM (v, &iter, f, BM_VERTS_OF_FACE) {
      if (BM_elem_index_get(v) != -2) {
        is_overlap = false;
        break;
      }
    }

    if (is_overlap) {
      if (f_links_array[i].is_face_a) {
        if (!kill_face_a) {
          continue;
        }
      }
      else {
        if (!kill_face_b) {
          continue;
        }
      }
      BM_face_kill_loose(bm, f);
    }
  }
}

static bool bm_mesh_intersect(BMesh *bm,
                              struct BMLoop *(*looptris)[3],
                              BVHTreeOverlap *overlap,
                              const uint tree_overlap_tot,
                              const bool use_island_connect,
                              const bool use_partial_connect,
                              const bool kill_coplanar_a,
                              const bool kill_coplanar_b,
                              const float eps)
{
  BMIter iter;
  BMEdge *e;
  BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
    BM_elem_index_set(e, 0);
    BM_elem_flag_disable(e, EDGE_ISECT_FLAG);
  }
  bm->elem_index_dirty |= BM_EDGE;

  ISectState s;
  isect_state_init(&s, bm, eps);

  BM_mesh_elem_index_ensure(bm, BM_VERT);
  for (uint i = 0; i < tree_overlap_tot; i++) {
    bm_isect_tri_tri(&s,
                     overlap[i].indexA,
                     overlap[i].indexB,
                     looptris[overlap[i].indexA],
                     looptris[overlap[i].indexB]);
  }

  bm_split_edges(&s);
  bm_merge_verts(&s);
  bm_split_faces_and_tag_edges(&s, use_island_connect, use_partial_connect);
  bm_resolve_coplanar_faces(&s, kill_coplanar_a, kill_coplanar_b);

  /* use to check if we made any changes */
  const bool has_edit_boolean = (BLI_ghash_len(s.face_links) != 0);

  isect_state_free(&s);

  /* It's unlikely the selection history is useful at this point,
   * if this is not called this array would need to be validated, see: #86799. */
  BM_select_history_clear(bm);

  return has_edit_boolean;
}

struct LoopFilterWrap {
  int (*test_fn)(BMFace *f, void *user_data);
  void *user_data;
};

static bool bm_loop_filter_fn(const BMLoop *l, void *user_data)
{
  if (BM_elem_flag_test(l->e, EDGE_ISECT_FLAG)) {
    return false;
  }

  if (l->radial_next != l) {
    struct LoopFilterWrap *data = user_data;
    BMLoop *l_iter = l->radial_next;
    const int face_side = data->test_fn(l->f, data->user_data);
    do {
      const int face_side_other = data->test_fn(l_iter->f, data->user_data);
      if (UNLIKELY(face_side_other == -1)) {
        /* pass */
      }
      else if (face_side_other != face_side) {
        return false;
      }
    } while ((l_iter = l_iter->radial_next) != l);
    return true;
  }
  return false;
}
struct RaycastData {
  const float (*looptris)[3][3];
  BLI_Buffer *z_buffer;
  float offset;
};

static void raycast_callback(void *userdata,
                             int index,
                             const BVHTreeRay *ray,
                             BVHTreeRayHit *UNUSED(hit))
{
  struct RaycastData *raycast_data = userdata;
  const float(*looptris)[3][3] = raycast_data->looptris;
  const float *v0 = looptris[index][0];
  const float *v1 = looptris[index][1];
  const float *v2 = looptris[index][2];
  float dist;

  if (
#ifdef USE_KDOPBVH_WATERTIGHT
      isect_ray_tri_watertight_v3(ray->origin, ray->isect_precalc, v0, v1, v2, &dist, NULL)
#else
      isect_ray_tri_epsilon_v3(ray->origin, ray->direction, v0, v1, v2, &dist, NULL, FLT_EPSILON)
#endif
  ) {
    dist += raycast_data->offset;
    /* Ignore Coplanar. */
    if ((dist < -RAYCAST_DEPTH_EPSILON) || (dist > RAYCAST_DEPTH_EPSILON)) {
      BLI_buffer_append(raycast_data->z_buffer, float, dist);
    }
  }
}

static float isect_bvhtree_point_v3(BVHTree *tree,
                                    const float (*looptris)[3][3],
                                    const float co[3],
                                    const float dir[3])
{
  BLI_buffer_declare_static(float, z_buffer, BLI_BUFFER_NOP, 64);

  struct RaycastData raycast_data = {
      looptris,
      &z_buffer,
  };
  BVHTreeRayHit hit = {0};

  /* Need to initialize hit even tho it's not used.
   * This is to make it so KD-tree believes we didn't intersect anything and
   * keeps calling the intersect callback.
   */
  hit.index = -1;
  hit.dist = BVH_RAYCAST_DIST_MAX;

  /* We need negative depth, but this is not yet supported by raycast, so move the origin of the
   * ray to the intersection point in the Bound Box opposite the ray and use an offset to know what
   * is negative. */
  float co_tmp[3], bb_min[3], bb_max[3];
  BLI_bvhtree_get_bounding_box(tree, bb_min, bb_max);
  if (!isect_ray_aabb_v3_simple(co, dir, bb_min, bb_max, &raycast_data.offset, NULL)) {
    return 0;
  }
  raycast_data.offset -= 1.0f;
  madd_v3_v3v3fl(co_tmp, co, dir, raycast_data.offset);

  BLI_bvhtree_ray_cast_ex(tree,
                          co_tmp,
                          dir,
                          0.0f,
                          &hit,
                          raycast_callback,
                          &raycast_data,
#ifdef USE_KDOPBVH_WATERTIGHT
                          BVH_RAYCAST_WATERTIGHT
#else
                          0
#endif
  );

  if (z_buffer.count > 1) {
    qsort(z_buffer.data, z_buffer.count, sizeof(float), BLI_sortutil_cmp_float);
  }
  else {
    return 0;
  }

  /* Count the values not equal.
   * Equal values are obtained where 2 triangles meet for example. */

  uint depth_arr_len = z_buffer.count;
  uint depth_arr_neg_len = 0;
  const float *depth_arr = z_buffer.data;

  float depth_prev = depth_arr[0];
  if (depth_prev < 0) {
    do {
      depth_arr++;
      depth_arr_len--;
      if ((*depth_arr - depth_prev) > RAYCAST_DEPTH_EPSILON) {
        depth_arr_neg_len++;
      }
      depth_prev = *depth_arr;
    } while ((depth_prev < 0) && (depth_arr_len != 0));
  }

  if (depth_arr_len > 1) {
    uint i = depth_arr_len;
    while (--i) {
      depth_arr++;
      if ((*depth_arr - depth_prev) <= RAYCAST_DEPTH_EPSILON) {
        depth_arr_len--;
      }
      depth_prev = *depth_arr;
    }
  }

  BLI_buffer_free(&z_buffer);
  return min_ii(depth_arr_neg_len, depth_arr_len);
}

bool BM_mesh_intersect(BMesh *bm,
                       struct BMLoop *(*looptris)[3],
                       const int looptris_tot,
                       int (*test_fn)(BMFace *f, void *user_data),
                       void *user_data,
                       const bool use_self,
                       const bool use_separate,
                       const bool use_dissolve,
                       const bool use_island_connect,
                       const bool use_partial_connect,
                       const bool UNUSED(use_edge_tag),
                       const int boolean_mode,
                       const float eps)
{
  BVHTree *tree_a, *tree_b;
  uint tree_overlap_tot;
  BVHTreeOverlap *overlap;

  /* use to check if we made any changes */
  bool has_edit_isect = false, has_edit_boolean = false;

  /* needed for boolean, since cutting up faces moves the loops within the face */
  float(*looptri_coords)[3][3] = NULL;

  if (boolean_mode != BMESH_ISECT_BOOLEAN_NONE) {
    /* Keep original geometry for ray-cast callbacks. */
    float(*cos)[3][3];
    cos = MEM_mallocN((size_t)looptris_tot * sizeof(*cos), __func__);
    for (int i = 0; i < looptris_tot; i++) {
      copy_v3_v3(cos[i][0], looptris[i][0]->v->co);
      copy_v3_v3(cos[i][1], looptris[i][1]->v->co);
      copy_v3_v3(cos[i][2], looptris[i][2]->v->co);
    }
    looptri_coords = cos;
  }

  {
    tree_a = BLI_bvhtree_new(looptris_tot, eps, 8, 8);
    for (int i = 0; i < looptris_tot; i++) {
      if (test_fn(looptris[i][0]->f, user_data) == 0) {
        const float t_cos[3][3] = {
            {UNPACK3(looptris[i][0]->v->co)},
            {UNPACK3(looptris[i][1]->v->co)},
            {UNPACK3(looptris[i][2]->v->co)},
        };

        BLI_bvhtree_insert(tree_a, i, (const float *)t_cos, 3);
      }
    }
    BLI_bvhtree_balance(tree_a);
  }

  if (use_self == false) {
    tree_b = BLI_bvhtree_new(looptris_tot, eps, 8, 8);
    for (int i = 0; i < looptris_tot; i++) {
      if (test_fn(looptris[i][0]->f, user_data) == 1) {
        const float t_cos[3][3] = {
            {UNPACK3(looptris[i][0]->v->co)},
            {UNPACK3(looptris[i][1]->v->co)},
            {UNPACK3(looptris[i][2]->v->co)},
        };

        BLI_bvhtree_insert(tree_b, i, (const float *)t_cos, 3);
      }
    }
    BLI_bvhtree_balance(tree_b);
  }
  else {
    tree_b = tree_a;
  }

  int flag = BVH_OVERLAP_USE_THREADING | BVH_OVERLAP_RETURN_PAIRS;
  overlap = BLI_bvhtree_overlap_ex(tree_b, tree_a, &tree_overlap_tot, NULL, NULL, 0, flag);

  if (overlap) {
    has_edit_isect = bm_mesh_intersect(bm,
                                       looptris,
                                       overlap,
                                       tree_overlap_tot,
                                       use_island_connect,
                                       use_partial_connect,
                                       false,
                                       true,
                                       eps);
    MEM_freeN(overlap);
  }

  if (use_separate) {
    BM_mesh_edgesplit(bm, false, true, false);
  }

  if (boolean_mode != BMESH_ISECT_BOOLEAN_NONE) {
    BVHTree *tree_pair[2] = {tree_a, tree_b};

    /* group vars */
    int *groups_array;
    int(*group_index)[2];
    int group_tot;
    int i;
    BMFace **ftable;

    BM_mesh_elem_table_ensure(bm, BM_FACE);
    ftable = bm->ftable;

    /* wrap the face-test callback to make it into an edge-loop delimiter */
    struct LoopFilterWrap user_data_wrap = {
        .test_fn = test_fn,
        .user_data = user_data,
    };

    groups_array = MEM_mallocN(sizeof(*groups_array) * (size_t)bm->totface, __func__);
    group_tot = BM_mesh_calc_face_groups(
        bm, groups_array, &group_index, bm_loop_filter_fn, NULL, &user_data_wrap, 0, BM_EDGE);

    float point_in_tree[2][3];
    {
      bool has_point[2] = {false, false};
      for (i = 0; i < group_tot; i++) {
        int fg = group_index[i][0];
        BMFace *f = ftable[groups_array[fg]];
        int side = test_fn(f, user_data);
        if (has_point[side]) {
          continue;
        }
        BM_face_calc_point_in_face(f, point_in_tree[side]);
        if (has_point[!side]) {
          break;
        }
      }
    }

    /* Check if island is inside/outside */
    for (i = 0; i < group_tot; i++) {
      int fg = group_index[i][0];
      int fg_end = group_index[i][1] + fg;
      bool do_remove, do_flip;

      {
        /* For now assume this is an OK face to test with (not degenerate!) */
        BMFace *f = ftable[groups_array[fg]];
        float co[3];
        int side = test_fn(f, user_data);

        if (side == -1) {
          continue;
        }
        BLI_assert(ELEM(side, 0, 1));
        side = !side;

        // BM_face_calc_center_median(f, co);
        BM_face_calc_point_in_face(f, co);

        /* Any direction can do. */
        float dir[3] = {1, 0, 0};

        uint hits = isect_bvhtree_point_v3(tree_pair[side], looptri_coords, co, dir);
        const bool is_inside = (hits & 1);

        switch (boolean_mode) {
          case BMESH_ISECT_BOOLEAN_ISECT:
            do_remove = !is_inside;
            do_flip = false;
            break;
          case BMESH_ISECT_BOOLEAN_UNION:
            do_remove = is_inside;
            do_flip = false;
            break;
          case BMESH_ISECT_BOOLEAN_DIFFERENCE:
            do_remove = is_inside == (bool)side;
            do_flip = (side == 0);
            break;
        }
      }

      if (do_remove) {
        for (; fg != fg_end; fg++) {
          /* postpone killing the face since we access below, mark instead */
          // BM_face_kill_loose(bm, ftable[groups_array[fg]]);
          ftable[groups_array[fg]]->mat_nr = -1;
        }
      }
      else if (do_flip) {
        for (; fg != fg_end; fg++) {
          BM_face_normal_flip(bm, ftable[groups_array[fg]]);
        }
      }

      has_edit_boolean |= (do_flip || do_remove);
    }

    MEM_freeN(groups_array);
    MEM_freeN(group_index);

    {
      int tot = bm->totface;
      for (i = 0; i < tot; i++) {
        if (ftable[i]->mat_nr == -1) {
          BM_face_kill_loose(bm, ftable[i]);
        }
      }
    }
  }

#ifdef USE_DISSOLVE
  /* We have dissolve code above, this is alternative logic,
   * we need to do it after the boolean is executed. */
  if (use_dissolve) {
    BMVert *v, *v_next;
    BMIter iter;
    BM_ITER_MESH_MUTABLE (v, v_next, &iter, bm, BM_VERTS_OF_MESH) {
      if (!BM_elem_flag_test(v, VERT_DISSOLVE_FLAG)) {
        continue;
      }
      if (BM_vert_is_edge_pair(v)) {
        /* we won't create degenerate faces from this */
        bool ok = true;

        /* would we create a 2-sided-face?
         * if so, don't dissolve this since we may */
        if (v->e->l) {
          BMLoop *l_iter = v->e->l;
          do {
            if (l_iter->f->len == 3) {
              ok = false;
              break;
            }
          } while ((l_iter = l_iter->radial_next) != v->e->l);
        }

        if (ok) {
          BM_vert_collapse_edge(bm, v->e, v, true, false, false);
        }
      }
    }
  }
#endif

  MEM_SAFE_FREE(looptri_coords);
  BLI_bvhtree_free(tree_a);
  if (tree_a != tree_b) {
    BLI_bvhtree_free(tree_b);
  }

  return (has_edit_isect || has_edit_boolean);
}
