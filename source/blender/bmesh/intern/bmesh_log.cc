/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bmesh
 *
 */

#include "MEM_guardedalloc.h"

#include "BKE_customdata.h"
#include "BKE_lib_id.h"
#include "BKE_mesh.h"

#include "DNA_customdata_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BLI_alloca.h"
#include "BLI_array.hh"
#include "BLI_asan.h"
#include "BLI_hash.hh"
#include "BLI_listbase_wrapper.hh"
#include "BLI_map.hh"
#include "BLI_math_vector.h"
#include "BLI_math_vector_types.hh"
#include "BLI_memory_utils.hh"
#include "BLI_mempool.h"
#include "BLI_set.hh"
#include "BLI_vector.hh"

#include "bmesh.h"
#include "bmesh_idmap.hh"
#include "bmesh_log.h"

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <type_traits>

using blender::Array;
using blender::DynamicStackBuffer;
using blender::float3;
using blender::IndexRange;
using blender::Map;
using blender::Set;
using blender::Vector;

/* Avoid C++ runtime type ids. */
enum class BMLogSetType { LOG_SET_DIFF, LOG_SET_FULL };

/* `customdata_layout_is_same` and `customdata_copy_all_layout` are
 *  used internally by BMLog and probably don't have much use elsewhere.
 *  They ignore all copy-on-write semantics which makes sense since this
 *  is BMesh.
 */

/* Returns true if the exact layout of a is the same as b. */
static bool customdata_layout_is_same(const CustomData *data_a, const CustomData *data_b)
{
  if (data_a->totlayer != data_b->totlayer || data_a->totsize != data_b->totsize) {
    return false;
  }

  for (int i : IndexRange(data_a->totlayer)) {
    CustomDataLayer &layer_a = data_a->layers[i];
    CustomDataLayer &layer_b = data_b->layers[i];

    if (!STREQ(layer_a.name, layer_b.name) || layer_a.type != layer_b.type ||
        layer_a.offset != layer_b.offset)
    {
      return false;
    }
  }

  return true;
}

/* Copies all customdata layers without allocating data
 * and without respect to type masks or NO_COPY/etc flags.
 */
static void customdata_copy_all_layout(const struct CustomData *source, struct CustomData *dest)
{
  *dest = *source;
  dest->external = nullptr;
  dest->pool = nullptr;

  if (source->layers) {
    dest->layers = static_cast<CustomDataLayer *>(
        MEM_mallocN(sizeof(*dest->layers) * source->maxlayer, __func__));

    for (int i : IndexRange(source->totlayer)) {
      CustomDataLayer *layer = &dest->layers[i];

      *layer = source->layers[i];

      layer->data = nullptr;
      layer->anonymous_id = nullptr;
      layer->sharing_info = nullptr;
    }
  }

  CustomData_update_typemap(dest);
}

void swap_customdata_block(CustomData *data_a, CustomData *data_b, void **block_a, void **block_b)
{
  if (!data_a->totsize || !data_b->totsize) {
    return;
  }
  if (!*block_a) {
    CustomData_bmesh_set_default(data_a, block_a);
  }
  if (!*block_b) {
    CustomData_bmesh_set_default(data_a, block_b);
  }

  DynamicStackBuffer<128> buffer(data_b->totsize, 4);
  void *scratch = buffer.buffer();
  memset(scratch, 0, data_b->totsize);

  CustomData_bmesh_copy_data(data_a, data_b, *block_a, &scratch);
  CustomData_bmesh_copy_data(data_b, data_a, *block_b, block_a);
  CustomData_bmesh_copy_data(data_a, data_b, scratch, block_b);
}

template<typename T> struct BMLogElem {
  int id;
  void *customdata;
  char flag;

#ifdef WITH_ASAN
  bool dead = false;
  ~BMLogElem()
  {
    dead = true;
  }
#endif

  void free(CustomData *domain)
  {
    if (customdata) {
      CustomData_bmesh_free_block_data(domain, customdata);
    }
  }
};

struct BMLogVert : public BMLogElem<BMVert> {
  float3 co;
  float3 no;
};

struct BMLogEdge : public BMLogElem<BMEdge> {
  int v1 = BM_ID_NONE;
  int v2 = BM_ID_NONE;
};

struct BMLogFace : public BMLogElem<BMFace> {
  std::array<int, 3> verts;
  std::array<void *, 3> loop_customdata;

  void free(CustomData *domain, CustomData *loop_domain)
  {
    BMLogElem<BMFace>::free(domain);

    if (loop_customdata[0]) {
      for (void *data : loop_customdata) {
        CustomData_bmesh_free_block_data(loop_domain, data);
      }
    }
  }
};

struct BMLogEntry;

static BMIdMap *entry_get_idmap(BMLogEntry *entry);

struct BMLogSetBase {
  BMLogSetType type;
  BMLogEntry *entry = nullptr; /* Parent entry */

  BMLogSetBase(BMLogEntry *_entry, BMLogSetType _type) : type(_type), entry(_entry) {}

  virtual ~BMLogSetBase() {}

  virtual const char *debug_name()
  {
    return "";
  }
  virtual void undo(BMesh * /*bm*/) = 0;
  virtual void redo(BMesh * /*bm*/) = 0;
};

struct BMLogSetDiff : public BMLogSetBase {
  BMLogSetDiff(BMLogEntry *entry) : BMLogSetBase(entry, BMLogSetType::LOG_SET_DIFF) {}

  Map<int, BMLogVert *> modified_verts;
  Map<int, BMLogEdge *> modified_edges;
  Map<int, BMLogFace *> modified_faces;

  Map<int, BMLogVert *> removed_verts;
  Map<int, BMLogEdge *> removed_edges;
  Map<int, BMLogFace *> removed_faces;

  Map<int, BMLogVert *> added_verts;
  Map<int, BMLogEdge *> added_edges;
  Map<int, BMLogFace *> added_faces;

  const char *debug_name() override
  {
    return "Diff";
  }

  void add_vert(BMesh *bm, BMVert *v);
  void remove_vert(BMesh *bm, BMVert *v);
  void modify_vert(BMesh *bm, BMVert *v);
  void add_edge(BMesh *bm, BMEdge *e);
  void remove_edge(BMesh *bm, BMEdge *e);
  void modify_edge(BMesh *bm, BMEdge *e);
  void add_face(BMesh *bm, BMFace *f);
  void remove_face(BMesh *bm, BMFace *f);
  void modify_face(BMesh *bm, BMFace *f);

  void undo(BMesh *bm) override;
  void redo(BMesh *bm) override;

  void restore_verts(BMesh *bm, blender::Map<int, BMLogVert *> verts);
  void remove_verts(BMesh *bm, blender::Map<int, BMLogVert *> verts);
  void swap_verts(BMesh *bm, blender::Map<int, BMLogVert *> verts);
  void restore_edges(BMesh *bm, blender::Map<int, BMLogEdge *> edges);
  void remove_edges(BMesh *bm, blender::Map<int, BMLogEdge *> edges);
  void swap_edges(BMesh *bm, blender::Map<int, BMLogEdge *> edges);

  void restore_faces(BMesh *bm, blender::Map<int, BMLogFace *> faces);
  void remove_faces(BMesh *bm, blender::Map<int, BMLogFace *> faces);
  void swap_faces(BMesh *bm, blender::Map<int, BMLogFace *> faces);
};

struct BMLogSetFullMesh : public BMLogSetBase {
  BMLogSetFullMesh(BMesh *bm, BMLogEntry *entry, BMIdMap *idmap)
      : BMLogSetBase(entry, BMLogSetType::LOG_SET_FULL)
  {
    /* Validate id map. */
    BM_idmap_check_ids(idmap);

    BMeshToMeshParams params{};
    mesh = static_cast<Mesh *>(BKE_id_new_nomain(ID_ME, nullptr));
    BM_mesh_bm_to_me(nullptr, bm, mesh, &params);
  }

  ~BMLogSetFullMesh()
  {
    if (mesh) {
      BKE_mesh_free_data_for_undo(mesh);
      MEM_SAFE_FREE(mesh);
    }
  }

  const char *debug_name() override
  {
    return "Full";
  }

  void swap(BMesh *bm)
  {
    CustomData_MeshMasks cd_mask_extra{};

    BMeshToMeshParams params{};

    /* Don't swap in a new mesh if bmesh is empty, happens during undo. */
    Mesh *current_mesh = nullptr;
    if (bm->totvert > 0) {
      current_mesh = static_cast<Mesh *>(BKE_id_new_nomain(ID_ME, nullptr));
      BM_mesh_bm_to_me(nullptr, bm, current_mesh, &params);
    }

    int shapenr = bm->shapenr;

    BMeshFromMeshParams params2 = {};
    params2.cd_mask_extra = cd_mask_extra;
    params2.calc_face_normal = params2.add_key_index = params2.use_shapekey = false;

    BM_mesh_clear(bm);
    BM_mesh_bm_from_me(bm,
                       mesh, /* Note: we stored shapekeys as customdata layers,
                              * that's why the shapekey params are false.
                              */
                       &params2);

    /* Regenerate ID map. */
    BMIdMap *idmap = entry_get_idmap(entry);
    BM_idmap_check_ids(idmap);

    bm->shapenr = shapenr;

    bm->elem_index_dirty |= BM_VERT | BM_EDGE | BM_FACE;
    bm->elem_table_dirty |= BM_VERT | BM_EDGE | BM_FACE;

    BM_mesh_elem_table_ensure(bm, BM_VERT | BM_EDGE | BM_FACE);
    BM_mesh_elem_index_ensure(bm, BM_VERT | BM_EDGE | BM_FACE);

    if (current_mesh) {
      BKE_mesh_free_data_for_undo(mesh);
      MEM_SAFE_FREE(mesh);
      mesh = current_mesh;
    }
  }

  void undo(BMesh *bm) override
  {
    swap(bm);
  }

  void redo(BMesh *bm) override
  {
    swap(bm);
  }

  Mesh *mesh = nullptr;
};

static const char *get_elem_htype_str(int htype)
{
  switch (htype) {
    case BM_VERT:
      return "vertex";
    case BM_EDGE:
      return "edge";
    case BM_LOOP:
      return "loop";
    case BM_FACE:
      return "face";
    default:
      return "unknown type";
  }
}

template<typename T> constexpr char get_elem_type()
{
  if constexpr (std::is_same_v<T, BMVert>) {
    return BM_VERT;
  }
  else if constexpr (std::is_same_v<T, BMEdge>) {
    return BM_EDGE;
  }
  else if constexpr (std::is_same_v<T, BMLoop>) {
    return BM_LOOP;
  }
  else if constexpr (std::is_same_v<T, BMFace>) {
    return BM_FACE;
  }
}

struct BMLogEntry {
  BMLogEntry *next = nullptr, *prev = nullptr;

  Vector<std::unique_ptr<BMLogSetBase>> sets;
  BLI_mempool *vpool = nullptr;
  BLI_mempool *epool = nullptr;
  BLI_mempool *fpool = nullptr;

  CustomData vdata;
  CustomData edata;
  CustomData ldata;
  CustomData pdata;

  BMIdMap *idmap = nullptr;

  int cd_mask_offset = -1;

  BMLog *log = nullptr;
  bool dead = false;

  bool cd_layout_changed = false;

  BMLogEntry(BMIdMap *_idmap,
             CustomData *src_vdata,
             CustomData *src_edata,
             CustomData *src_ldata,
             CustomData *src_pdata)
      : idmap(_idmap)
  {
    vpool = BLI_mempool_create(sizeof(BMLogVert), 0, 512, BLI_MEMPOOL_ALLOW_ITER);
    epool = BLI_mempool_create(sizeof(BMLogEdge), 0, 512, BLI_MEMPOOL_ALLOW_ITER);
    fpool = BLI_mempool_create(sizeof(BMLogFace), 0, 512, BLI_MEMPOOL_ALLOW_ITER);

    customdata_copy_all_layout(src_vdata, &vdata);
    customdata_copy_all_layout(src_edata, &edata);
    customdata_copy_all_layout(src_ldata, &ldata);
    customdata_copy_all_layout(src_pdata, &pdata);

    CustomData_bmesh_init_pool(&vdata, 0, BM_VERT);
    CustomData_bmesh_init_pool(&edata, 0, BM_EDGE);
    CustomData_bmesh_init_pool(&ldata, 0, BM_LOOP);
    CustomData_bmesh_init_pool(&pdata, 0, BM_FACE);

    cd_mask_offset = CustomData_get_offset(&vdata, CD_PAINT_MASK);
  }

  BMLogVert *find_logvert(BMVert *v)
  {
    int id = get_elem_id(v);
    BMLogVert **lv = nullptr;

    for (int i = sets.size() - 1; i >= 0; i--) {
      BMLogSetBase *set = sets[i].get();

      if (set->type != BMLogSetType::LOG_SET_DIFF) {
        continue;
      }

      BMLogSetDiff *diff = static_cast<BMLogSetDiff *>(set);

      lv = diff->modified_verts.lookup_ptr(id);
      if (lv) {
        return *lv;
      }

      lv = diff->added_verts.lookup_ptr(id);
      if (lv) {
        return *lv;
      }
    }

    return nullptr;
  }

  ~BMLogEntry()
  {
    dead = true;

    BLI_mempool_iter iter;

    BLI_mempool_iternew(vpool, &iter);
    BMLogVert *vert = static_cast<BMLogVert *>(BLI_mempool_iterstep(&iter));
    for (; vert; vert = static_cast<BMLogVert *>(BLI_mempool_iterstep(&iter))) {
      vert->free(&vdata);
    }

    BLI_mempool_iternew(epool, &iter);
    BMLogEdge *edge = static_cast<BMLogEdge *>(BLI_mempool_iterstep(&iter));
    for (; edge; edge = static_cast<BMLogEdge *>(BLI_mempool_iterstep(&iter))) {
      edge->free(&edata);
    }

    BLI_mempool_iternew(fpool, &iter);
    BMLogFace *face = static_cast<BMLogFace *>(BLI_mempool_iterstep(&iter));
    for (; face; face = static_cast<BMLogFace *>(BLI_mempool_iterstep(&iter))) {
      face->free(&pdata, &ldata);
    }

    BLI_mempool_destroy(vpool);
    BLI_mempool_destroy(epool);
    BLI_mempool_destroy(fpool);

    if (vdata.pool) {
      BLI_mempool_destroy(vdata.pool);
    }
    if (edata.pool) {
      BLI_mempool_destroy(edata.pool);
    }
    if (ldata.pool) {
      BLI_mempool_destroy(ldata.pool);
    }
    if (pdata.pool) {
      BLI_mempool_destroy(pdata.pool);
    }

    CustomData_free(&vdata, 0);
    CustomData_free(&edata, 0);
    CustomData_free(&ldata, 0);
    CustomData_free(&pdata, 0);
  }

  void print()
  {
    int av = 0, ae = 0, af = 0, mv = 0, me = 0, mf = 0, dv = 0, de = 0, df = 0;
    int totmesh = 0;

    for (std::unique_ptr<BMLogSetBase> &set : sets) {
      switch (set->type) {
        case BMLogSetType::LOG_SET_DIFF: {
          BMLogSetDiff *diff = static_cast<BMLogSetDiff *>(set.get());

          av += diff->added_verts.size();
          ae += diff->added_edges.size();
          af += diff->added_faces.size();

          mv += diff->modified_verts.size();
          me += diff->modified_edges.size();
          mf += diff->modified_faces.size();

          dv += diff->removed_verts.size();
          de += diff->removed_edges.size();
          df += diff->removed_faces.size();
          break;
        }
        case BMLogSetType::LOG_SET_FULL:
          totmesh++;
          break;
      }
    }

    if (av + ae + af + mv + me + mf + dv + de + df) {
      printf("\n  addv: %d, adde: %d, addf: %d\n", av, ae, af);
      printf("  modv: %d, mode: %d, modf: %d\n", mv, me, mf);
      printf("  delv: %d, dele: %d, delf: %d\n", dv, de, df);
    }

    if (totmesh > 0) {
      printf("  totmesh: %d\n", totmesh);
    }
  }

  template<typename T> T *get_elem_from_id(int id)
  {
    if (id < 0 || id >= idmap->map.size()) {
      return nullptr;
    }

    T *elem = BM_idmap_lookup<T>(idmap, id);
    char htype = 0;

    if (!elem) {
      return nullptr;
    }

    if constexpr (std::is_same_v<T, BMVert>) {
      htype = BM_VERT;
    }
    if constexpr (std::is_same_v<T, BMEdge>) {
      htype = BM_EDGE;
    }
    if constexpr (std::is_same_v<T, BMFace>) {
      htype = BM_FACE;
    }

    if (elem->head.htype != htype) {
      printf("%s: error: expected %s, got %s; id: %d\n",
             __func__,
             get_elem_htype_str(htype),
             get_elem_htype_str(elem->head.htype),
             id);
      return nullptr;
    }

    return elem;
  }

  template<typename T> void assign_elem_id(BMesh * /*bm*/, T *elem, int id, bool check_unique)
  {
    if (check_unique && id >= 0 && id < idmap->map.size()) {
      T *old = BM_idmap_lookup<T>(idmap, id);

      if (old && old != elem) {
        printf(
            "id conflict in BMLogEntry::assign_elem_id; elem %p (a %s) is being reassinged to id "
            "%d.\n",
            elem,
            get_elem_htype_str((int)elem->head.htype),
            (int)id);
        printf(
            "  elem %p (a %s) will get a new id\n", old, get_elem_htype_str((int)old->head.htype));

        BM_idmap_assign(idmap, elem, id);
        return;
      }
    }

    BM_idmap_assign(idmap, elem, id);
  }

  template<typename T> int get_elem_id(T *elem)
  {
    BM_idmap_check_assign(idmap, elem);
    return BM_idmap_get_id(idmap, elem);
  }

  void push_set(BMesh *bm, BMLogSetType type)
  {
    switch (type) {
      case BMLogSetType::LOG_SET_DIFF: {
        sets.append(
            std::unique_ptr<BMLogSetBase>(static_cast<BMLogSetBase *>(new BMLogSetDiff(this))));
        break;
      }
      case BMLogSetType::LOG_SET_FULL:
        sets.append(std::unique_ptr<BMLogSetBase>(
            static_cast<BMLogSetBase *>(new BMLogSetFullMesh(bm, this, idmap))));
        break;
    }
  }

  BMLogSetDiff *current_diff_set(BMesh *bm)
  {
    if (sets.size() == 0 || sets[sets.size() - 1]->type != BMLogSetType::LOG_SET_DIFF) {
      push_set(bm, BMLogSetType::LOG_SET_DIFF);
    }

    return static_cast<BMLogSetDiff *>(sets[sets.size() - 1].get());
  }

  BMLogSetDiff *first_diff_set(BMesh *bm)
  {
    for (std::unique_ptr<BMLogSetBase> &set : sets) {
      if (set->type == BMLogSetType::LOG_SET_DIFF) {
        return static_cast<BMLogSetDiff *>(set.get());
      }
    }

    return current_diff_set(bm);
  }

  void update_logvert(BMesh *bm, BMVert *v, BMLogVert *lv)
  {
    CustomData_bmesh_copy_data(&bm->vdata, &vdata, v->head.data, &lv->customdata);

    lv->co = v->co;
    lv->no = v->no;
    lv->flag = v->head.hflag;
  }

  void swap_logvert(BMesh *bm, int /*id*/, BMVert *v, BMLogVert *lv)
  {
    if (v->head.data && lv->customdata) {
      swap_customdata_block(&vdata, &bm->vdata, &lv->customdata, &v->head.data);
    }

    std::swap(v->head.hflag, lv->flag);
    swap_v3_v3(v->co, lv->co);
    swap_v3_v3(v->no, lv->no);
  }

  void swap_logedge(BMesh *bm, int /*id*/, BMEdge *e, BMLogEdge *le)
  {
    if (e->head.data && le->customdata) {
      swap_customdata_block(&edata, &bm->edata, &le->customdata, &e->head.data);
    }

    std::swap(e->head.hflag, le->flag);
  }

  void swap_logface(BMesh *bm, int /*id*/, BMFace *f, BMLogFace *lf)
  {
    if (f->head.data && lf->customdata) {
      swap_customdata_block(&pdata, &bm->pdata, &lf->customdata, &f->head.data);
    }

    if (f->len != lf->verts.size()) {
      printf("%s: error: wrong length for face, was %d, should be %d\n",
             __func__,
             f->len,
             (int)lf->verts.size());
      return;
    }

    if (lf->loop_customdata[0]) {
      BMLoop *l = f->l_first;

      int i = 0;
      do {
        swap_customdata_block(&ldata, &bm->ldata, &lf->loop_customdata[i], &l->head.data);
        i++;
      } while ((l = l->next) != f->l_first);
    }
    std::swap(f->head.hflag, lf->flag);
  }

  template<typename T> T *alloc_logelem(BLI_mempool *pool)
  {
    return static_cast<T *>(BLI_mempool_calloc(pool));
  }

  template<typename T> void free_logelem(BLI_mempool *pool, T *elem)
  {
    BLI_mempool_free(pool, static_cast<void *>(elem));
  }

  BMLogVert *alloc_logvert(BMesh *bm, BMVert *v)
  {
    int id = get_elem_id<BMVert>(v);
    BMLogVert *lv = alloc_logelem<BMLogVert>(vpool);

    lv->id = id;

    update_logvert(bm, v, lv);

    return lv;
  }

  void free_logvert(BMLogVert *lv)
  {
    if (lv->customdata) {
      BLI_mempool_free(vdata.pool, lv->customdata);
    }

    free_logelem(vpool, lv);
  }

  void load_vert(BMesh *bm, BMVert *v, BMLogVert *lv)
  {
    if (v->head.data && lv->customdata) {
      CustomData_bmesh_copy_data(&vdata, &bm->vdata, lv->customdata, &v->head.data);
    }

    v->head.hflag = lv->flag;
    copy_v3_v3(v->co, lv->co);
    copy_v3_v3(v->no, lv->no);
  }

  BMLogEdge *alloc_logedge(BMesh *bm, BMEdge *e)
  {
    BMLogEdge *le = alloc_logelem<BMLogEdge>(epool);

    le->id = get_elem_id<BMEdge>(e);
    le->v1 = get_elem_id<BMVert>(e->v1);
    le->v2 = get_elem_id<BMVert>(e->v2);

    update_logedge(bm, e, le);

    return le;
  }

  void update_logedge(BMesh *bm, BMEdge *e, BMLogEdge *le)
  {
    le->flag = e->head.hflag;
    CustomData_bmesh_copy_data(&bm->edata, &edata, e->head.data, &le->customdata);
  }

  void free_logedge(BMesh * /*bm*/, BMLogEdge *le)
  {
    if (le->customdata) {
      BLI_mempool_free(edata.pool, le->customdata);
    }

    free_logelem(epool, le);
  }

  BMLogFace *alloc_logface(BMesh *bm, BMFace *f)
  {
    BMLogFace *lf = alloc_logelem<BMLogFace>(fpool);

    lf->id = get_elem_id<BMFace>(f);
    lf->flag = f->head.hflag;

    CustomData_bmesh_copy_data(&bm->pdata, &pdata, f->head.data, &lf->customdata);

    BMLoop *l = f->l_first;
    int i = 0;
    do {
      lf->verts[i] = get_elem_id<BMVert>(l->v);
      void *loop_customdata = nullptr;

      if (l->head.data) {
        CustomData_bmesh_copy_data(&bm->ldata, &ldata, l->head.data, &loop_customdata);
      }

      lf->loop_customdata[i] = loop_customdata;
      i++;
    } while (i < 3 && (l = l->next) != f->l_first);

    return lf;
  }

  void update_logface(BMesh *bm, BMLogFace *lf, BMFace *f)
  {
    lf->flag = f->head.hflag;

    CustomData_bmesh_copy_data(&bm->pdata, &pdata, f->head.data, &lf->customdata);

    if (f->len != lf->verts.size()) {
      printf("%s: error: face length changed.\n", __func__);
      return;
    }

    BMLoop *l = f->l_first;
    int i = 0;
    do {
      if (l->head.data) {
        CustomData_bmesh_copy_data(&bm->ldata, &ldata, l->head.data, &lf->loop_customdata[i]);
      }

      i++;
    } while ((l = l->next) != f->l_first);
  }

  void free_logface(BMesh * /*bm*/, BMLogFace *lf)
  {
    if (lf->loop_customdata[0]) {
      for (int i = 0; i < lf->verts.size(); i++) {
        BLI_mempool_free(ldata.pool, lf->loop_customdata[i]);
      }
    }

    if (lf->customdata) {
      BLI_mempool_free(pdata.pool, lf->customdata);
    }

    free_logelem(fpool, lf);
  }

  void add_vert(BMesh *bm, BMVert *v)
  {
    current_diff_set(bm)->add_vert(bm, v);
  }

  void remove_vert(BMesh *bm, BMVert *v)
  {
    current_diff_set(bm)->remove_vert(bm, v);
  }

  void modify_vert(BMesh *bm, BMVert *v)
  {
    current_diff_set(bm)->modify_vert(bm, v);
  }

  void add_edge(BMesh *bm, BMEdge *e)
  {
    current_diff_set(bm)->add_edge(bm, e);
  }

  void remove_edge(BMesh *bm, BMEdge *e)
  {
    current_diff_set(bm)->remove_edge(bm, e);
  }

  void modify_edge(BMesh *bm, BMEdge *e)
  {
    current_diff_set(bm)->modify_edge(bm, e);
  }

  void add_face(BMesh *bm, BMFace *f)
  {
    current_diff_set(bm)->add_face(bm, f);
  }
  void remove_face(BMesh *bm, BMFace *f)
  {
    current_diff_set(bm)->remove_face(bm, f);
  }
  void modify_face(BMesh *bm, BMFace *f)
  {
    current_diff_set(bm)->modify_face(bm, f);
  }

  void undo(BMesh *bm)
  {
    for (int i = sets.size() - 1; i >= 0; i--) {
      sets[i]->undo(bm);
    }
  }

  void redo(BMesh *bm)
  {
    for (int i = 0; i < sets.size(); i++) {
      sets[i]->redo(bm);
    }
  }
};

struct BMLog {
  BMIdMap *idmap = nullptr;
  BMLogEntry *current_entry = nullptr;
  BMLogEntry *first_entry = nullptr;
  int refcount = 1;
  bool dead = false;

  BMLog(BMesh *bm)
  {
    idmap = BM_idmap_new(bm, BM_VERT | BM_EDGE | BM_FACE);
    BM_idmap_check_ids(idmap);
  }

  ~BMLog() {}

  void set_entries_idmap(BMIdMap *new_idmap)
  {
    idmap = new_idmap;

    BMLogEntry *entry = first_entry;
    while (entry) {
      entry->idmap = new_idmap;
      entry = entry->next;
    }
  }

  bool free_all_entries()
  {
    BMLogEntry *entry = first_entry;

    if (!entry) {
      return false;
    }

    while (entry) {
      BMLogEntry *next = entry->next;

      MEM_delete<BMLogEntry>(entry);
      entry = next;
    }

    return true;
  }

  BMLogEntry *push_entry(BMesh *bm)
  {
    BMLogEntry *entry = MEM_new<BMLogEntry>(
        "BMLogEntry", idmap, &bm->vdata, &bm->edata, &bm->ldata, &bm->pdata);

    /* Truncate undo list. */
    BMLogEntry *entry2 = current_entry ? current_entry->next : nullptr;
    while (entry2) {
      BMLogEntry *next = entry2->next;
      MEM_delete<BMLogEntry>(entry2);

      entry2 = next;
    }

    entry->prev = current_entry;
    entry->log = this;
    entry->idmap = idmap;

    if (!first_entry) {
      first_entry = entry;
    }
    else if (current_entry) {
      current_entry->next = entry;
    }

    current_entry = entry;
    return entry;
  }

  void load_entries(BMLogEntry *entry)
  {
    first_entry = current_entry = entry;
    while (first_entry->prev) {
      first_entry = first_entry->prev;
    }

    entry = first_entry;
    while (entry) {
      entry->log = this;
      entry->idmap = idmap;
      entry = entry->next;
    }
  }

  void ensure_entry(BMesh *bm)
  {
    if (!current_entry) {
      push_entry(bm);
    }
  }

  void add_vert(BMesh *bm, BMVert *v)
  {
    ensure_entry(bm);
    current_entry->add_vert(bm, v);
  }

  void remove_vert(BMesh *bm, BMVert *v)
  {
    ensure_entry(bm);
    current_entry->remove_vert(bm, v);
  }

  void modify_vert(BMesh *bm, BMVert *v)
  {
    ensure_entry(bm);
    current_entry->modify_vert(bm, v);
  }

  void add_edge(BMesh *bm, BMEdge *e)
  {
    ensure_entry(bm);
    current_entry->add_edge(bm, e);
  }

  void remove_edge(BMesh *bm, BMEdge *e)
  {
    ensure_entry(bm);
    current_entry->remove_edge(bm, e);
  }

  void modify_edge(BMesh *bm, BMEdge *e)
  {
    ensure_entry(bm);
    current_entry->modify_edge(bm, e);
  }

  void add_face(BMesh *bm, BMFace *f)
  {
    ensure_entry(bm);
    current_entry->add_face(bm, f);
  }

  void remove_face(BMesh *bm, BMFace *f)
  {
    ensure_entry(bm);
    current_entry->remove_face(bm, f);
  }

  void modify_face(BMesh *bm, BMFace *f)
  {
    ensure_entry(bm);
    current_entry->modify_face(bm, f);
  }

  void full_mesh(BMesh *bm)
  {
    ensure_entry(bm);
    current_entry->push_set(bm, BMLogSetType::LOG_SET_FULL);
  }

  void skip(int dir)
  {
    if (current_entry) {
      current_entry = dir > 0 ? current_entry->next : current_entry->prev;
    }
  }

  void undo(BMesh *bm)
  {
    if (!current_entry) {
      current_entry = first_entry;
      while (current_entry->next) {
        current_entry = current_entry->next;
      }
    }

    current_entry->undo(bm);
    current_entry = current_entry->prev;
  }

  void redo(BMesh *bm)
  {
    if (!current_entry) {
      current_entry = first_entry;
    }
    else {
      current_entry = current_entry->next;
    }

    if (current_entry) {
      current_entry->redo(bm);
    }
  }
};

void BMLogSetDiff::add_vert(BMesh *bm, BMVert *v)
{
  int id = entry->get_elem_id(v);

  BMLogVert *lv = nullptr;
  if (added_verts.contains(id)) {
    return;
  }

  if (!lv) {
    lv = entry->alloc_logvert(bm, v);
  }

  added_verts.add(id, lv);
}

void BMLogSetDiff::remove_vert(BMesh *bm, BMVert *v)
{
  int id = entry->get_elem_id(v);

  BMLogVert **added_lv = added_verts.lookup_ptr(id);
  if (added_lv) {
    added_verts.remove(id);
    entry->free_logvert(*added_lv);
    BM_idmap_release(entry->idmap, v);
    return;
  }

  BMLogVert *lv;
  BMLogVert **modified_lv = modified_verts.lookup_ptr(id);
  if (modified_lv) {
    modified_verts.remove(id);
    lv = *modified_lv;
  }
  else {
    lv = entry->alloc_logvert(bm, v);
  }

  removed_verts.add(id, lv);
  BM_idmap_release(entry->idmap, v);
}

void BMLogSetDiff::modify_vert(BMesh *bm, BMVert *v)
{
  int id = entry->get_elem_id(v);
  if (modified_verts.contains(id)) {
    return;
  }

  BMLogVert **added_lv = added_verts.lookup_ptr(id);
  if (added_lv) {
    entry->update_logvert(bm, v, *added_lv);
  }
  else {
    modified_verts.add(id, entry->alloc_logvert(bm, v));
  }
}

void BMLogSetDiff::add_edge(BMesh *bm, BMEdge *e)
{
  int id = entry->get_elem_id(e);
  BMLogEdge *le;

  le = entry->alloc_logedge(bm, e);
  added_edges.add_or_modify(
      id, [&](BMLogEdge **le_out) { *le_out = le; }, [&](BMLogEdge **le_out) { *le_out = le; });
}

void BMLogSetDiff::remove_edge(BMesh *bm, BMEdge *e)
{
  int id = entry->get_elem_id(e);

  if (added_edges.remove(id)) {
    BM_idmap_release(entry->idmap, e);
    return;
  }

  BMLogEdge *le;
  BMLogEdge **modified_le = modified_edges.lookup_ptr(id);
  if (modified_le) {
    le = *modified_le;
    modified_edges.remove(id);
  }
  else {
    le = entry->alloc_logedge(bm, e);
  }

  removed_edges.add(id, le);
  BM_idmap_release(entry->idmap, e);
}

void BMLogSetDiff::modify_edge(BMesh *bm, BMEdge *e)
{
  int id = entry->get_elem_id(e);

  if (modified_edges.contains(id)) {
    return;
  }

  modified_edges.add(id, entry->alloc_logedge(bm, e));
}

void BMLogSetDiff::add_face(BMesh *bm, BMFace *f)
{
  BM_idmap_check_assign(entry->idmap, f);

  int id = entry->get_elem_id<BMFace>(f);

  if (added_faces.contains(id)) {
    return;
  }

  added_faces.add(id, entry->alloc_logface(bm, f));
}

void BMLogSetDiff::remove_face(BMesh *bm, BMFace *f)
{
  int id = entry->get_elem_id<BMFace>(f);

  if (added_faces.remove(id)) {
    BM_idmap_release(entry->idmap, f);
    return;
  }

  BMLogFace *lf;

  if (BMLogFace **ptr = modified_faces.lookup_ptr(id)) {
    lf = *ptr;
    modified_faces.remove(id);
    if (lf->verts.size() != f->len) {
      entry->update_logface(bm, lf, f);
    }
  }
  else {
    lf = entry->alloc_logface(bm, f);
  }

  removed_faces.add(id, lf);
  BM_idmap_release(entry->idmap, f);
}

void BMLogSetDiff::modify_face(BMesh *bm, BMFace *f)
{
  int id = entry->get_elem_id<BMFace>(f);

  BMLogFace *lf;

  if (BMLogFace **ptr = modified_faces.lookup_ptr(id)) {
    lf = *ptr;
    entry->update_logface(bm, lf, f);
  }
  else {
    lf = entry->alloc_logface(bm, f);

    modified_faces.add(id, lf);
  }
}

void BMLogSetDiff::swap_verts(BMesh *bm, blender::Map<int, BMLogVert *> verts)
{
  void *old_customdata = bm->vdata.pool ? BLI_mempool_alloc(bm->vdata.pool) : nullptr;

  const int cd_id = entry->idmap->cd_id_off[BM_VERT];

  for (BMLogVert *lv : verts.values()) {
    BMVert *v = entry->get_elem_from_id<BMVert>(lv->id);

    if (!v) {
      printf("modified_verts: invalid vertex %d\n", lv->id);
      continue;
    }

    if (old_customdata) {
      memcpy(old_customdata, v->head.data, bm->vdata.totsize);
    }

    entry->swap_logvert(bm, lv->id, v, lv);

    /* Ensure id wasn't mangled in customdata swap. */
    BM_ELEM_CD_SET_INT(v, cd_id, lv->id);
  }

  if (old_customdata) {
    BLI_mempool_free(bm->vdata.pool, old_customdata);
  }
}

void BMLogSetDiff::restore_verts(BMesh *bm, blender::Map<int, BMLogVert *> verts)
{
  for (BMLogVert *lv : verts.values()) {
    BMVert *v = BM_vert_create(bm, lv->co, nullptr, BM_CREATE_NOP);

    v->head.hflag = lv->flag;
    copy_v3_v3(v->no, lv->no);

    CustomData_bmesh_copy_data(&entry->vdata, &bm->vdata, lv->customdata, &v->head.data);
    entry->assign_elem_id<BMVert>(bm, v, lv->id, true);
  }

  bm->elem_index_dirty |= BM_VERT | BM_EDGE;
  bm->elem_table_dirty |= BM_VERT | BM_EDGE;
}

void BMLogSetDiff::remove_verts(BMesh *bm, blender::Map<int, BMLogVert *> verts)
{
  for (BMLogVert *lv : verts.values()) {
    BMVert *v = entry->get_elem_from_id<BMVert>(lv->id);

    if (!v) {
      printf("%s: Failed to find vertex %d\b", __func__, lv->id);
      continue;
    }

    BM_idmap_release(entry->idmap, v, false);
    BM_vert_kill(bm, v);
  }

  bm->elem_index_dirty |= BM_VERT | BM_EDGE;
  bm->elem_table_dirty |= BM_VERT | BM_EDGE;
}

void BMLogSetDiff::restore_edges(BMesh *bm, blender::Map<int, BMLogEdge *> edges)
{
  for (BMLogEdge *le : edges.values()) {
    BMVert *v1 = entry->get_elem_from_id<BMVert>(le->v1);
    BMVert *v2 = entry->get_elem_from_id<BMVert>(le->v2);

    if (!v1) {
      printf("%s: missing vertex v1 %d\n", __func__, le->v1);
      continue;
    }

    if (!v2) {
      printf("%s: missing vertex v2 %d\n", __func__, le->v2);
      continue;
    }

    BMEdge *e = BM_edge_create(bm, v1, v2, nullptr, BM_CREATE_NOP);
    e->head.hflag = le->flag;

    CustomData_bmesh_copy_data(&entry->edata, &bm->edata, le->customdata, &e->head.data);

    entry->assign_elem_id<BMEdge>(bm, e, le->id, true);
  }
}

void BMLogSetDiff::remove_edges(BMesh *bm, blender::Map<int, BMLogEdge *> edges)
{
  for (BMLogEdge *le : edges.values()) {
    BMEdge *e = entry->get_elem_from_id<BMEdge>(le->id);

    if (!e) {
      printf("%s: failed to find edge %d\n", __func__, le->id);
      continue;
    }

    BM_idmap_release(entry->idmap, e, true);
    BM_edge_kill(bm, e);
  }
}

void BMLogSetDiff::swap_edges(BMesh *bm, blender::Map<int, BMLogEdge *> edges)
{
  void *old_customdata = entry->edata.pool ? BLI_mempool_alloc(bm->edata.pool) : nullptr;
  const int cd_id = entry->idmap->cd_id_off[BM_EDGE];

  for (BMLogEdge *le : edges.values()) {
    BMEdge *e = entry->get_elem_from_id<BMEdge>(le->id);

    if (!e) {
      printf("%s: failed to find edge %d\n", __func__, le->id);
      continue;
    }

    if (old_customdata) {
      memcpy(old_customdata, e->head.data, bm->edata.totsize);
    }

    entry->swap_logedge(bm, le->id, e, le);

    /* Ensure id wasn't mangled in customdata swap. */
    BM_ELEM_CD_SET_INT(e, cd_id, le->id);
  }

  if (old_customdata) {
    BLI_mempool_free(bm->edata.pool, old_customdata);
  }
}

void BMLogSetDiff::restore_faces(BMesh *bm, blender::Map<int, BMLogFace *> faces)
{
  Vector<BMVert *, 16> verts;

  for (BMLogFace *lf : faces.values()) {
    bool ok = true;
    verts.clear();

    for (int v_id : lf->verts) {
      BMVert *v = entry->get_elem_from_id<BMVert>(v_id);

      if (!v) {
        printf("%s: Error looking up vertex %d\n", __func__, v_id);
        ok = false;
        continue;
      }

      verts.append(v);
    }

    if (!ok) {
      continue;
    }

    BMFace *f = BM_face_create_verts(bm, verts.data(), verts.size(), nullptr, BM_CREATE_NOP, true);
    f->head.hflag = lf->flag;

    CustomData_bmesh_copy_data(&entry->pdata, &bm->pdata, lf->customdata, &f->head.data);
    entry->assign_elem_id<BMFace>(bm, f, lf->id, true);

    BMLoop *l = f->l_first;
    int i = 0;

    if (lf->loop_customdata[0]) {
      do {
        CustomData_bmesh_copy_data(
            &entry->ldata, &bm->ldata, lf->loop_customdata[i], &l->head.data);
        i++;
      } while ((l = l->next) != f->l_first);
    }
  }

  bm->elem_index_dirty |= BM_FACE;
  bm->elem_table_dirty |= BM_FACE;
}

void BMLogSetDiff::remove_faces(BMesh *bm, blender::Map<int, BMLogFace *> faces)
{
  for (BMLogFace *lf : faces.values()) {
    BMFace *f = entry->get_elem_from_id<BMFace>(lf->id);

    if (!f) {
      printf("%s: error finding face %d\n", __func__, lf->id);
      continue;
    }

    BM_idmap_release(entry->idmap, f, true);
    BM_face_kill(bm, f);
  }

  bm->elem_index_dirty |= BM_FACE;
  bm->elem_table_dirty |= BM_FACE;
}

void BMLogSetDiff::swap_faces(BMesh *bm, blender::Map<int, BMLogFace *> faces)
{
  void *old_customdata = entry->pdata.pool ? BLI_mempool_alloc(bm->pdata.pool) : nullptr;

  const int cd_id = entry->idmap->cd_id_off[BM_FACE];

  for (BMLogFace *lf : faces.values()) {
    BMFace *f = entry->get_elem_from_id<BMFace>(lf->id);

    if (!f) {
      printf("modified_faces: invalid face %d\n", lf->id);
      continue;
    }

    if (old_customdata) {
      memcpy(old_customdata, f->head.data, bm->pdata.totsize);
    }

    entry->swap_logface(bm, lf->id, f, lf);

    /* Ensure id wasn't mangled in customdata swap. */
    BM_ELEM_CD_SET_INT(f, cd_id, lf->id);
  }

  if (old_customdata) {
    BLI_mempool_free(bm->pdata.pool, old_customdata);
  }
}

void BMLogSetDiff::undo(BMesh *bm)
{
  remove_faces(bm, added_faces);
  remove_edges(bm, added_edges);
  remove_verts(bm, added_verts);

  restore_verts(bm, removed_verts);
  restore_edges(bm, removed_edges);
  restore_faces(bm, removed_faces);

  swap_faces(bm, modified_faces);
  swap_edges(bm, modified_edges);
  swap_verts(bm, modified_verts);
}

void BMLogSetDiff::redo(BMesh *bm)
{
  remove_faces(bm, removed_faces);
  remove_edges(bm, removed_edges);
  remove_verts(bm, removed_verts);

  restore_verts(bm, added_verts);
  restore_edges(bm, added_edges);
  restore_faces(bm, added_faces);

  swap_faces(bm, modified_faces);
  swap_edges(bm, modified_edges);
  swap_verts(bm, modified_verts);
}

static BMIdMap *entry_get_idmap(BMLogEntry *entry)
{
  return entry->idmap;
}

BMLog *BM_log_from_existing_entries_create(BMesh *bm, BMLogEntry *entry, bool for_redo)
{
  BMLog *log = BM_log_create(bm);
  log->load_entries(entry);

  if (for_redo) {
    log->current_entry = entry->prev;
  }

  return log;
}

BMLog *BM_log_create(BMesh *bm)
{
  BMLog *log = MEM_new<BMLog>("BMLog", bm);

  return log;
}

bool BM_log_is_dead(BMLog *log)
{
  return log->dead;
}

bool BM_log_free(BMesh *bm, BMLog *log)
{
  BMLogEntry *entry = log->first_entry;

  while (entry) {
    entry->log = nullptr;
    entry = entry->next;
  }

  /* Delete ID attribute layers. */
  BM_idmap_delete_attributes(bm);

  MEM_delete<BMLog>(log);
  return true;
}

BMLogEntry *BM_log_entry_add_delta_set(BMesh *bm, BMLog *log)
{
  if (!log->current_entry) {
    log->push_entry(bm);
  }
  else {
    log->current_entry->push_set(bm, BMLogSetType::LOG_SET_DIFF);
  }

  return log->current_entry;
}

BMLogEntry *BM_log_entry_add(BMesh *bm, BMLog *log)
{
  log->push_entry(bm)->push_set(bm, BMLogSetType::LOG_SET_DIFF);
  return log->current_entry;
}

void BM_log_vert_added(BMesh *bm, BMLog *log, BMVert *v)
{
  /* Forcibly allocate new ID for v, presumably
   * it's existing one is garbage.
   */
  BM_idmap_alloc(log->idmap, v);
  log->add_vert(bm, v);
}

void BM_log_vert_removed(BMesh *bm, BMLog *log, BMVert *v)
{
  log->remove_vert(bm, v);
}

void BM_log_vert_before_modified(BMesh *bm, BMLog *log, BMVert *v)
{
  log->modify_vert(bm, v);
}

BMLogEntry *BM_log_entry_check_customdata(BMesh *bm, BMLog *log)
{
  BMLogEntry *entry = log->current_entry;

  if (!entry) {
    return BM_log_entry_add(bm, log);
  }

  CustomData *cd1[4] = {&bm->vdata, &bm->edata, &bm->ldata, &bm->pdata};
  CustomData *cd2[4] = {&entry->vdata, &entry->edata, &entry->ldata, &entry->pdata};

  for (int i = 0; i < 4; i++) {
    if (!customdata_layout_is_same(cd1[i], cd2[i])) {
      printf("%s: Customdata changed during stroke.\n", __func__);

      entry->cd_layout_changed = true;
      return BM_log_entry_add_delta_set(bm, log);
    }
  }

  return entry;
}

void BM_log_edge_added(BMesh *bm, BMLog *log, BMEdge *e)
{
  /* Forcibly allocate new ID for e presumably
   * it's existing one is garbage.
   */
  BM_idmap_alloc(log->idmap, e);
  log->add_edge(bm, e);
}
void BM_log_edge_modified(BMesh *bm, BMLog *log, BMEdge *e)
{
  log->modify_edge(bm, e);
}
void BM_log_edge_removed(BMesh *bm, BMLog *log, BMEdge *e)
{
  log->remove_edge(bm, e);
}

void BM_log_face_added(BMesh *bm, BMLog *log, BMFace *f)
{
  /* Forcibly allocate new ID for f, presumably
   * it's existing one is garbage.
   */
  BM_idmap_alloc(log->idmap, f);
  log->add_face(bm, f);
}
void BM_log_face_modified(BMesh *bm, BMLog *log, BMFace *f)
{
  log->modify_face(bm, f);
}
void BM_log_face_removed(BMesh *bm, BMLog *log, BMFace *f)
{
  log->remove_face(bm, f);
}

void BM_log_full_mesh(BMesh *bm, BMLog *log)
{
  log->full_mesh(bm);
}

BMVert *BM_log_id_vert_get(BMesh * /*bm*/, BMLog *log, uint id)
{
  return BM_idmap_lookup<BMVert>(log->idmap, id);
}

uint BM_log_vert_id_get(BMesh * /*bm*/, BMLog *log, BMVert *v)
{
  return BM_idmap_get_id(log->idmap, v);
}

BMFace *BM_log_id_face_get(BMesh * /*bm*/, BMLog *log, uint id)
{
  return BM_idmap_lookup<BMFace>(log->idmap, id);
}

uint BM_log_face_id_get(BMesh * /*bm*/, BMLog *log, BMFace *f)
{
  return BM_idmap_get_id(log->idmap, f);
}

void BM_log_undo(BMesh *bm, BMLog *log)
{
  log->undo(bm);
}

void BM_log_redo(BMesh *bm, BMLog *log)
{
  log->redo(bm);
}

void BM_log_undo_skip(BMesh * /*bm*/, BMLog *log)
{
  log->skip(-1);
}

void BM_log_redo_skip(BMesh * /*bm*/, BMLog *log)
{
  log->skip(1);
}

BMLogEntry *BM_log_entry_prev(BMLogEntry *entry)
{
  return entry->prev;
}

BMLogEntry *BM_log_entry_next(BMLogEntry *entry)
{
  return entry->next;
}

void BM_log_set_current_entry(BMLog *log, BMLogEntry *entry)
{
  log->current_entry = entry;
}

bool BM_log_entry_drop(BMLogEntry *entry)
{
  if (entry->prev) {
    entry->prev->next = entry->next;
  }
  if (entry->next) {
    entry->next->prev = entry->prev;
  }

  if (entry->log) {
    if (entry == entry->log->current_entry) {
      entry->log->current_entry = entry->prev;
    }

    if (entry == entry->log->first_entry) {
      entry->log->first_entry = entry->next;
    }
  }

  MEM_delete<BMLogEntry>(entry);
  return true;
}

void BM_log_print_entry(BMLogEntry *entry)
{
  printf("BMLogEntry: %p", entry);
  entry->print();
  printf("\n");
}

void BM_log_original_vert_data(BMLog *log, BMVert *v, const float **co, const float **no)
{
  BLI_assert(log->current_entry);

  BMLogVert *lv = log->current_entry->find_logvert(v);
  BLI_assert(lv);

  *co = lv->co;
  *no = lv->no;
}

const float *BM_log_original_vert_co(BMLog *log, BMVert *v)
{
  BLI_assert(log->current_entry);

  BMLogVert *lv = log->current_entry->find_logvert(v);
  BLI_assert(lv);

  return lv->co;
}

const float *BM_log_original_vert_no(BMLog *log, BMVert *v)
{
  BLI_assert(log->current_entry);

  BMLogVert *lv = log->current_entry->find_logvert(v);
  BLI_assert(lv);

  return lv->no;
}

float BM_log_original_mask(BMLog *log, BMVert *v)
{
  BLI_assert(log->current_entry);

  if (log->current_entry->cd_mask_offset == -1) {
    return 0.0f;
  }

  BMLogVert *lv = log->current_entry->find_logvert(v);
  BLI_assert(lv);

  BMElem log_v;
  log_v.head.data = lv->customdata;

  return BM_ELEM_CD_GET_FLOAT(&log_v, log->current_entry->cd_mask_offset);
}

static int uint_compare(const void *a_v, const void *b_v)
{
  const uint *a = static_cast<const uint *>(a_v);
  const uint *b = static_cast<const uint *>(b_v);
  return (*a) < (*b);
}

/* Remap IDs to contiguous indices
 *
 * E.g. if the vertex IDs are (4, 1, 10, 3), the mapping will be:
 *    4 -> 2
 *    1 -> 0
 *   10 -> 3
 *    3 -> 1
 */
static GHash *bm_log_compress_ids_to_indices(uint *ids, uint totid)
{
  GHash *map = BLI_ghash_int_new_ex(__func__, totid);
  uint i;

  qsort(ids, totid, sizeof(*ids), uint_compare);

  for (i = 0; i < totid; i++) {
    void *key = POINTER_FROM_UINT(ids[i]);
    void *val = POINTER_FROM_UINT(i);
    BLI_ghash_insert(map, key, val);
  }

  return map;
}

void BM_log_mesh_elems_reorder(BMesh *bm, BMLog *log)
{
  uint *varr;
  uint *earr;
  uint *farr;

  GHash *id_to_idx;

  BMIter bm_iter;
  BMVert *v;
  BMEdge *e;
  BMFace *f;

  uint i;

  /* Put all vertex IDs into an array */
  varr = static_cast<uint *>(MEM_mallocN(sizeof(int) * size_t(bm->totvert), __func__));
  BM_ITER_MESH_INDEX (v, &bm_iter, bm, BM_VERTS_OF_MESH, i) {
    varr[i] = BM_idmap_get_id(log->idmap, v);
  }

  /* Put all edge IDs into an array */
  earr = static_cast<uint *>(MEM_mallocN(sizeof(int) * size_t(bm->totedge), __func__));
  BM_ITER_MESH_INDEX (e, &bm_iter, bm, BM_EDGES_OF_MESH, i) {
    earr[i] = BM_idmap_get_id(log->idmap, e);
  }

  /* Put all face IDs into an array */
  farr = static_cast<uint *>(MEM_mallocN(sizeof(int) * size_t(bm->totface), __func__));
  BM_ITER_MESH_INDEX (f, &bm_iter, bm, BM_FACES_OF_MESH, i) {
    farr[i] = BM_idmap_get_id(log->idmap, f);
  }

  /* Create BMVert index remap array */
  id_to_idx = bm_log_compress_ids_to_indices(varr, uint(bm->totvert));
  BM_ITER_MESH_INDEX (v, &bm_iter, bm, BM_VERTS_OF_MESH, i) {
    const uint id = BM_idmap_get_id(log->idmap, v);
    const void *key = POINTER_FROM_UINT(id);
    const void *val = BLI_ghash_lookup(id_to_idx, key);
    varr[i] = POINTER_AS_UINT(val);
  }
  BLI_ghash_free(id_to_idx, nullptr, nullptr);

  /* Create BMEdge index remap array */
  id_to_idx = bm_log_compress_ids_to_indices(earr, uint(bm->totedge));
  BM_ITER_MESH_INDEX (e, &bm_iter, bm, BM_EDGES_OF_MESH, i) {
    const uint id = BM_idmap_get_id(log->idmap, e);
    const void *key = POINTER_FROM_UINT(id);
    const void *val = BLI_ghash_lookup(id_to_idx, key);
    earr[i] = POINTER_AS_UINT(val);
  }
  BLI_ghash_free(id_to_idx, nullptr, nullptr);

  /* Create BMFace index remap array */
  id_to_idx = bm_log_compress_ids_to_indices(farr, uint(bm->totface));
  BM_ITER_MESH_INDEX (f, &bm_iter, bm, BM_FACES_OF_MESH, i) {
    const uint id = BM_idmap_get_id(log->idmap, f);
    const void *key = POINTER_FROM_UINT(id);
    const void *val = BLI_ghash_lookup(id_to_idx, key);
    farr[i] = POINTER_AS_UINT(val);
  }
  BLI_ghash_free(id_to_idx, nullptr, nullptr);

  BM_mesh_remap(bm, varr, earr, farr);
  BM_idmap_check_ids(log->idmap);

  MEM_SAFE_FREE(varr);
  MEM_SAFE_FREE(earr);
  MEM_SAFE_FREE(farr);
}
