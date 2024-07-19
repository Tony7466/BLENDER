/* SPDX-FileCopyrightText: 2016 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#include "DNA_material_types.h"
#include "DNA_object_types.h"
#include "DNA_world_types.h"

#include "BLI_dynstr.h"
#include "BLI_listbase.h"
#include "BLI_map.hh"
#include "BLI_string_utils.hh"
#include "BLI_threads.h"
#include "BLI_time.h"

#include "BKE_context.hh"
#include "BKE_global.hh"
#include "BKE_main.hh"

#include "DEG_depsgraph_query.hh"

#include "GPU_capabilities.hh"
#include "GPU_material.hh"
#include "GPU_shader.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "wm_window.hh"

#include "draw_manager_c.hh"

#include "CLG_log.h"

static CLG_LogRef LOG = {"draw.manager.shader"};

extern "C" char datatoc_gpu_shader_depth_only_frag_glsl[];
extern "C" char datatoc_common_fullscreen_vert_glsl[];

#define USE_DEFERRED_COMPILATION 1

/* -------------------------------------------------------------------- */
/** \name Deferred Compilation (DRW_deferred)
 *
 * Since compiling shader can take a long time, we do it in a non blocking
 * manner in another thread.
 *
 * \{ */

struct DRWShaderCompiler {
  /** Default compilation queue. */
  ListBase queue; /* GPUMaterial */
  SpinLock list_lock;

  /** Optimization queue. */
  ListBase optimize_queue; /* GPUMaterial */

  void *system_gpu_context;
  GPUContext *blender_gpu_context;
  bool own_context;

  std::atomic<bool> stop;
};

static ListBase compilation_threadpool = {};
static DRWShaderCompiler compiler_data = {};

static void *drw_deferred_shader_compilation_exec(void *)
{
  using namespace blender;

  void *system_gpu_context = compiler_data.system_gpu_context;
  GPUContext *blender_gpu_context = compiler_data.blender_gpu_context;
  BLI_assert(system_gpu_context != nullptr);
  BLI_assert(blender_gpu_context != nullptr);

  const bool use_main_context_workaround = GPU_use_main_context_workaround();
  bool is_context_acquired = false;
  auto acquire_context = [&](bool force = false) {
    if ((!use_main_context_workaround && !force) || is_context_acquired) {
      return;
    }
    GPU_render_begin();
    if (use_main_context_workaround) {
      BLI_assert(system_gpu_context == DST.system_gpu_context);
      GPU_context_main_lock();
    }
    WM_system_gpu_context_activate(system_gpu_context);
    GPU_context_active_set(blender_gpu_context);
    is_context_acquired = true;
  };
  auto release_context = [&](bool force = false) {
    if ((!use_main_context_workaround && !force) || !is_context_acquired) {
      return;
    }
    GPU_context_active_set(nullptr);
    WM_system_gpu_context_release(system_gpu_context);
    if (use_main_context_workaround) {
      GPU_context_main_unlock();
    }
    GPU_render_end();
    is_context_acquired = false;
  };

  acquire_context(true);

  const bool use_parallel_compilation = GPU_use_parallel_compilation();
  Vector<GPUMaterial *> next_batch;
  Map<BatchHandle, Vector<GPUMaterial *>> batches;

  while (true) {
    if (compiler_data.stop) {
      break;
    }

    BLI_spin_lock(&compiler_data.list_lock);
    /* Pop tail because it will be less likely to lock the main thread
     * if all GPUMaterials are to be freed (see DRW_deferred_shader_remove()). */
    LinkData *link = (LinkData *)BLI_poptail(&compiler_data.queue);
    GPUMaterial *mat = link ? (GPUMaterial *)link->data : nullptr;
    if (mat) {
      /* Avoid another thread freeing the material mid compilation. */
      GPU_material_acquire(mat);
      MEM_freeN(link);
    }
    BLI_spin_unlock(&compiler_data.list_lock);

    if (mat) {
      acquire_context();
      /* We have a new material that must be compiled,
       * we either compile it directly or add it to a parallel compilation batch. */
      if (use_parallel_compilation) {
        next_batch.append(mat);
      }
      else {
        GPU_material_compile(mat);
        GPU_material_release(mat);
      }
    }
    else if (!next_batch.is_empty()) {
      /* (only if use_parallel_compilation == true)
       * We ran out of pending materials. Request the compilation of the current batch. */
      BatchHandle batch_handle = GPU_material_batch_compile(next_batch);
      batches.add(batch_handle, next_batch);
      next_batch.clear();
    }
    else if (!batches.is_empty()) {
      /* (only if use_parallel_compilation == true)
       * Keep querying the requested batches until all of them are ready. */
      Vector<BatchHandle> ready_handles;
      for (BatchHandle handle : batches.keys()) {
        if (GPU_material_batch_is_ready(handle)) {
          ready_handles.append(handle);
        }
      }
      for (BatchHandle handle : ready_handles) {
        Vector<GPUMaterial *> batch = batches.pop(handle);
        GPU_material_batch_finalize(handle, batch);
        for (GPUMaterial *mat : batch) {
          GPU_material_release(mat);
        }
      }
    }
    else {
      /* Check for Material Optimization job once there are no more
       * shaders to compile. */
      BLI_spin_lock(&compiler_data.list_lock);
      /* Pop tail because it will be less likely to lock the main thread
       * if all GPUMaterials are to be freed (see DRW_deferred_shader_remove()). */
      LinkData *link = (LinkData *)BLI_poptail(&compiler_data.optimize_queue);
      GPUMaterial *optimize_mat = link ? (GPUMaterial *)link->data : nullptr;
      if (optimize_mat) {
        /* Avoid another thread freeing the material during optimization. */
        GPU_material_acquire(optimize_mat);
      }
      BLI_spin_unlock(&compiler_data.list_lock);

      if (optimize_mat) {
        acquire_context();
        /* Compile optimized material shader. */
        GPU_material_optimize(optimize_mat);
        GPU_material_release(optimize_mat);
        MEM_freeN(link);
      }
      else {
        /* No more materials to optimize, or shaders to compile. */
        release_context();
        BLI_time_sleep_ms(1);
      }
    }

    if (is_context_acquired &&
        GPU_type_matches_ex(GPU_DEVICE_ANY, GPU_OS_ANY, GPU_DRIVER_ANY, GPU_BACKEND_OPENGL))
    {
      GPU_flush();
    }
  }

  /* We have to wait until all the requested batches are ready,
   * even if compiler_data.stop is true. */
  for (BatchHandle handle : batches.keys()) {
    Vector<GPUMaterial *> &batch = batches.lookup(handle);
    GPU_material_batch_finalize(handle, batch);
    for (GPUMaterial *mat : batch) {
      GPU_material_release(mat);
    }
  }

  release_context(true);

  return nullptr;
}

void DRW_shader_init()
{
  static bool initialized = false;
  if (initialized) {
    BLI_assert_unreachable();
    return;
  }
  initialized = true;

  const bool use_main_context = GPU_use_main_context_workaround();

  BLI_spin_init(&compiler_data.list_lock);
  compiler_data.stop = false;

  /* Create only one context. */
  compiler_data.own_context = !use_main_context;
  if (use_main_context) {
    compiler_data.system_gpu_context = DST.system_gpu_context;
    compiler_data.blender_gpu_context = DST.blender_gpu_context;
  }
  else {
    compiler_data.system_gpu_context = WM_system_gpu_context_create();
    compiler_data.blender_gpu_context = GPU_context_create(nullptr,
                                                           compiler_data.system_gpu_context);
    GPU_context_active_set(nullptr);
    WM_system_gpu_context_activate(DST.system_gpu_context);
    GPU_context_active_set(DST.blender_gpu_context);
  }

  BLI_threadpool_init(&compilation_threadpool, drw_deferred_shader_compilation_exec, 1);
  BLI_threadpool_insert(&compilation_threadpool, nullptr);
}

void DRW_shader_exit()
{
  compiler_data.stop = true;
  BLI_threadpool_end(&compilation_threadpool);

  BLI_spin_lock(&compiler_data.list_lock);
  LISTBASE_FOREACH (LinkData *, link, &compiler_data.queue) {
    GPU_material_status_set(static_cast<GPUMaterial *>(link->data), GPU_MAT_CREATED);
  }
  LISTBASE_FOREACH (LinkData *, link, &compiler_data.optimize_queue) {
    GPU_material_optimization_status_set(static_cast<GPUMaterial *>(link->data),
                                         GPU_MAT_OPTIMIZATION_READY);
  }
  BLI_freelistN(&compiler_data.queue);
  BLI_freelistN(&compiler_data.optimize_queue);
  BLI_spin_unlock(&compiler_data.list_lock);

  if (compiler_data.own_context) {
    /* Only destroy if the job owns the context. */
    WM_system_gpu_context_activate(compiler_data.system_gpu_context);
    GPU_context_active_set(compiler_data.blender_gpu_context);
    GPU_context_discard(compiler_data.blender_gpu_context);
    WM_system_gpu_context_dispose(compiler_data.system_gpu_context);
  }
}

/**
 * Append either shader compilation or optimization job to deferred queue and
 * ensure shader compilation worker is active.
 * We keep two separate queue's to ensure core compilations always complete before optimization.
 */
static void drw_deferred_queue_append(GPUMaterial *mat, bool is_optimization_job)
{
  /* Add to either compilation or optimization queue. */
  if (is_optimization_job) {
    BLI_assert(GPU_material_optimization_status(mat) != GPU_MAT_OPTIMIZATION_QUEUED);
    GPU_material_optimization_status_set(mat, GPU_MAT_OPTIMIZATION_QUEUED);
    LinkData *node = BLI_genericNodeN(mat);
    BLI_addtail(&compiler_data.optimize_queue, node);
  }
  else {
    GPU_material_status_set(mat, GPU_MAT_QUEUED);
    LinkData *node = BLI_genericNodeN(mat);
    BLI_addtail(&compiler_data.queue, node);
  }
}

static void drw_deferred_shader_add(GPUMaterial *mat, bool deferred)
{
  if (ELEM(GPU_material_status(mat), GPU_MAT_SUCCESS, GPU_MAT_FAILED)) {
    return;
  }

  /* Avoid crashes with RenderDoc on Windows + Nvidia. */
  if (G.debug & G_DEBUG_GPU_RENDERDOC &&
      GPU_type_matches(GPU_DEVICE_NVIDIA, GPU_OS_ANY, GPU_DRIVER_OFFICIAL))
  {
    deferred = false;
  }

  if (GPU_use_main_context_workaround() && !BLI_thread_is_main()) {
    deferred = false;
  }

  if (!deferred) {
    DRW_deferred_shader_remove(mat);
    /* Shaders could already be compiling. Have to wait for compilation to finish. */
    while (GPU_material_status(mat) == GPU_MAT_QUEUED) {
      BLI_time_sleep_ms(20);
    }
    if (GPU_material_status(mat) == GPU_MAT_CREATED) {
      GPU_material_compile(mat);
    }
    return;
  }

  /* Don't add material to the queue twice. */
  if (GPU_material_status(mat) == GPU_MAT_QUEUED) {
    return;
  }

  /* Add deferred shader compilation to queue. */
  drw_deferred_queue_append(mat, false);
}

static void drw_register_shader_vlattrs(GPUMaterial *mat)
{
  const ListBase *attrs = GPU_material_layer_attributes(mat);

  if (!attrs) {
    return;
  }

  GHash *hash = DST.vmempool->vlattrs_name_cache;
  ListBase *list = &DST.vmempool->vlattrs_name_list;

  LISTBASE_FOREACH (GPULayerAttr *, attr, attrs) {
    GPULayerAttr **p_val;

    /* Add to the table and list if newly seen. */
    if (!BLI_ghash_ensure_p(hash, POINTER_FROM_UINT(attr->hash_code), (void ***)&p_val)) {
      DST.vmempool->vlattrs_ubo_ready = false;

      GPULayerAttr *new_link = *p_val = static_cast<GPULayerAttr *>(MEM_dupallocN(attr));

      /* Insert into the list ensuring sorted order. */
      GPULayerAttr *link = static_cast<GPULayerAttr *>(list->first);

      while (link && link->hash_code <= attr->hash_code) {
        link = link->next;
      }

      new_link->prev = new_link->next = nullptr;
      BLI_insertlinkbefore(list, link, new_link);
    }

    /* Reset the unused frames counter. */
    (*p_val)->users = 0;
  }
}

void DRW_deferred_shader_remove(GPUMaterial *mat)
{
  BLI_spin_lock(&compiler_data.list_lock);

  /* Search for compilation job in queue. */
  LinkData *link = (LinkData *)BLI_findptr(&compiler_data.queue, mat, offsetof(LinkData, data));
  if (link) {
    BLI_remlink(&compiler_data.queue, link);
    GPU_material_status_set(static_cast<GPUMaterial *>(link->data), GPU_MAT_CREATED);
  }

  MEM_SAFE_FREE(link);

  /* Search for optimization job in queue. */
  LinkData *opti_link = (LinkData *)BLI_findptr(
      &compiler_data.optimize_queue, mat, offsetof(LinkData, data));
  if (opti_link) {
    BLI_remlink(&compiler_data.optimize_queue, opti_link);
    GPU_material_optimization_status_set(static_cast<GPUMaterial *>(opti_link->data),
                                         GPU_MAT_OPTIMIZATION_READY);
  }
  BLI_spin_unlock(&compiler_data.list_lock);

  MEM_SAFE_FREE(opti_link);
}

void DRW_deferred_shader_optimize_remove(GPUMaterial *mat)
{
  BLI_spin_lock(&compiler_data.list_lock);
  /* Search for optimization job in queue. */
  LinkData *opti_link = (LinkData *)BLI_findptr(
      &compiler_data.optimize_queue, mat, offsetof(LinkData, data));
  if (opti_link) {
    BLI_remlink(&compiler_data.optimize_queue, opti_link);
    GPU_material_optimization_status_set(static_cast<GPUMaterial *>(opti_link->data),
                                         GPU_MAT_OPTIMIZATION_READY);
  }
  BLI_spin_unlock(&compiler_data.list_lock);

  MEM_SAFE_FREE(opti_link);
}

/** \} */

/* -------------------------------------------------------------------- */

/** \{ */

GPUShader *DRW_shader_create_from_info_name(const char *info_name)
{
  return GPU_shader_create_from_info_name(info_name);
}

GPUShader *DRW_shader_create_ex(
    const char *vert, const char *geom, const char *frag, const char *defines, const char *name)
{
  return GPU_shader_create(vert, frag, geom, nullptr, defines, name);
}

GPUShader *DRW_shader_create_with_lib_ex(const char *vert,
                                         const char *geom,
                                         const char *frag,
                                         const char *lib,
                                         const char *defines,
                                         const char *name)
{
  GPUShader *sh;
  char *vert_with_lib = nullptr;
  char *frag_with_lib = nullptr;
  char *geom_with_lib = nullptr;

  vert_with_lib = BLI_string_joinN(lib, vert);
  frag_with_lib = BLI_string_joinN(lib, frag);
  if (geom) {
    geom_with_lib = BLI_string_joinN(lib, geom);
  }

  sh = GPU_shader_create(vert_with_lib, frag_with_lib, geom_with_lib, nullptr, defines, name);

  MEM_freeN(vert_with_lib);
  MEM_freeN(frag_with_lib);
  if (geom) {
    MEM_freeN(geom_with_lib);
  }

  return sh;
}

GPUShader *DRW_shader_create_with_shaderlib_ex(const char *vert,
                                               const char *geom,
                                               const char *frag,
                                               const DRWShaderLibrary *lib,
                                               const char *defines,
                                               const char *name)
{
  GPUShader *sh;
  char *vert_with_lib = DRW_shader_library_create_shader_string(lib, vert);
  char *frag_with_lib = DRW_shader_library_create_shader_string(lib, frag);
  char *geom_with_lib = (geom) ? DRW_shader_library_create_shader_string(lib, geom) : nullptr;

  sh = GPU_shader_create(vert_with_lib, frag_with_lib, geom_with_lib, nullptr, defines, name);

  MEM_SAFE_FREE(vert_with_lib);
  MEM_SAFE_FREE(frag_with_lib);
  MEM_SAFE_FREE(geom_with_lib);

  return sh;
}

GPUShader *DRW_shader_create_with_transform_feedback(const char *vert,
                                                     const char *geom,
                                                     const char *defines,
                                                     const eGPUShaderTFBType prim_type,
                                                     const char **varying_names,
                                                     const int varying_count)
{
  return GPU_shader_create_ex(vert,
                              datatoc_gpu_shader_depth_only_frag_glsl,
                              geom,
                              nullptr,
                              nullptr,
                              defines,
                              prim_type,
                              varying_names,
                              varying_count,
                              __func__);
}

GPUShader *DRW_shader_create_fullscreen_ex(const char *frag, const char *defines, const char *name)
{
  return GPU_shader_create(
      datatoc_common_fullscreen_vert_glsl, frag, nullptr, nullptr, defines, name);
}

GPUShader *DRW_shader_create_fullscreen_with_shaderlib_ex(const char *frag,
                                                          const DRWShaderLibrary *lib,
                                                          const char *defines,
                                                          const char *name)
{

  GPUShader *sh;
  char *vert = datatoc_common_fullscreen_vert_glsl;
  char *frag_with_lib = DRW_shader_library_create_shader_string(lib, frag);

  sh = GPU_shader_create(vert, frag_with_lib, nullptr, nullptr, defines, name);

  MEM_SAFE_FREE(frag_with_lib);

  return sh;
}

GPUMaterial *DRW_shader_from_world(World *wo,
                                   bNodeTree *ntree,
                                   eGPUMaterialEngine engine,
                                   const uint64_t shader_id,
                                   const bool is_volume_shader,
                                   bool deferred,
                                   GPUCodegenCallbackFn callback,
                                   void *thunk)
{
  Scene *scene = (Scene *)DEG_get_original_id(&DST.draw_ctx.scene->id);
  GPUMaterial *mat = GPU_material_from_nodetree(scene,
                                                nullptr,
                                                ntree,
                                                &wo->gpumaterial,
                                                wo->id.name,
                                                engine,
                                                shader_id,
                                                is_volume_shader,
                                                false,
                                                callback,
                                                thunk);

  drw_register_shader_vlattrs(mat);

  if (DRW_state_is_image_render()) {
    /* Do not deferred if doing render. */
    deferred = false;
  }

  drw_deferred_shader_add(mat, deferred);
  DRW_shader_queue_optimize_material(mat);
  return mat;
}

GPUMaterial *DRW_shader_from_material(Material *ma,
                                      bNodeTree *ntree,
                                      eGPUMaterialEngine engine,
                                      const uint64_t shader_id,
                                      const bool is_volume_shader,
                                      bool deferred,
                                      GPUCodegenCallbackFn callback,
                                      void *thunk,
                                      GPUMaterialPassReplacementCallbackFn pass_replacement_cb)
{
  Scene *scene = (Scene *)DEG_get_original_id(&DST.draw_ctx.scene->id);
  GPUMaterial *mat = GPU_material_from_nodetree(scene,
                                                ma,
                                                ntree,
                                                &ma->gpumaterial,
                                                ma->id.name,
                                                engine,
                                                shader_id,
                                                is_volume_shader,
                                                false,
                                                callback,
                                                thunk,
                                                pass_replacement_cb);

  drw_register_shader_vlattrs(mat);

  drw_deferred_shader_add(mat, deferred);
  DRW_shader_queue_optimize_material(mat);
  return mat;
}

void DRW_shader_queue_optimize_material(GPUMaterial *mat)
{
  /* Do not perform deferred optimization if performing render.
   * De-queue any queued optimization jobs. */
  if (DRW_state_is_image_render()) {
    if (GPU_material_optimization_status(mat) == GPU_MAT_OPTIMIZATION_QUEUED) {
      /* Remove from pending optimization job queue. */
      DRW_deferred_shader_optimize_remove(mat);
      /* If optimization job had already started, wait for it to complete. */
      while (GPU_material_optimization_status(mat) == GPU_MAT_OPTIMIZATION_QUEUED) {
        BLI_time_sleep_ms(20);
      }
    }
    return;
  }

  /* We do not need to perform optimization on the material if it is already compiled or in the
   * optimization queue. If optimization is not required, the status will be flagged as
   * `GPU_MAT_OPTIMIZATION_SKIP`.
   * We can also skip cases which have already been queued up. */
  if (ELEM(GPU_material_optimization_status(mat),
           GPU_MAT_OPTIMIZATION_SKIP,
           GPU_MAT_OPTIMIZATION_SUCCESS,
           GPU_MAT_OPTIMIZATION_QUEUED))
  {
    return;
  }

  /* Only queue optimization once the original shader has been successfully compiled. */
  if (GPU_material_status(mat) != GPU_MAT_SUCCESS) {
    return;
  }

  /* Defer optimization until sufficient time has passed beyond creation. This avoids excessive
   * recompilation for shaders which are being actively modified. */
  if (!GPU_material_optimization_ready(mat)) {
    return;
  }

  /* Add deferred shader compilation to queue. */
  drw_deferred_queue_append(mat, true);
}

void DRW_shader_free(GPUShader *shader)
{
  GPU_shader_free(shader);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Shader Library
 *
 * Simple include system for glsl files.
 *
 * Usage: Create a DRWShaderLibrary and add the library in the right order.
 * You can have nested dependencies but each new library needs to have all its dependencies already
 * added to the DRWShaderLibrary.
 * Finally you can use DRW_shader_library_create_shader_string to get a shader string that also
 * contains the needed libraries for this shader.
 * \{ */

/* 64 because we use a 64bit bitmap. */
#define MAX_LIB 64
#define MAX_LIB_NAME 64
#define MAX_LIB_DEPS 8

struct DRWShaderLibrary {
  const char *libs[MAX_LIB];
  char libs_name[MAX_LIB][MAX_LIB_NAME];
  uint64_t libs_deps[MAX_LIB];
};

DRWShaderLibrary *DRW_shader_library_create()
{
  return static_cast<DRWShaderLibrary *>(
      MEM_callocN(sizeof(DRWShaderLibrary), "DRWShaderLibrary"));
}

void DRW_shader_library_free(DRWShaderLibrary *lib)
{
  MEM_SAFE_FREE(lib);
}

static int drw_shader_library_search(const DRWShaderLibrary *lib, const char *name)
{
  for (int i = 0; i < MAX_LIB; i++) {
    if (lib->libs[i]) {
      if (!strncmp(lib->libs_name[i], name, strlen(lib->libs_name[i]))) {
        return i;
      }
    }
    else {
      break;
    }
  }
  return -1;
}

/* Return bitmap of dependencies. */
static uint64_t drw_shader_dependencies_get(const DRWShaderLibrary *lib,
                                            const char *pragma_str,
                                            const char *lib_code,
                                            const char * /*lib_name*/)
{
  /* Search dependencies. */
  uint pragma_len = strlen(pragma_str);
  uint64_t deps = 0;
  const char *haystack = lib_code;
  while ((haystack = strstr(haystack, pragma_str))) {
    haystack += pragma_len;
    int dep = drw_shader_library_search(lib, haystack);
    if (dep == -1) {
      char dbg_name[MAX_NAME];
      int i = 0;
      while ((*haystack != ')') && (i < (sizeof(dbg_name) - 2))) {
        dbg_name[i] = *haystack;
        haystack++;
        i++;
      }
      dbg_name[i] = '\0';

      CLOG_INFO(&LOG,
                0,
                "Dependency '%s' not found\n"
                "This might be due to bad lib ordering or overriding a builtin shader.\n",
                dbg_name);
    }
    else {
      deps |= 1llu << uint64_t(dep);
    }
  }
  return deps;
}

void DRW_shader_library_add_file(DRWShaderLibrary *lib, const char *lib_code, const char *lib_name)
{
  int index = -1;
  for (int i = 0; i < MAX_LIB; i++) {
    if (lib->libs[i] == nullptr) {
      index = i;
      break;
    }
  }

  if (index > -1) {
    lib->libs[index] = lib_code;
    STRNCPY(lib->libs_name[index], lib_name);
    lib->libs_deps[index] = drw_shader_dependencies_get(
        lib, "BLENDER_REQUIRE(", lib_code, lib_name);
  }
  else {
    printf("Error: Too many libraries. Cannot add %s.\n", lib_name);
    BLI_assert(0);
  }
}

char *DRW_shader_library_create_shader_string(const DRWShaderLibrary *lib, const char *shader_code)
{
  uint64_t deps = drw_shader_dependencies_get(lib, "BLENDER_REQUIRE(", shader_code, "shader code");

  DynStr *ds = BLI_dynstr_new();
  /* Add all dependencies recursively. */
  for (int i = MAX_LIB - 1; i > -1; i--) {
    if (lib->libs[i] && (deps & (1llu << uint64_t(i)))) {
      deps |= lib->libs_deps[i];
    }
  }
  /* Concatenate all needed libs into one string. */
  for (int i = 0; i < MAX_LIB && deps != 0llu; i++, deps >>= 1llu) {
    if (deps & 1llu) {
      BLI_dynstr_append(ds, lib->libs[i]);
    }
  }

  BLI_dynstr_append(ds, shader_code);

  char *str = BLI_dynstr_get_cstring(ds);
  BLI_dynstr_free(ds);

  return str;
}

/** \} */
