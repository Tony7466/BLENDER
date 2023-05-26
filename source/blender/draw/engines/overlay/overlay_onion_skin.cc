#include "BKE_mesh.h"
#include "DRW_render.h"
#include "draw_cache_impl.h"
#include "overlay_private.hh"

void OVERLAY_onion_skin_init(OVERLAY_Data *vedata)
{
  OVERLAY_PassList *psl = vedata->psl;
  OVERLAY_PrivateData *pd = vedata->stl->pd;

  DRWState state = DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA;
  DRW_PASS_CREATE(psl->onion_skin_ps, state | pd->clipping_state);

  GPUShader *shader = OVERLAY_shader_onion_skin_mesh();
  DRWShadingGroup *grp;
  pd->onion_skin_grp = grp = DRW_shgroup_create(shader, psl->onion_skin_ps);

  DRW_shgroup_uniform_block(grp, "globalsBlock", G_draw.block_ubo);
}

static bool str_equals(const char *__restrict str, const char *__restrict start)
{
  for (; *str && *start; str++, start++) {
    if (*str != *start) {
      return false;
    }
  }

  return (*start == *str);
}

void OVERLAY_onion_skin_populate(OVERLAY_Data *vedata, Object *ob)
{
  OVERLAY_PrivateData *pd = vedata->stl->pd;
  DRWShadingGroup *grp = pd->onion_skin_grp;

  const DRWContextState *draw_ctx = DRW_context_state_get();

  Scene *scene = draw_ctx->scene;
  const float current_frame = BKE_scene_ctime_get(scene);

  DRW_shgroup_uniform_float_copy(grp, "alpha", draw_ctx->scene->onion_skin_cache.alpha);

  LISTBASE_FOREACH (OnionSkinMeshLink *, mesh_link, &scene->onion_skin_cache.objects) {
    if (!str_equals(ob->id.name, mesh_link->object->id.name)) {
      continue;
    }

    if (compare_ff(mesh_link->frame, current_frame, FLT_EPSILON)) {
      continue;
    }

    if (mesh_link->frame < current_frame - scene->onion_skin_cache.relative_left) {
      continue;
    }

    if (mesh_link->frame > current_frame + scene->onion_skin_cache.relative_right) {
      continue;
    }
    if (mesh_link->frame < current_frame) {
      DRW_shgroup_uniform_vec3_copy(grp, "color", draw_ctx->scene->onion_skin_cache.color_left);
    }
    else {
      DRW_shgroup_uniform_vec3_copy(grp, "color", draw_ctx->scene->onion_skin_cache.color_right);
    }

    struct GPUBatch *geom = DRW_cache_object_surface_get(ob);
    if (geom) {
      DRW_shgroup_call(grp, geom, ob);
    }
  }
}

void OVERLAY_onion_skin_draw(OVERLAY_Data *vedata)
{
  OVERLAY_PassList *psl = vedata->psl;
  DRW_draw_pass(psl->onion_skin_ps);
}