#include "DRW_render.h"
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

void OVERLAY_onion_skin_populate(OVERLAY_Data *vedata, Object *ob)
{
  OVERLAY_PrivateData *pd = vedata->stl->pd;
  struct GPUBatch *geom = DRW_cache_object_surface_get(ob);
  if (geom) {
    DRW_shgroup_call(pd->onion_skin_grp, geom, ob);
  }
}

void OVERLAY_onion_skin_draw(OVERLAY_Data *vedata)
{
  OVERLAY_PassList *psl = vedata->psl;
  DRW_draw_pass(psl->onion_skin_ps);
}