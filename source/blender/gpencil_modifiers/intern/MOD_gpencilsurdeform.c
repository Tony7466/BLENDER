

/** \file
 * \ingroup modifiers
 */

#include <stdio.h>

#include "BLI_string.h"

#include "BLI_listbase.h"
#include "BLI_math_geom.h"
#include "BLI_math_vector.h"
#include "BLI_utildefines.h"

#include "BLT_translation.h"

#include "DNA_defaults.h"
#include "DNA_gpencil_modifier_types.h"
#include "DNA_gpencil_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "BKE_scene.h"
#include "BKE_context.h"
#include "BKE_deform.h"
#include "BKE_gpencil.h"
#include "BKE_gpencil_geom.h"
#include "BKE_gpencil_modifier.h"
#include "BKE_lib_query.h"
#include "BKE_modifier.h"
#include "BKE_screen.h"

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_build.h"
#include "DEG_depsgraph_query.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "RNA_access.h"

#include "MOD_gpencil_modifiertypes.h"
#include "MOD_gpencil_ui_common.h"
#include "MOD_gpencil_util.h"

#include "MEM_guardedalloc.h"

#include "WM_api.h"

/* HEADER FROM MOD_surfacedeform.c */

#include "BLI_alloca.h"
#include "BLI_math.h"
#include "BLI_task.h"


#include "DNA_mesh_types.h"

#include "BKE_bvhutils.h"
#include "BKE_editmesh.h"
#include "BKE_lib_id.h"
#include "BKE_mesh.h"
#include "BKE_mesh_runtime.h"
#include "BKE_mesh_wrapper.h"


/*#include "BLO_read_write.h"*/

#include "RNA_prototypes.h"



typedef struct SDefDeformData {
  const SDefGPVert *const bind_verts;
  float (*const targetCos)[3];
  float (*const vertexCos)[3];
  const MDeformVert *const dvert;
  int const defgrp_index;
  bool const invert_vgroup;
  float const strength;
  bGPDstroke *gps;
} SDefDeformData;

/* HEADER FUNCS */

void rollback_layers(SurDeformGpencilModifierData *smd)
{
  if (smd->layers == NULL)
    return;
  smd->layers = smd->layers->first;

}

static void rollback_frames(SurDeformGpencilModifierData *smd, SDefGPLayer *layer)
{
  if (layer->frames == NULL)
    return;
  layer->frames = layer->frames->first;
  return;
  
}

static void rollback_strokes(SurDeformGpencilModifierData *smd, SDefGPFrame *frame)
{
  
  if (frame->strokes == NULL) return;
  frame->strokes = frame->strokes->first;
  return;
  
}
struct SurDeformGpencilModifierData *get_original_modifier(Object *ob, SurDeformGpencilModifierData *smd, GpencilModifierData *md)
{
  Object *object_orig = DEG_get_original_object(ob);
  SurDeformGpencilModifierData *smd_orig;
  GpencilModifierData *md_orig;
  

  if (object_orig == ob) {
    smd_orig = smd;
    md_orig = md;  }
  else {
  /*smd_orig = (SurDeformGpencilModifierData *)BKE_gpencil_modifiers_findby_name(object_orig, md->name);}*/
  md_orig = BKE_gpencil_modifiers_findby_name(object_orig, md->name);
  smd_orig = (SurDeformGpencilModifierData *)md_orig;}

  return smd_orig;
}


/*Free a single frame*/
static bool free_frame_b(SurDeformGpencilModifierData *smd_orig,
                       SurDeformGpencilModifierData *smd_eval,
                       SDefGPLayer *sdef_layer,
                       uint framenum)
{
  if (sdef_layer->frames == NULL)
    return true;
  /* If we're not on the first frame, rollback the pointer*/
  if (sdef_layer->frames->frame_idx > 0) {
    rollback_frames(smd_orig, sdef_layer);
  }
  /*Do the same thing as add, but in reverse */
  sdef_layer->num_of_frames--;
  SDefGPFrame *temp_frames_pointer = MEM_calloc_arrayN(
      sdef_layer->num_of_frames, sizeof(*sdef_layer->frames), "SDefGPFrames");

  if (temp_frames_pointer == NULL) {
    BKE_gpencil_modifier_set_error((GpencilModifierData *)smd_eval, "Out of memory");
    return false;
  }
  SDefGPFrame *first_frame = temp_frames_pointer;
  /*if this was the last frame, so num_of_frame has just become 0, we don't need to copy
   * anything.*/
  if (sdef_layer->num_of_frames > 0) {
    /* Copy one frame at a time, except the one we are unbinding*/
    int g = 0;
    for (int f = 0; f < sdef_layer->num_of_frames; f++) {
      if (sdef_layer->frames[g].frame_number == framenum) {
        g++;
        f--;
        continue;
      }
      memcpy(&temp_frames_pointer[f], &(sdef_layer->frames[g]), sizeof(*sdef_layer->frames));
      g++;
    }
    MEM_SAFE_FREE(sdef_layer->frames);
    sdef_layer->frames = first_frame;
    /*Set the first frame again if the first was the one being removed*/
    if (sdef_layer->frames->first != first_frame) {
      for (int f = 0; f < sdef_layer->num_of_frames; f++) {
        sdef_layer->frames[f].first = first_frame;
      }
    }
  }
  else
    MEM_SAFE_FREE(sdef_layer->frames);

  return true;
}

/* HEADER END */

static void initData(GpencilModifierData *md)
{
  SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(smd, modifier));

  MEMCPY_STRUCT_AFTER(smd, DNA_struct_default_get(SurDeformGpencilModifierData), modifier);

}

static void freeData(GpencilModifierData *md)
{

  SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;
  if (smd->layers)
  {
    rollback_layers(smd);
    for (int l= 0; l < smd->num_of_layers; l++)
    {
      if (smd->layers[l].frames)
      {
        rollback_frames(smd, &smd->layers[l]);
        for(int m= 0; m < smd->layers[l].num_of_frames; m++)
        {
          if (smd->layers[l].frames[m].strokes) 
          {
            rollback_strokes(smd, &smd->layers[l].frames[m] );
            for (int k= 0; k < smd->layers[l].frames[m].strokes_num; k++){
              if (smd->layers[l].frames[m].strokes[k].verts) {
                for (int i = 0; i < smd->layers[l].frames[m].strokes[k].stroke_verts_num; i++) {
                  if (smd->layers[l].frames[m].strokes[k].verts[i].binds) {
                    for (int j = 0; j < smd->layers[l].frames[m].strokes[k].verts[i].binds_num; j++) {
                      MEM_SAFE_FREE(smd->layers[l].frames[m].strokes[k].verts[i].binds[j].vert_inds);
                      MEM_SAFE_FREE(smd->layers[l].frames[m].strokes[k].verts[i].binds[j].vert_weights);
                    }

                    MEM_SAFE_FREE(smd->layers[l].frames[m].strokes[k].verts[i].binds);
                  }
                }
              }
              MEM_SAFE_FREE(smd->layers[l].frames[m].strokes[k].verts);
              }
            MEM_SAFE_FREE(smd->layers[l].frames[m].strokes);
          }
        }
        MEM_SAFE_FREE(smd->layers[l].frames);
      }
    }
    MEM_SAFE_FREE(smd->layers); 
  }
  smd->num_of_layers = 0;
  smd->bound_flags = 0;

}

static void copyData(const GpencilModifierData *md, GpencilModifierData *target)
{
  SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;
  SurDeformGpencilModifierData *tsmd = (SurDeformGpencilModifierData *)target;

  BKE_gpencil_modifier_copydata_generic(md, target);
  tsmd->bound_flags = smd->bound_flags;
  if (smd->layers)
  { 
    rollback_layers(smd);
    tsmd->layers = MEM_dupallocN(smd->layers);
    for(int l = 0; l < smd->num_of_layers; l++)
    {
      tsmd->layers[l].first = tsmd->layers;
      if (smd->layers[l].frames)
      {
        rollback_frames(smd, &smd->layers[l]);
        tsmd->layers[l].frames = MEM_dupallocN(smd->layers[l].frames);
        for (int f = 0; f < smd->layers[l].num_of_frames; f++)
        {
          tsmd->layers[l].frames[f].first = tsmd->layers[l].frames;
          if (smd->layers[l].frames[f].strokes)  
          {
            rollback_strokes(smd, &smd->layers[l].frames[f] );
            tsmd->layers[l].frames[f].strokes = MEM_dupallocN(smd->layers[l].frames[f].strokes);

            for(int k = 0; k < smd->layers[l].frames[f].strokes_num; k++)
            {
              tsmd->layers[l].frames[f].strokes[k].first = tsmd->layers[l].frames[f].strokes;
              if (smd->layers[l].frames[f].strokes[k].verts) 
              {
                tsmd->layers[l].frames[f].strokes[k].verts = MEM_dupallocN(smd->layers[l].frames[f].strokes[k].verts);

                for (int i = 0; i < smd->layers[l].frames[f].strokes[k].stroke_verts_num; i++) 
                {
                  if (smd->layers[l].frames[f].strokes[k].verts[i].binds) 
                  {
                    tsmd->layers[l].frames[f].strokes[k].verts[i].binds = MEM_dupallocN(smd->layers[l].frames[f].strokes[k].verts[i].binds);
                    for (int j = 0; j < smd->layers[l].frames[f].strokes[k].verts[i].binds_num; j++) 
                    {
                      if (smd->layers[l].frames[f].strokes[k].verts[i].binds[j].vert_inds) 
                      {
                        tsmd->layers[l].frames[f].strokes[k].verts[i].binds[j].vert_inds = MEM_dupallocN(
                            smd->layers[l].frames[f].strokes[k].verts[i].binds[j].vert_inds);
                      }

                      if (smd->layers[l].frames[f].strokes[k].verts[i].binds[j].vert_weights) 
                      {
                        tsmd->layers[l].frames[f].strokes[k].verts[i].binds[j].vert_weights = MEM_dupallocN(
                            smd->layers[l].frames[f].strokes[k].verts[i].binds[j].vert_weights);
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}

static bool dependsOnTime(GpencilModifierData *md)

{
  return true;
}

static void updateDepsgraph(GpencilModifierData *md, const ModifierUpdateDepsgraphContext *ctx, const int UNUSED(mode))
{
  SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;
  if (smd->target != NULL) {
    DEG_add_object_relation(
        ctx->node, smd->target, DEG_OB_COMP_GEOMETRY, "Surface Deform GP Modifier");
    DEG_add_object_relation(
        ctx->node, smd->target, DEG_OB_COMP_TRANSFORM, "Surface Deform GP Modifier");
  }
}

/*Set the binding combinations flags.*/
static void check_bind_situation(SurDeformGpencilModifierData *smd,
                                Depsgraph *depsgraph,
                                Scene *scene,
                                Object *ob)
{
  smd->bound_flags = 0;
  if (smd->layers == NULL) 
  {return;}
  rollback_layers(smd);

  
  smd->bound_flags |= GP_MOD_SDEF_SOMETHING_BOUND;

  Object *ob_orig = (Object *)DEG_get_original_id(&ob->id);
  bGPdata *gpd = ob_orig->data;
  bGPDlayer *gpl_active = BKE_gpencil_layer_active_get(gpd);
  bGPDlayer *blender_layer = NULL;
  //bGPDframe *blender_frame;

  int num_of_layers_with_all_their_frames_bound = 0;
  int num_of_layers_with_their_curr_frame_bound = 0;

  for (int l = 0; l < smd->num_of_layers; l++)
  {
    LISTBASE_FOREACH(bGPDlayer *, curr_gpl, &gpd->layers)
    {
      if (!strcmp(curr_gpl->info, smd->layers[l].layer_info))
      {
        blender_layer = curr_gpl;
        break;
      }
    }
     if (blender_layer == NULL) 
    {
      printf("NULL blender LAYER IN CHECK bind situation");
      continue;
    }
    if (BLI_listbase_is_empty(&blender_layer->frames)) continue;
    if (smd->layers[l].num_of_frames == BLI_listbase_count(&(blender_layer->frames)))
    {
      /* layer layers[l] has all of its frames bound*/
      num_of_layers_with_all_their_frames_bound++;
      if (blender_layer == gpl_active)
      {smd->bound_flags |= GP_MOD_SDEF_CURRENT_LAYER_ALL_FRAMES_BOUND;}
    }
    rollback_frames(smd, &(smd->layers[l]));
    
    for (int f = 0; f < smd->layers[l].num_of_frames; f++)
    {
      smd->layers[l].frames[f].blender_frame = NULL;
      bGPDframe *blender_frame = BKE_gpencil_frame_retime_get(depsgraph, scene, ob, blender_layer);
      if (blender_frame->framenum
          == smd->layers[l].frames[f].frame_number    )
      {
        smd->layers[l].frames[f].blender_frame = blender_frame;
        if (blender_layer == gpl_active)
        {smd->bound_flags |= GP_MOD_SDEF_CURRENT_LAYER_CURRENT_FRAME_BOUND;}
        num_of_layers_with_their_curr_frame_bound++;
        break;
      }
    }
    

  }
  uint totlayers = BLI_listbase_count(&(gpd->layers));
  if   ((smd->num_of_layers == totlayers) &&
        (num_of_layers_with_all_their_frames_bound == smd->num_of_layers))
  {smd->bound_flags |= GP_MOD_SDEF_ALL_LAYERS_AND_FRAMES_BOUND;}
  
    
  if   ((smd->num_of_layers == totlayers) &&
        (num_of_layers_with_their_curr_frame_bound == smd->num_of_layers))
      {smd->bound_flags |= GP_MOD_SDEF_ALL_LAYERS_CURRENT_FRAMES_BOUND;}
    
  
  
}


static void deformVert(void *__restrict userdata,
                       const int index,
                       const TaskParallelTLS *__restrict UNUSED(tls))
{
  const SDefDeformData *const data = (SDefDeformData *)userdata;
  const SDefGPBind *sdbind = data->bind_verts[index].binds;
  const int sdbind_num = data->bind_verts[index].binds_num;
  const unsigned int vertex_idx = data->bind_verts[index].vertex_idx;
  float *vertexCos = &(data->gps->points[vertex_idx].x);
  float norm[3], temp[3], offset[3];
  float tmp_mat_err[4][4] = {
      {1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 1.0, -1.0}, {0.0, -1.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};

  /* Retrieve the value of the weight vertex group if specified. */
  float weight = 1.0f;
  
  if (data->dvert && data->defgrp_index != -1) {
    weight = get_modifier_point_weight(&data->dvert[vertex_idx], data->invert_vgroup, data->defgrp_index);

  }
  /* Check if this vertex will be deformed. If it is not deformed we return and avoid
   * unnecessary calculations. */
  if (weight <= 0.0f) {
    return;
  }

  zero_v3(offset);
  
  /* Allocate a `coords_buffer` that fits all the temp-data. */
  int max_verts = 0;
  for (int j = 0; j < sdbind_num; j++) {
    max_verts = MAX2(max_verts, sdbind[j].verts_num);
  }


  const bool big_buffer = max_verts > 256;
  float(*coords_buffer)[3];
  if (UNLIKELY(big_buffer)) {
    coords_buffer = MEM_malloc_arrayN(max_verts, sizeof(*coords_buffer), __func__);
  }
  else {
    coords_buffer = BLI_array_alloca(coords_buffer, max_verts);
  }
  
  for (int j = 0; j < sdbind_num; j++, sdbind++) {
    for (int k = 0; k < sdbind->verts_num; k++) {
      copy_v3_v3(coords_buffer[k], data->targetCos[sdbind->vert_inds[k]]);
    }

    normal_poly_v3(norm, coords_buffer, sdbind->verts_num);
    zero_v3(temp);

    switch (sdbind->mode) {
      /* ---------- looptri mode ---------- */
      case MOD_SDEF_MODE_LOOPTRI: {
        madd_v3_v3fl(temp, data->targetCos[sdbind->vert_inds[0]], sdbind->vert_weights[0]);
        madd_v3_v3fl(temp, data->targetCos[sdbind->vert_inds[1]], sdbind->vert_weights[1]);
        madd_v3_v3fl(temp, data->targetCos[sdbind->vert_inds[2]], sdbind->vert_weights[2]);
        break;
      }

      /* ---------- ngon mode ---------- */
      case MOD_SDEF_MODE_NGON: {
        for (int k = 0; k < sdbind->verts_num; k++) {
          madd_v3_v3fl(temp, coords_buffer[k], sdbind->vert_weights[k]);
        }
        break;
      }

      /* ---------- centroid mode ---------- */
      case MOD_SDEF_MODE_CENTROID: {
        float cent[3];
        mid_v3_v3_array(cent, coords_buffer, sdbind->verts_num);

        madd_v3_v3fl(temp, data->targetCos[sdbind->vert_inds[0]], sdbind->vert_weights[0]);
        madd_v3_v3fl(temp, data->targetCos[sdbind->vert_inds[1]], sdbind->vert_weights[1]);
        madd_v3_v3fl(temp, cent, sdbind->vert_weights[2]);
        break;
      }
    }

    /* Apply normal offset (generic for all modes) */
    madd_v3_v3fl(temp, norm, sdbind->normal_dist);

    madd_v3_v3fl(offset, temp, sdbind->influence);
  }
  
  /* Subtract the vertex coord to get the deformation offset. */
  sub_v3_v3(offset, vertexCos);

  /* Add the offset to start coord multiplied by the strength and weight values. */
  madd_v3_v3fl(vertexCos, offset, data->strength * weight);

      if (UNLIKELY(big_buffer)) {
    MEM_freeN(coords_buffer);
  }
}

static void end_stroke_evaluation(SurDeformGpencilModifierData *smd, bGPDframe *gpf)
{
  if (smd->layers->frames == NULL) return;
  if (smd->layers->frames->strokes == NULL) return;

  if (smd->layers->frames->strokes->stroke_idx == smd->layers->frames->strokes_num-1)/*If we are on the last stroke...*/
  {
    /* ...Go back to the start of the stroke array, 
    increase the layer array  or rol it back*/
    rollback_strokes(smd, smd->layers->frames);
   // if (smd->layers->layer_idx == smd->num_of_layers-1)
   // {smd->layers++;}
   // else
   // {rollback_layers;}
    /*Make it point to the right frame
    
    int i=0;
    while (smd->layers->frames->frame_number != gpf->framenum &&
          i < smd->layers->num_of_frames)
    { 
      smd->layers->frames++;
      i++;  
    }*/
  }
  else { /*Else increase the pointer */
    (smd->layers->frames->strokes)++;
  }
}

static void surfacedeformModifier_do(GpencilModifierData *md,
                                     SurDeformGpencilModifierData *smd,
                                     Depsgraph *depsgraph,
                                     bGPDstroke *gps,
                                     bGPDframe *gpf,
                                     bGPDlayer *gpl,
                                     Object *ob,
                                     Mesh *mesh)
{
  //SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;
  
  Mesh *target;
  uint target_verts_num, target_polys_num;
  uint verts_num = gps->totpoints;
  uint strokes_num = BLI_listbase_count(&gpf->strokes);
  SurDeformGpencilModifierData *smd_orig = get_original_modifier(ob, smd, md);
  Object *ob_orig = (Object *)DEG_get_original_id(&ob->id);
  bGPdata *gpd_orig = ob_orig->data;
  if (smd->layers == NULL)
  {
    smd_orig->flags = 0;
    return;
  }

  if (smd->layers->frames == NULL)
  {
    return;
  }

  
  
  
  /*Check only one time. Once the evaluation is over, the flag data will be copied again from 
  the original modifier, so the flag will hopefully became 0 again on its own.
  smd->bound_flags = smd_orig->bound_flags;
  if (!(smd->flags & GP_MOD_SDEF_CHECKED))
  {
    check_bind_situation(smd, depsgraph, DEG_get_evaluated_scene(depsgraph), ob);
    smd->flags |= GP_MOD_SDEF_CHECKED;
  }*/
 


  Object *ob_target = DEG_get_evaluated_object(depsgraph, smd->target);
  target = BKE_modifier_get_evaluated_mesh_from_evaluated_object(ob_target);

  /*So there might be many reason why the order of the layers (and frames?) 
  stored in the surdef structure may differ from the order of the bGPDlayer list.
  The best solution for now is to look for the right data at the start of every new layer
  evaluation. Could be made quicker with a check to see if it corresponds to the saved order,
  and even to swap it if it doesnt.*/
  rollback_layers(smd);
  while (strcmp(smd->layers->layer_info, gpl->info) != 0 &&
  smd->layers->layer_idx < smd->num_of_layers)
   {smd->layers++;}
  /*Make it point to the right frame*/
  rollback_frames(smd, smd->layers);
  if (smd->layers->frames != NULL)
  {
    int i = 0;
    while (smd->layers->frames->frame_number != gpf->framenum &&
           smd->layers->frames->frame_idx < smd->layers->num_of_frames) {
      i++;
      if (i == smd->layers->num_of_frames) {
        /*We are on a frame thats not bound*/
        /*No end of stroke evaluation. All the strokes won't be evaluated till a new, bound frame
        is accessed.*/

        /*Though we need to make sure this is not an actual bound frame that was moved.*/
        if (gpf->key_type == BEZT_KEYTYPE_SURDEFBOUND) {
          /*Dang, this was a bound frame! Someone moved it!*/
          Scene *scene = DEG_get_evaluated_scene(depsgraph);
          bGPDlayer *gpl_orig = BKE_gpencil_layer_active_get(gpd_orig);
          rollback_frames(smd, smd->layers);
          /*Let's try to find which SDefGPFrame doesnt have an associated bGPDframe, and remove
           * it.*/
          for (int f = 0; f < smd->layers->num_of_frames; f++) {
            bool found = false;
            LISTBASE_FOREACH (bGPDframe *, curr_gpf, &gpl_orig->frames) {
              if (curr_gpf->framenum == smd->layers->frames[f].frame_number) {
                found = true;
                break;
              }
            }
            if (!found) {
              /*Free this frame*/
              for (int l = 0; strcmp(smd->layers->layer_info, smd_orig->layers->layer_info); l++) {
                if (l > smd->num_of_layers) {
                  BKE_gpencil_modifier_set_error(md,
                                                 "Layers mismatch between original and evaluated "
                                                 "modifier. Deleting all data");
                  freeData(md);
                  return;
                }
                smd_orig->layers++;
              }
              free_frame_b(smd_orig, smd, smd_orig->layers, smd->layers->frames->frame_number);
              /*Clean the bound color*/
              bGPDframe *gpf_orig = BKE_gpencil_frame_retime_get(depsgraph, scene, ob, gpl_orig);
              gpf_orig->key_type = BEZT_KEYTYPE_KEYFRAME;
              return;
            }
          }
        }
        return;
      }
      else {
        smd->layers->frames++;
      }
    }
  }
  else
  {
    end_stroke_evaluation(smd, gpf);
    return;
  }
  
  
  

  if (!target) {
    BKE_gpencil_modifier_set_error(md,  "No valid target mesh");
    return;
  }
  target_verts_num = BKE_mesh_wrapper_vert_len(target);
  target_polys_num = BKE_mesh_wrapper_poly_len(target);

  if (!smd->layers)
  {return;}
    
  

  /*Exit evaluation if we are on a frame that is not bound.
  (? USELESS OR DANGEROUS)
  
  if (smd->layers->frames->frame_number != gpf->framenum) 
  {
    end_stroke_evaluation(smd, gpf);
    return;
  }*/

  

  /* Strokes count on the deforming Frame. */
  if (!(BLI_listbase_is_empty(&gpf->strokes)))
  {
    uint tot_strokes_num = BLI_listbase_count(&gpf->strokes);
    if (smd->layers->frames->strokes_num != tot_strokes_num) {
      BKE_gpencil_modifier_set_error(
          md, "Strokes changed from %u to %u", smd->layers->frames->strokes_num, tot_strokes_num);
      //TODO: free_frame
      return;
    } 
  }

  /* Points count on the deforming Stroke. */
  if (smd->layers->frames->strokes->stroke_verts_num != gps->totpoints) {
    BKE_gpencil_modifier_set_error(
        md, "Stroke %u: Points changed from %i to %i", smd->layers->frames->strokes->stroke_idx, smd->layers->frames->strokes->stroke_verts_num, gps->totpoints);
    return;
  } 

  /* Geometry count on the target mesh. */
  if (smd->target_polys_num != target_polys_num && smd->target_verts_num == 0) {
    /* Change in the number of polygons does not really imply change in the vertex count, but
     * this is how the modifier worked before the vertex count was known. Follow the legacy
     * logic without requirement to re-bind the mesh. */
    BKE_gpencil_modifier_set_error(
        md, "Target polygons changed from %u to %u", smd->target_polys_num, target_polys_num);
    return;
  }
  if (smd->target_verts_num != 0 && smd->target_verts_num != target_verts_num) {
    if (smd->target_verts_num > target_verts_num) {
      /* Number of vertices on the target did reduce. There is no usable recovery from this. */
      BKE_gpencil_modifier_set_error(
                             md,
                             "Target vertices changed from %u to %u",
                             smd->target_verts_num,
                             target_verts_num);
      return;
    }

    /* Assume the increase in the vertex count means that the "new" vertices in the target mesh are
     * added after the original ones. This covers typical case when target was at the subdivision
     * level 0 and then subdivision was increased (i.e. for the render purposes). 

    BKE_modifier_set_warning(ob,
                             md,
                             "Target vertices changed from %u to %u, continuing anyway",
                             smd->target_verts_num,
                             target_verts_num);*/

    /* In theory we only need the `smd->verts_num` vertices in the `targetCos` for evaluation, but
     * it is not currently possible to request a subset of coordinates: the API expects that the
     * caller needs coordinates of all vertices and asserts for it. */
  }

  /* Early out if modifier would not affect input at all - still *after* the sanity checks
   * (and potential binding) above. */
  if (smd->strength == 0.0f) {
    return;
  }

  int defgrp_index = BKE_object_defgroup_name_index(ob, smd->defgrp_name);
  MDeformVert *dvert = gps->dvert;

  //MOD_get_vgroup(ob, mesh, smd->defgrp_name, &dvert, &defgrp_index);
  const bool invert_vgroup = (smd->flags & GP_MOD_SDEF_INVERT_VGROUP) != 0;

  /* Actual vertex location update starts here */
  SDefGPLayer *curr_layer = NULL;
  SDefGPFrame *curr_frame = NULL;
  

  /*If we are on stroke 0, check all the current frames of all the layers in memory with their pointer
  to bGPDframe to find the right one and point to that. */
  if (gps->prev == NULL)
  {
    rollback_layers(smd);
    for (int l= 0; l < smd->num_of_layers; l++)
    {
      bool resutl = strcmp(smd->layers[l].layer_info, gpl->info);
      if (!resutl) //strcmp returns non zero in case of a difference
      {
        curr_layer = &(smd->layers[l]);
        
      }
    }

    if (!curr_layer) /* If layer or frame not found, retun and pass onto the next evaluation */
    {
      end_stroke_evaluation(smd, gpf);
      return;
    }

    
    smd->layers = curr_layer;
    rollback_frames(smd, smd->layers);
    for (int f= 0; f < smd->layers->num_of_frames; f++)
    {
      if (smd->layers->frames[f].frame_number == gpf->framenum)
      {
        curr_frame = &(smd->layers->frames[f]);
      }
    }
    if (!curr_frame) /* If layer or frame not found, retun and pass onto the next evaluation */
    {
      end_stroke_evaluation(smd, gpf);
      return;
    }
    smd->layers->frames = curr_frame;
  }

  
  SDefGPStroke *current_sdef_stroke = smd->layers->frames->strokes;
  SDefDeformData data = {
      .bind_verts = current_sdef_stroke->verts,
      .targetCos = MEM_malloc_arrayN(target_verts_num, sizeof(float[3]), "SDefTargetVertArray"),
      .gps = gps,
      .dvert = dvert,
      .defgrp_index = defgrp_index,
      .invert_vgroup = invert_vgroup,
      .strength = smd->strength,
  };
  
  if (data.targetCos != NULL) {
    float tmp_mat[4][4] = {
        {1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 1.0, -1.0}, {0.0, -1.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
    BKE_mesh_wrapper_vert_coords_copy_with_mat4(
        target, data.targetCos, target_verts_num, smd->mat);

    TaskParallelSettings settings;
    BLI_parallel_range_settings_defaults(&settings);
    settings.use_threading = (current_sdef_stroke->stroke_verts_num > 10000);
    BLI_task_parallel_range(0, current_sdef_stroke->stroke_verts_num, &data, deformVert, &settings);

    MEM_freeN(data.targetCos);
  }

  BKE_gpencil_stroke_geometry_update(ob->data, gps);
  
  end_stroke_evaluation(smd, gpf);
  
  
}


    /* END MOD_surfacedeform.c FUNCTIONS */

/* uses evaluated modifer */
static void deformStroke(GpencilModifierData *md, 
                         Depsgraph *depsgraph,
                         Object *ob,
                         bGPDlayer *gpl,
                         bGPDframe *gpf, // bGPDframe *gpf = BKE_gpencil_frame_retime_get(depsgraph, scene, ob, gpl)
                         bGPDstroke *gps)
{
  SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;
  if (smd->flags & GP_MOD_SDEF_WITHHOLD_EVALUATION) return;

  surfacedeformModifier_do(md, smd, depsgraph, gps, gpf, gpl, ob, NULL /*, mesh_src*/);

  /*if (!ELEM(mesh_src, NULL, mesh)) {
    BKE_id_free(NULL, mesh_src);
  }*/
  

}

static void bakeModifier(struct Main *UNUSED(bmain),
                         Depsgraph *depsgraph,
                         GpencilModifierData *md,
                         Object *ob)
{
  generic_bake_deform_stroke(depsgraph, md, ob, false, deformStroke);
}

/* uses original modifer */
static bool isDisabled(GpencilModifierData *md, bool UNUSED(useRenderParams))
{
  SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;

  /* The object type check is only needed here in case we have a placeholder
   * object assigned (because the library containing the mesh is missing).
   *
   * In other cases it should be impossible to have a type mismatch.
   */
  return (smd->target == NULL || smd->target->type != OB_MESH) &&
         (smd->layers == NULL || smd->bound_flags ==0);
}

static void frame_list_item(struct uiList *UNUSED(ui_list),
                              const struct bContext *UNUSED(C),
                              struct uiLayout *layout,
                              struct PointerRNA *UNUSED(idataptr),
                              struct PointerRNA *itemptr,
                              int UNUSED(icon),
                              struct PointerRNA *UNUSED(active_dataptr),
                              const char *UNUSED(active_propname),
                              int UNUSED(index),
                              int UNUSED(flt_flag))
{
  uiLayout *row = uiLayoutRow(layout, true);
  uiItemR(row, itemptr, "name", UI_ITEM_R_NO_BG, "", ICON_OUTLINER_DATA_GP_LAYER);
}

static void panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *sub, *row, *col;
  uiLayout *layout = panel->layout;
  char label[128];

  PointerRNA op_ptr_all;
  PointerRNA op_ptr_curr;
  PointerRNA ob_ptr;
  PointerRNA *ptr = gpencil_modifier_panel_get_property_pointers(panel, &ob_ptr);

  PointerRNA target_ptr = RNA_pointer_get(ptr, "target");
  GpencilModifierData *md = (GpencilModifierData *)ptr->data;
  SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;
  

 // bool unbind_mode = RNA_boolean_get(ptr, "unbind_mode");
  bool bind_all_frames = (RNA_enum_get(ptr, "curr_frame_or_all_frames") == GP_MOD_SDEF_BIND_ALL_FRAMES);
 
  bool all_layers_and_frames_bound = RNA_boolean_get(ptr, "all_layers_and_frames_bound");
  bool all_layers_current_frames_bound = RNA_boolean_get(ptr, "all_layers_current_frames_bound");
  bool current_layer_all_frames_bound = RNA_boolean_get(ptr, "current_layer_all_frames_bound");
  bool current_layer_current_frame_bound = RNA_boolean_get(ptr, "current_layer_current_frame_bound");
 // bool something_bound = RNA_boolean_get(ptr, "current_layer_current_frame_bound");
  uiLayoutSetPropSep(layout, true);

  col = uiLayoutColumn(layout, false);
  uiLayoutSetActive(col, !smd->layers);
  uiItemR(col, ptr, "target", 0, NULL, ICON_NONE); // TODO: disable layout if bound
  col = uiLayoutColumn(layout, false);
  uiItemR(col, ptr, "falloff", 0, NULL, ICON_NONE);

  bool display_unbind = false;

  uiItemR(layout, ptr, "strength", 0, NULL, ICON_NONE);
  row = uiLayoutRow(layout, true);
  uiItemPointerR(row, ptr, "vertex_group", &ob_ptr, "vertex_groups", NULL, ICON_NONE);
  sub = uiLayoutRow(row, true);
  uiItemR(sub, ptr, "invert_vertex_group", 0, "", ICON_ARROW_LEFTRIGHT);
  //modifier_vgroup_ui(layout, ptr, &ob_ptr, "vertex_group", "invert_vertex_group", NULL);

  /*col = uiLayoutColumn(layout, false);
  uiLayoutSetEnabled(col, !is_bound);
  uiLayoutSetActive(col, !is_bound && RNA_string_length(ptr, "vertex_group") != 0);
  uiItemR(col, ptr, "use_sparse_bind", 0, NULL, ICON_NONE); */

  uiItemS(layout);

  col = uiLayoutColumn(layout, false);
  

  /* BIND */
  row = uiLayoutRow(col, true);
  uiLayoutSetActive(col, !RNA_pointer_is_null(&target_ptr));
  uiItemFullO(row,  "GPENCIL_OT_gpencilsurdeform_bind",
               IFACE_("Bind All"), ICON_NONE, NULL,
              WM_OP_INVOKE_DEFAULT, 0, &op_ptr_all);
  RNA_enum_set(&op_ptr_all, "curr_frame_or_all_frames", 0);
  uiItemFullO(row,  "GPENCIL_OT_gpencilsurdeform_bind",
              IFACE_("Bind Current Frame"), ICON_NONE, NULL,
              WM_OP_INVOKE_DEFAULT,  0, &op_ptr_curr);
  RNA_enum_set(&op_ptr_curr, "curr_frame_or_all_frames", 1);

  /*UNBIND*/
  row = uiLayoutRow(col, true);
  uiLayoutSetActive(col, !RNA_pointer_is_null(&target_ptr));
  uiItemFullO(row, "GPENCIL_OT_gpencilsurdeform_unbind", 
              IFACE_("Unbind All"), ICON_NONE, NULL,
              WM_OP_INVOKE_DEFAULT, 0, &op_ptr_all);
  RNA_enum_set(&op_ptr_all, "curr_frame_or_all_frames", 0);
  uiItemFullO(row,  "GPENCIL_OT_gpencilsurdeform_unbind",
              IFACE_("Unbind current Frame"), ICON_NONE, NULL,
              WM_OP_INVOKE_DEFAULT, 0,  &op_ptr_curr);
  RNA_enum_set(&op_ptr_curr, "curr_frame_or_all_frames", 1);

  row = uiLayoutRow(col, true);
  
  if (smd->layers) 
  {
    for (int l = 0; l < smd->num_of_layers; l++)
    {
      row = uiLayoutRow(col, true);
      BLI_sprintf(label,
                  "Layer %s, Number of bound frames: %u",
                  smd->layers[l].layer_info, smd->layers[l].num_of_frames);
      uiItemL(row, label, ICON_INFO);
    }
    
    
  }
  else uiItemL(row, "No bound frames", ICON_INFO);
  /*
  row = uiLayoutRow(col, true);
  uiTemplateList(row,
                 (bContext *)C,
                 "MOD_UL_gpsurdef",
                 "",
                 ptr,
                 "uilist_frames",
                 ptr,
                 "uilist_frame_active_index",
                 NULL,
                 3,
                 10,
                 0,
                 1,
                 UI_TEMPLATE_LIST_FLAG_NONE);
  if (smd->uilist_frame_active_index != NULL) {

    /*Show layers
    row = uiLayoutRow(col, true);
    BLI_sprintf(label,
                "Layers: %s,",
                smd->layers[l].layer_info,);
    uiItemL(row, label, ICON_NONE);

    // OPERATOR unbind frame
    row = uiLayoutRow(col, true);
    RNA_enum_set(&op_ptr_all, "curr_frame_or_all_frames", 0);
    uiItemFullO(row,
                "GPENCIL_OT_gpencilsurdeform_unbind",
                IFACE_("Unbind"),
                ICON_NONE,
                NULL,
                WM_OP_INVOKE_DEFAULT,
                0,
                &op_ptr_curr);
  }
    */

  gpencil_modifier_panel_end(layout, ptr);
}


static void bake_panel_draw(const bContext *(C), Panel *panel)
{
  uiLayout *row, *col, *sub;
  uiLayout *layout = panel->layout;

  PointerRNA bake_op;

  int toggles_flag = UI_ITEM_R_TOGGLE | UI_ITEM_R_FORCE_BLANK_DECORATE;
  PointerRNA *ptr = gpencil_modifier_panel_get_property_pointers(panel, NULL);
  Scene *scene = CTX_data_scene(C);

  col = uiLayoutColumn(layout, false);

  uiLayoutSetPropSep(layout, true);

  row = uiLayoutRow(col, true);
  uiItemFullO(row,
              "GPENCIL_OT_gpencilsurdeform_bake",
              IFACE_("Bake Current Frame"),
              ICON_NONE,
              NULL,
              WM_OP_INVOKE_DEFAULT,
              0,
              &bake_op);
  int current_frame_num = (int)BKE_scene_frame_get(scene);
  RNA_int_set(&bake_op, "frame_start", current_frame_num);
  RNA_int_set(&bake_op, "frame_end", current_frame_num);

  row = uiLayoutRow(col, true);
  uiItemFullO(row,
              "GPENCIL_OT_gpencilsurdeform_bake",
              IFACE_("Bake Range"),
              ICON_NONE,
              NULL,
              WM_OP_INVOKE_DEFAULT,
              0,
              &bake_op);
  RNA_int_set(&bake_op, "frame_start", RNA_int_get(ptr, "bake_range_start"));
  RNA_int_set(&bake_op, "frame_end", RNA_int_get(ptr, "bake_range_end"));

  row = uiLayoutRow(col, true);
  uiItemL(row, "Range:", ICON_NONE);
  row = uiLayoutRow(col, true);
  uiItemR(row, ptr, "bake_range_start", 0, "Start", ICON_NONE);
  sub = uiLayoutRow(row, true);
  uiItemR(sub, ptr, "bake_range_end", 0, "End", ICON_NONE);
  sub = uiLayoutRow(row, true);
  uiItemFullO(sub,
              "GPENCIL_OT_gpencilsurdeform_fillrange",
              IFACE_(""),
              ICON_PREVIEW_RANGE,
              NULL,
              WM_OP_INVOKE_DEFAULT,
              0,
              NULL);

  gpencil_modifier_panel_end(layout, ptr);

  /*gpencil_modifier_masking_panel_draw(panel, true, true);*/
}

static void panelRegister(ARegionType *region_type)
{
  PanelType *panel_type = gpencil_modifier_panel_register(
      region_type, eGpencilModifierType_SurDeform, panel_draw);
  gpencil_modifier_subpanel_register(
      region_type, "bake", "Bake", NULL, bake_panel_draw, panel_type);

  uiListType *list_type = MEM_callocN(sizeof(uiListType), "dash modifier segment uilist");
  strcpy(list_type->idname, "MOD_UL_gpsurdef");
  list_type->draw_item = frame_list_item;
  WM_uilisttype_add(list_type);

}

static void foreachIDLink(GpencilModifierData *md, Object *ob, IDWalkFunc walk, void *userData)
{
  SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;

  walk(userData, ob, (ID **)&smd->target, IDWALK_CB_NOP);
}

GpencilModifierTypeInfo modifierType_Gpencil_SurDeform = {
    /* name */ "Surface Deform",
    /* structName */ "SurDeformGpencilModifierData",
    /* structSize */ sizeof(SurDeformGpencilModifierData),
    /* type */ eGpencilModifierTypeType_Gpencil,
    /* flags */ 0,

    /* copyData */ copyData,

    /* deformStroke */ deformStroke,
    /* generateStrokes */ NULL,
    /* bakeModifier */ bakeModifier,
    /* remapTime */ NULL,

    /* initData */ initData,
    /* freeData */ freeData,
    /* isDisabled */ isDisabled,
    /* updateDepsgraph */ updateDepsgraph,
    /* dependsOnTime */ dependsOnTime,
    /* foreachIDLink */ foreachIDLink,
    /* foreachTexLink */ NULL,
    /* panelRegister */ panelRegister,
};







