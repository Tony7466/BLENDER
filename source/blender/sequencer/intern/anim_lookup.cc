/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sequencer
 */

#include "BLI_blenlib.h"
#include "BLI_hash_mm3.hh"
#include "BLI_map.hh"

#include "BKE_image.h"
#include "BKE_main.hh"
#include "BKE_scene.hh"

#include "DNA_scene_types.h"
#include "DNA_sequence_types.h"

#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"

#include "SEQ_sequencer.hh"
#include "SEQ_utils.hh"

#include "multiview.hh"
#include "proxy.hh"

/* For now api funcs are declared here. */
#include "SEQ_relations.hh"
#include "utils.hh"

static void anim_filepath_get(const Scene *scene,
                              Sequence *seq,
                              size_t filepath_size,
                              char *r_filepath)
{
  BLI_path_join(r_filepath, filepath_size, seq->strip->dirpath, seq->strip->stripdata->filename);
  BLI_path_abs(r_filepath, ID_BLEND_PATH_FROM_GLOBAL(&scene->id));
}

static bool use_proxy(Editing *ed, Sequence *seq)
{
  StripProxy *proxy = seq->strip->proxy;
  return proxy && ((proxy->storage & SEQ_STORAGE_PROXY_CUSTOM_DIR) != 0 ||
                   (ed->proxy_storage == SEQ_EDIT_PROXY_DIR_STORAGE));
}

static void proxy_dir_get(Editing *ed, Sequence *seq, size_t str_len, char *r_proxy_dirpath)
{
  if (use_proxy(ed, seq)) {
    if (ed->proxy_storage == SEQ_EDIT_PROXY_DIR_STORAGE) {
      if (ed->proxy_dir[0] == 0) {
        BLI_strncpy(r_proxy_dirpath, "//BL_proxy", str_len);
      }
      else {
        BLI_strncpy(r_proxy_dirpath, ed->proxy_dir, str_len);
      }
    }
    else {
      BLI_strncpy(r_proxy_dirpath, seq->strip->proxy->dirpath, str_len);
    }
    BLI_path_abs(r_proxy_dirpath, BKE_main_blendfile_path_from_global());
  }
}

static void index_dir_set(Editing *ed, Sequence *seq, StripAnim *sanim)
{
  if (sanim->anim == nullptr || !use_proxy(ed, seq)) {
    return;
  }

  char proxy_dirpath[FILE_MAX];
  proxy_dir_get(ed, seq, sizeof(proxy_dirpath), proxy_dirpath);
  seq_proxy_index_dir_set(sanim->anim, proxy_dirpath);
}

static ImBufAnim *open_anim_filepath(Sequence *seq, const char *filepath, bool openfile)
{
  ImBufAnim *anim = nullptr;

  if (openfile) {
    anim = openanim(filepath,
                    IB_rect | ((seq->flag & SEQ_FILTERY) ? IB_animdeinterlace : 0),
                    seq->streamindex,
                    seq->strip->colorspace_settings.name);
  }
  else {
    anim = openanim_noload(filepath,
                           IB_rect | ((seq->flag & SEQ_FILTERY) ? IB_animdeinterlace : 0),
                           seq->streamindex,
                           seq->strip->colorspace_settings.name);
  }

  return anim;
}

static bool open_anim_file_multiview(Scene *scene, Sequence *seq, char *filepath)
{
  char prefix[FILE_MAX];
  const char *ext = nullptr;
  BKE_scene_multiview_view_prefix_get(scene, filepath, prefix, &ext);

  if (seq->views_format != R_IMF_VIEWS_INDIVIDUAL || prefix[0] == '\0') {
    return false;
  }

  Editing *ed = SEQ_editing_get(scene);
  bool is_multiview_loaded = false;
  int totfiles = seq_num_files(scene, seq->views_format, true);

  for (int i = 0; i < totfiles; i++) {
    const char *suffix = BKE_scene_multiview_view_id_suffix_get(&scene->r, i);
    char filepath_view[FILE_MAX];
    SNPRINTF(filepath_view, "%s%s%s", prefix, suffix, ext);

    /* Multiview files must be loaded, otherwise it is not possible to detect failure. */
    ImBufAnim *anim = open_anim_filepath(seq, filepath_view, true);

    if (anim == nullptr) {
      return false; /* Multiview render failed. */
    }

    StripAnim *sanim = static_cast<StripAnim *>(MEM_mallocN(sizeof(StripAnim), "Strip Anim"));
    sanim->anim = anim;
    index_dir_set(ed, seq, sanim);
    BLI_addtail(&seq->anims, sanim);
    IMB_suffix_anim(sanim->anim, suffix);
    is_multiview_loaded = true;
  }

  return is_multiview_loaded;
}

//////////////////////////////////////////////////////////////////////////////////////////

struct SharableAnim {
  ImBufAnim *anim;  // this can be vector of anims for multiview in order as they are placed in LB
  int user_count;

  void release_from_strip(Sequence *seq)
  {
    if (user_count > 1) {
      user_count--;
      return;
    }

    IMB_free_anim(anim);
    BLI_freelist(&seq->anims);  // XXX perhaps it's better to traverse anims and only free matching
  };

  void assign_to_strip(Editing *ed, Sequence *seq)
  {
    BLI_assert(anim != nullptr);
    StripAnim *sanim = static_cast<StripAnim *>(MEM_mallocN(sizeof(StripAnim), "Strip Anim"));
    sanim->anim = anim;
    BLI_addtail(&seq->anims, sanim);
    index_dir_set(ed, seq, sanim);
    user_count++;
  };

  void acquire_anim(Sequence *seq, const char *filepath, bool openfile)
  {
    anim = open_anim_filepath(seq, filepath, openfile);
  }

  bool has_anim()
  {
    return anim != nullptr;
  }
};

struct AnimLookup {
  blender::Map<std::string, SharableAnim> anims;
};

static void anim_lookup_ensure(Editing *ed)
{
  if (ed->runtime.anim_lookup == nullptr) {
    ed->runtime.anim_lookup = MEM_new<AnimLookup>(__func__);
  }
}

static SharableAnim &anim_lookup_by_filepath(Editing *ed, const char *filepath)
{
  blender::Map<std::string, SharableAnim> &anims = ed->runtime.anim_lookup->anims;
  return anims.lookup_or_add_default(std::string(filepath));
}

static SharableAnim &anim_lookup_by_seq(const Scene *scene, Sequence *seq)
{
  Editing *ed = SEQ_editing_get(scene);
  anim_lookup_ensure(ed);
  char filepath[FILE_MAX];
  anim_filepath_get(scene, seq, sizeof(filepath), filepath);
  return anim_lookup_by_filepath(ed, filepath);
}
//////////////////////////////////////////////////////////////////////////////////////////

void seq_open_anim_file(Scene *scene, Sequence *seq, bool openfile)
{
  if ((seq->anims.first != nullptr) && (((StripAnim *)seq->anims.first)->anim != nullptr) &&
      !openfile)
  {
    return;
  }

  Editing *ed = SEQ_editing_get(scene);
  SEQ_relations_sequence_free_anim(scene, seq); /* Reset all the previously created anims. */
  anim_lookup_ensure(ed);

  char filepath[FILE_MAX];
  anim_filepath_get(scene, seq, sizeof(filepath), filepath);
  bool is_multiview = (seq->flag & SEQ_USE_VIEWS) != 0 && (scene->r.scemode & R_MULTIVIEW) != 0;
  bool multiview_is_loaded = false;

  if (is_multiview) {
    multiview_is_loaded = open_anim_file_multiview(scene, seq, filepath);
  }

  SharableAnim &sh_anim = anim_lookup_by_filepath(ed, filepath);

  if (!is_multiview || !multiview_is_loaded) {
    if (!sh_anim.has_anim()) {
      sh_anim.acquire_anim(seq, filepath, openfile);
    }
    sh_anim.assign_to_strip(ed, seq);
  }
}

/* Eeeh I will need to think about how to make this function  */
void SEQ_relations_sequence_free_anim(const Scene *scene, Sequence *seq)
{
  if (BLI_listbase_count(&seq->anims) == 1) {
    SharableAnim &sh_anim = anim_lookup_by_seq(scene, seq);
    sh_anim.release_from_strip(seq);
  }
  else {
    /* Case for multiview for now. */
    LISTBASE_FOREACH (StripAnim *, sanim, &seq->anims) {
      if (sanim->anim) {
        IMB_free_anim(sanim->anim);
        sanim->anim = nullptr;
      }

      BLI_freelinkN(&seq->anims, sanim);
    }
    BLI_listbase_clear(&seq->anims);
  }
}
