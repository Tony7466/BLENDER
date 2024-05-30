/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "ANIM_action.hh"
#include "ANIM_keyframing.hh"

#include "BKE_action.hh"
#include "BKE_anim_data.hh"
#include "BKE_animsys.h"
#include "BKE_fcurve.hh"
#include "BKE_idtype.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_nla.h"
#include "BKE_object.hh"

#include "DNA_anim_types.h"
#include "DNA_object_types.h"

#include "RNA_access.hh"

#include "BLI_listbase.h"
#include "BLI_string_utf8.h"

#include <limits>

#include "CLG_log.h"
#include "testing/testing.h"

namespace blender::animrig::tests {
class KeyframingTest : public testing::Test {
 public:
  Main *bmain;
  Object *object;
  Object *object_with_nla;
  bAction *action;
  bAction *nla_action;

  static void SetUpTestSuite()
  {
    /* BKE_id_free() hits a code path that uses CLOG, which crashes if not initialized properly. */
    CLG_init();

    /* To make id_can_have_animdata() and friends work, the `id_types` array needs to be set up. */
    BKE_idtype_init();
  }

  static void TearDownTestSuite()
  {
    CLG_exit();
  }

  void SetUp() override
  {
    bmain = BKE_main_new();

    object = BKE_object_add_only_object(bmain, OB_EMPTY, "OBEmpty");
    object_with_nla = BKE_object_add_only_object(bmain, OB_EMPTY, "OBEmptyWithNLA");
    action = static_cast<bAction *>(BKE_id_new(bmain, ID_AC, "ACAction"));
    nla_action = static_cast<bAction *>(BKE_id_new(bmain, ID_AC, "ACNLAAction"));

    /* Set up an NLA system with a single NLA track with a single NLA strip, and
     * make that strip active and in tweak mode. */
    AnimData *adt = BKE_animdata_ensure_id(&object_with_nla->id);
    NlaTrack *track = BKE_nlatrack_new_head(&adt->nla_tracks, false);
    NlaStrip *strip = BKE_nlastack_add_strip(adt, nla_action, false);
    track->flag |= NLATRACK_ACTIVE;
    strip->flag |= NLASTRIP_FLAG_ACTIVE;
    strip->start = 0.0;
    strip->end = 1000.0;
    strip->actstart = 0.0;
    strip->actend = 1000.0;
    strip->scale = 1.0;
    strip->blendmode = NLASTRIP_MODE_COMBINE;
    BKE_nla_tweakmode_enter(adt);
  }

  void TearDown() override
  {
    BKE_main_free(bmain);
  }
};

/* ------------------------------------------------------------
 * Testing a special case of the NLA system:
 * If you're keying on a track with a REPLACE or COMBINE mix mode, then
 * attempting to key just a single quaternion array element should force either
 * all or none of the quaternion elements to get keyed, depending on the
 * situation and keyframing settings.
 */

TEST_F(KeyframingTest, keying_quaternion_on_nla)
{
  AnimationEvalContext anim_eval_context = {nullptr, 1.0};
  object_with_nla->rotmode = ROT_MODE_QUAT;

  const CombinedKeyingResult result = insert_keyframe(bmain,
                                                      object_with_nla->id,
                                                      nullptr,
                                                      "rotation_quaternion",
                                                      0,
                                                      &anim_eval_context,
                                                      BEZT_KEYTYPE_KEYFRAME,
                                                      INSERTKEY_NOFLAGS);

  EXPECT_EQ(4, result.get_count(SingleKeyingResult::SUCCESS));
}

TEST_F(KeyframingTest, keying_quaternion_on_nla__only_available)
{
  AnimationEvalContext anim_eval_context = {nullptr, 1.0};
  object_with_nla->rotmode = ROT_MODE_QUAT;

  /* There are no fcurves at all yet, so all elements getting no keys is what
   * should happen. */
  const CombinedKeyingResult result_1 = insert_keyframe(bmain,
                                                        object_with_nla->id,
                                                        nullptr,
                                                        "rotation_quaternion",
                                                        0,
                                                        &anim_eval_context,
                                                        BEZT_KEYTYPE_KEYFRAME,
                                                        INSERTKEY_AVAILABLE);
  EXPECT_EQ(0, result_1.get_count(SingleKeyingResult::SUCCESS));

  /* Create an fcurve and key for a single quaternion channel. */
  PointerRNA id_rna_ptr = RNA_id_pointer_create(&object_with_nla->id);
  FCurve *fcu = action_fcurve_ensure(
      bmain, nla_action, nullptr, &id_rna_ptr, "rotation_quaternion", 0);
  const KeyframeSettings keyframe_settings = {BEZT_KEYTYPE_KEYFRAME, HD_AUTO_ANIM, BEZT_IPO_BEZ};
  insert_vert_fcurve(fcu, {1.0, 1.0}, keyframe_settings, INSERTKEY_NOFLAGS);

  /* Now that there is one fcurve, all elements should get keyed. */
  const CombinedKeyingResult result_2 = insert_keyframe(bmain,
                                                        object_with_nla->id,
                                                        nullptr,
                                                        "rotation_quaternion",
                                                        0,
                                                        &anim_eval_context,
                                                        BEZT_KEYTYPE_KEYFRAME,
                                                        INSERTKEY_AVAILABLE);
  EXPECT_EQ(4, result_2.get_count(SingleKeyingResult::SUCCESS));
}

TEST_F(KeyframingTest, keying_quaternion_on_nla__only_needed)
{
  AnimationEvalContext anim_eval_context = {nullptr, 1.0};
  object_with_nla->rotmode = ROT_MODE_QUAT;

  /* First time should insert all 4 components, since there are no fcurves yet. */
  const CombinedKeyingResult result_1 = insert_keyframe(bmain,
                                                        object_with_nla->id,
                                                        nullptr,
                                                        "rotation_quaternion",
                                                        0,
                                                        &anim_eval_context,
                                                        BEZT_KEYTYPE_KEYFRAME,
                                                        INSERTKEY_NEEDED);
  EXPECT_EQ(4, result_1.get_count(SingleKeyingResult::SUCCESS));

  /* Second time should insert none, since no keys are needed. */
  anim_eval_context.eval_time = 10.0;
  const CombinedKeyingResult result_2 = insert_keyframe(bmain,
                                                        object_with_nla->id,
                                                        nullptr,
                                                        "rotation_quaternion",
                                                        0,
                                                        &anim_eval_context,
                                                        BEZT_KEYTYPE_KEYFRAME,
                                                        INSERTKEY_NEEDED);
  EXPECT_EQ(0, result_2.get_count(SingleKeyingResult::SUCCESS));
}

}  // namespace blender::animrig::tests
