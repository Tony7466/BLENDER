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

  /* For standard single-action testing. */
  Object *object;

  /* For NLA testing. */
  Object *object_with_nla;
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
 * Tests for `insert_keyframe()`.
 */

/* Keying a non-array property with no keying flags. */
TEST_F(KeyframingTest, insert_keyframe__non_array_property)
{
  AnimationEvalContext anim_eval_context = {nullptr, 1.0};

  const CombinedKeyingResult result = insert_keyframe(bmain,
                                                      object->id,
                                                      nullptr,
                                                      "rotation_mode",
                                                      -1,
                                                      &anim_eval_context,
                                                      BEZT_KEYTYPE_KEYFRAME,
                                                      INSERTKEY_NOFLAGS);

  EXPECT_EQ(1, result.get_count(SingleKeyingResult::SUCCESS));
  EXPECT_NE(nullptr, object->adt);
  EXPECT_NE(nullptr, object->adt->action);
  EXPECT_EQ(1, BLI_listbase_count(&object->adt->action->curves));
  EXPECT_NE(nullptr, BKE_fcurve_find(&object->adt->action->curves, "rotation_mode", 0));
}

/* Keying a single element of an array property with no keying flags. */
TEST_F(KeyframingTest, insert_keyframe__single_element)
{
  AnimationEvalContext anim_eval_context = {nullptr, 1.0};

  const CombinedKeyingResult result = insert_keyframe(bmain,
                                                      object->id,
                                                      nullptr,
                                                      "rotation_euler",
                                                      0,
                                                      &anim_eval_context,
                                                      BEZT_KEYTYPE_KEYFRAME,
                                                      INSERTKEY_NOFLAGS);

  EXPECT_EQ(1, result.get_count(SingleKeyingResult::SUCCESS));
  EXPECT_NE(nullptr, object->adt);
  EXPECT_NE(nullptr, object->adt->action);
  EXPECT_EQ(1, BLI_listbase_count(&object->adt->action->curves));
  EXPECT_NE(nullptr, BKE_fcurve_find(&object->adt->action->curves, "rotation_euler", 0));
}

/* Keying all elements of an array property with no keying flags. */
TEST_F(KeyframingTest, insert_keyframe__all_elements)
{
  AnimationEvalContext anim_eval_context = {nullptr, 1.0};

  const CombinedKeyingResult result = insert_keyframe(bmain,
                                                      object->id,
                                                      nullptr,
                                                      "rotation_euler",
                                                      -1,
                                                      &anim_eval_context,
                                                      BEZT_KEYTYPE_KEYFRAME,
                                                      INSERTKEY_NOFLAGS);

  EXPECT_EQ(3, result.get_count(SingleKeyingResult::SUCCESS));
  EXPECT_NE(nullptr, object->adt);
  EXPECT_NE(nullptr, object->adt->action);
  EXPECT_EQ(3, BLI_listbase_count(&object->adt->action->curves));
  EXPECT_NE(nullptr, BKE_fcurve_find(&object->adt->action->curves, "rotation_euler", 0));
  EXPECT_NE(nullptr, BKE_fcurve_find(&object->adt->action->curves, "rotation_euler", 1));
  EXPECT_NE(nullptr, BKE_fcurve_find(&object->adt->action->curves, "rotation_euler", 2));
}

/* Keying with the "Only Insert Available" flag. */
TEST_F(KeyframingTest, insert_keyframe__only_available)
{
  AnimationEvalContext anim_eval_context = {nullptr, 1.0};

  /* First attempt should fail, because there are no fcurves yet. */
  const CombinedKeyingResult result_1 = insert_keyframe(bmain,
                                                        object->id,
                                                        nullptr,
                                                        "rotation_euler",
                                                        -1,
                                                        &anim_eval_context,
                                                        BEZT_KEYTYPE_KEYFRAME,
                                                        INSERTKEY_AVAILABLE);

  EXPECT_EQ(0, result_1.get_count(SingleKeyingResult::SUCCESS));
  EXPECT_NE(nullptr, object->adt);
  EXPECT_NE(nullptr, object->adt->action);
  EXPECT_EQ(0, BLI_listbase_count(&object->adt->action->curves));
  EXPECT_EQ(nullptr, BKE_fcurve_find(&object->adt->action->curves, "rotation_euler", 0));

  /* Insert a key on two of the elements without using the flag so that there
   * will be two fcurves. */
  insert_keyframe(bmain,
                  object->id,
                  nullptr,
                  "rotation_euler",
                  0,
                  &anim_eval_context,
                  BEZT_KEYTYPE_KEYFRAME,
                  INSERTKEY_NOFLAGS);
  insert_keyframe(bmain,
                  object->id,
                  nullptr,
                  "rotation_euler",
                  2,
                  &anim_eval_context,
                  BEZT_KEYTYPE_KEYFRAME,
                  INSERTKEY_NOFLAGS);

  /* Second attempt should succeed with two keys, because two of the elements
   * now have fcurves. */
  const CombinedKeyingResult result_2 = insert_keyframe(bmain,
                                                        object->id,
                                                        nullptr,
                                                        "rotation_euler",
                                                        -1,
                                                        &anim_eval_context,
                                                        BEZT_KEYTYPE_KEYFRAME,
                                                        INSERTKEY_AVAILABLE);

  EXPECT_EQ(2, result_2.get_count(SingleKeyingResult::SUCCESS));
  EXPECT_EQ(2, BLI_listbase_count(&object->adt->action->curves));
  EXPECT_NE(nullptr, BKE_fcurve_find(&object->adt->action->curves, "rotation_euler", 0));
  EXPECT_NE(nullptr, BKE_fcurve_find(&object->adt->action->curves, "rotation_euler", 2));
}

/* Keying with the "Only Insert Needed" flag. */
TEST_F(KeyframingTest, insert_keyframe__only_needed)
{
  AnimationEvalContext anim_eval_context = {nullptr, 1.0};

  /* First attempt should succeed, because there are no fcurves yet. */
  const CombinedKeyingResult result_1 = insert_keyframe(bmain,
                                                        object->id,
                                                        nullptr,
                                                        "rotation_euler",
                                                        -1,
                                                        &anim_eval_context,
                                                        BEZT_KEYTYPE_KEYFRAME,
                                                        INSERTKEY_NEEDED);

  EXPECT_EQ(3, result_1.get_count(SingleKeyingResult::SUCCESS));
  EXPECT_NE(nullptr, object->adt);
  EXPECT_NE(nullptr, object->adt->action);
  EXPECT_EQ(3, BLI_listbase_count(&object->adt->action->curves));
  FCurve *fcurve_x = BKE_fcurve_find(&object->adt->action->curves, "rotation_euler", 0);
  FCurve *fcurve_y = BKE_fcurve_find(&object->adt->action->curves, "rotation_euler", 1);
  FCurve *fcurve_z = BKE_fcurve_find(&object->adt->action->curves, "rotation_euler", 2);
  EXPECT_NE(nullptr, fcurve_x);
  EXPECT_NE(nullptr, fcurve_y);
  EXPECT_NE(nullptr, fcurve_z);

  /* Second attempt should fail, because there is now an fcurve for the
   * property, but its value matches the current property value. */
  anim_eval_context.eval_time = 10.0;
  const CombinedKeyingResult result_2 = insert_keyframe(bmain,
                                                        object->id,
                                                        nullptr,
                                                        "rotation_euler",
                                                        -1,
                                                        &anim_eval_context,
                                                        BEZT_KEYTYPE_KEYFRAME,
                                                        INSERTKEY_NEEDED);

  EXPECT_EQ(0, result_2.get_count(SingleKeyingResult::SUCCESS));
  EXPECT_EQ(3, BLI_listbase_count(&object->adt->action->curves));
  EXPECT_EQ(1, fcurve_x->totvert);
  EXPECT_EQ(1, fcurve_y->totvert);
  EXPECT_EQ(1, fcurve_z->totvert);

  /* Third attempt should succeed on two elements, because we change the value
   * of those elements to differ from the existing fcurves. */
  object->rot[0] = 123.0;
  object->rot[2] = 123.0;
  const CombinedKeyingResult result_3 = insert_keyframe(bmain,
                                                        object->id,
                                                        nullptr,
                                                        "rotation_euler",
                                                        -1,
                                                        &anim_eval_context,
                                                        BEZT_KEYTYPE_KEYFRAME,
                                                        INSERTKEY_NEEDED);

  EXPECT_EQ(2, result_3.get_count(SingleKeyingResult::SUCCESS));
  EXPECT_EQ(3, BLI_listbase_count(&object->adt->action->curves));
  EXPECT_EQ(2, fcurve_x->totvert);
  EXPECT_EQ(1, fcurve_y->totvert);
  EXPECT_EQ(2, fcurve_z->totvert);
}

/* ------------------------------------------------------------
 * Testing a special case of the NLA system:
 * When keying a strip with the "replace" or "combine" mix mode, keying a single
 * quaternion element should be treated as keying all quaternion elements in an
 * all-or-nothing fashion.
 */

/* With no special keyframing flags *all* quaternion elements should get keyed
 * if any of them are. */
TEST_F(KeyframingTest, insert_keyframe__quaternion_on_nla)
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

/* With the "Only Insert Available" flag enabled, keys for all four elements
 * should be inserted if *any* of the elements have fcurves already, and
 * otherwise none of the elements should be keyed. */
TEST_F(KeyframingTest, insert_keyframe__quaternion_on_nla__only_available)
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

/* With the "Only Insert Needed" flag enabled, keys for all four elements should
 * be inserted if *any* of them don't yet have an fcurve or they differ from the
 * evaluation at their existing fcurve, and otherwise none of the elements
 * should be keyed. */
TEST_F(KeyframingTest, insert_keyframe__quaternion_on_nla__only_needed)
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

  /* If we modify one of the object's quaternion values, then all four should
   * insert keys again. */
  object_with_nla->quat[0] = 0.5;
  const CombinedKeyingResult result_3 = insert_keyframe(bmain,
                                                        object_with_nla->id,
                                                        nullptr,
                                                        "rotation_quaternion",
                                                        0,
                                                        &anim_eval_context,
                                                        BEZT_KEYTYPE_KEYFRAME,
                                                        INSERTKEY_NEEDED);
  EXPECT_EQ(0, result_3.get_count(SingleKeyingResult::SUCCESS));
}

}  // namespace blender::animrig::tests
