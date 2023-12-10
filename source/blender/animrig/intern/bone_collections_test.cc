/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_string.h"

#include "BLT_translation.h"

#include "BKE_idtype.h"
#include "BKE_lib_id.h"

#include "ANIM_bone_collections.hh"
#include "intern/bone_collections_internal.hh"

#include "intern/bone_collections_internal.hh"

#include "testing/testing.h"

namespace blender::animrig::tests {

TEST(ANIM_bone_collections, bonecoll_new_free)
{
  BoneCollection *bcoll = ANIM_bonecoll_new("some name");
  EXPECT_NE(nullptr, bcoll);
  EXPECT_EQ("some name", std::string(bcoll->name));
  EXPECT_TRUE(BLI_listbase_is_empty(&bcoll->bones));
  EXPECT_EQ(BONE_COLLECTION_VISIBLE | BONE_COLLECTION_SELECTABLE, bcoll->flags);
  ANIM_bonecoll_free(bcoll);
}

TEST(ANIM_bone_collections, bonecoll_default_name)
{
  {
    BoneCollection *bcoll = ANIM_bonecoll_new("");
    EXPECT_EQ(DATA_("Bones"), std::string(bcoll->name));
    ANIM_bonecoll_free(bcoll);
  }

  {
    BoneCollection *bcoll = ANIM_bonecoll_new(nullptr);
    EXPECT_EQ(DATA_("Bones"), std::string(bcoll->name));
    ANIM_bonecoll_free(bcoll);
  }
}

class ANIM_armature_bone_collections : public testing::Test {
 protected:
  bArmature arm;
  Bone bone1, bone2, bone3;

  void SetUp() override
  {
    memset(&arm, 0, sizeof(arm));
    memset(&bone1, 0, sizeof(Bone));
    memset(&bone2, 0, sizeof(Bone));
    memset(&bone3, 0, sizeof(Bone));

    STRNCPY(arm.id.name, "ARArmature");
    STRNCPY(bone1.name, "bone1");
    STRNCPY(bone2.name, "bone2");
    STRNCPY(bone3.name, "bone3");

    BLI_addtail(&arm.bonebase, &bone1);    /* bone1 is root bone. */
    BLI_addtail(&arm.bonebase, &bone2);    /* bone2 is root bone. */
    BLI_addtail(&bone2.childbase, &bone3); /* bone3 has bone2 as parent. */

    BKE_armature_bone_hash_make(&arm);
  }

  void TearDown() override
  {
    /* Avoid freeing the bones, as they are part of this struct and not owned by
     * the armature. */
    BLI_listbase_clear(&arm.bonebase);

    BKE_idtype_init();
    BKE_libblock_free_datablock(&arm.id, 0);
  }
};

TEST_F(ANIM_armature_bone_collections, armature_owned_collections)
{
  BoneCollection *bcoll1 = ANIM_armature_bonecoll_new(&arm, "collection");
  BoneCollection *bcoll2 = ANIM_armature_bonecoll_new(&arm, "collection");

  EXPECT_EQ(std::string("collection"), std::string(bcoll1->name));
  EXPECT_EQ(std::string("collection.001"), std::string(bcoll2->name));

  ANIM_armature_bonecoll_remove(&arm, bcoll1);
  ANIM_armature_bonecoll_remove(&arm, bcoll2);
}

TEST_F(ANIM_armature_bone_collections, collection_hierarchy_creation)
{
  /* Implicit root: */
  BoneCollection *bcoll_root_0 = ANIM_armature_bonecoll_new(&arm, "wortel");
  /* Explicit root: */
  BoneCollection *bcoll_root_1 = ANIM_armature_bonecoll_new(&arm, "wortel", -1);

  ASSERT_EQ(0, bonecoll_find_index(&arm, bcoll_root_0));
  ASSERT_EQ(1, bonecoll_find_index(&arm, bcoll_root_1));

  /* Child of bcoll at index 0: */
  BoneCollection *bcoll_child_of_0 = ANIM_armature_bonecoll_new(&arm, "koter", 0);
  /* Child of bcoll at index 1: */
  BoneCollection *bcoll_child_of_1 = ANIM_armature_bonecoll_new(&arm, "koter", 1);

  ASSERT_EQ(4, arm.collection_array_num);
  EXPECT_EQ(0, bonecoll_find_index(&arm, bcoll_root_0));
  EXPECT_EQ(1, bonecoll_find_index(&arm, bcoll_root_1));
  EXPECT_EQ(2, bonecoll_find_index(&arm, bcoll_child_of_0));
  EXPECT_EQ(3, bonecoll_find_index(&arm, bcoll_child_of_1));

  /* Add another child of bcoll_root_0, which should push bcoll_child_of_1 further down the array.
   */
  BoneCollection *bcoll_another_child_of_0 = ANIM_armature_bonecoll_new(&arm, "koter", 0);
  ASSERT_EQ(5, arm.collection_array_num);
  EXPECT_EQ(0, bonecoll_find_index(&arm, bcoll_root_0));
  EXPECT_EQ(1, bonecoll_find_index(&arm, bcoll_root_1));
  EXPECT_EQ(2, bonecoll_find_index(&arm, bcoll_child_of_0));
  EXPECT_EQ(3, bonecoll_find_index(&arm, bcoll_another_child_of_0));
  EXPECT_EQ(4, bonecoll_find_index(&arm, bcoll_child_of_1));

  /* Make sure the names remain unique within the entire Armature, and not just between siblings
   * (i.e. a unique 'path' is not strong enough). */
  EXPECT_EQ(std::string("wortel"), std::string(bcoll_root_0->name));
  EXPECT_EQ(std::string("wortel.001"), std::string(bcoll_root_1->name));
  EXPECT_EQ(std::string("koter"), std::string(bcoll_child_of_0->name));
  EXPECT_EQ(std::string("koter.001"), std::string(bcoll_child_of_1->name));
  EXPECT_EQ(std::string("koter.002"), std::string(bcoll_another_child_of_0->name));

  /* Test the internal hierarchy bookkeeping. */
  EXPECT_EQ(2, arm.collection_root_count);
  EXPECT_EQ(2, bcoll_root_0->child_count);
  EXPECT_EQ(1, bcoll_root_1->child_count);
  EXPECT_EQ(0, bcoll_child_of_0->child_count);
  EXPECT_EQ(0, bcoll_another_child_of_0->child_count);
  EXPECT_EQ(0, bcoll_child_of_1->child_count);

  EXPECT_EQ(2, bcoll_root_0->child_index);
  EXPECT_EQ(4, bcoll_root_1->child_index);
  EXPECT_EQ(0, bcoll_child_of_0->child_index);
  EXPECT_EQ(0, bcoll_another_child_of_0->child_index);
  EXPECT_EQ(0, bcoll_child_of_1->child_index);

  /* TODO: test with deeper hierarchy. */
}

TEST_F(ANIM_armature_bone_collections, collection_hierarchy_removal)
{
  /* Set up a small hierarchy. */
  BoneCollection *bcoll_root_0 = ANIM_armature_bonecoll_new(&arm, "root_0");
  BoneCollection *bcoll_root_1 = ANIM_armature_bonecoll_new(&arm, "root_1");
  BoneCollection *bcoll_r0_child0 = ANIM_armature_bonecoll_new(&arm, "r0_child0", 0);
  BoneCollection *bcoll_r1_child0 = ANIM_armature_bonecoll_new(&arm, "r1_child0", 1);
  BoneCollection *bcoll_r0_child1 = ANIM_armature_bonecoll_new(&arm, "r0_child1", 0);
  BoneCollection *bcoll_r0_child2 = ANIM_armature_bonecoll_new(&arm, "r0_child2", 0);

  ASSERT_EQ(2, arm.collection_root_count);
  ASSERT_EQ(6, arm.collection_array_num);
  ASSERT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  ASSERT_STREQ(bcoll_root_1->name, arm.collection_array[1]->name);
  ASSERT_STREQ(bcoll_r0_child0->name, arm.collection_array[2]->name);
  ASSERT_STREQ(bcoll_r0_child1->name, arm.collection_array[3]->name);
  ASSERT_STREQ(bcoll_r0_child2->name, arm.collection_array[4]->name);
  ASSERT_STREQ(bcoll_r1_child0->name, arm.collection_array[5]->name);

  ASSERT_EQ(2, arm.collection_array[0]->child_index);
  ASSERT_EQ(5, arm.collection_array[1]->child_index);
  ASSERT_EQ(0, arm.collection_array[2]->child_index);
  ASSERT_EQ(0, arm.collection_array[3]->child_index);
  ASSERT_EQ(0, arm.collection_array[4]->child_index);
  ASSERT_EQ(0, arm.collection_array[5]->child_index);

  ASSERT_EQ(3, arm.collection_array[0]->child_count);
  ASSERT_EQ(1, arm.collection_array[1]->child_count);
  ASSERT_EQ(0, arm.collection_array[2]->child_count);
  ASSERT_EQ(0, arm.collection_array[3]->child_count);
  ASSERT_EQ(0, arm.collection_array[4]->child_count);
  ASSERT_EQ(0, arm.collection_array[5]->child_count);

  /* Remove the middle child of root_0. */
  ANIM_armature_bonecoll_remove(&arm, bcoll_r0_child1);

  ASSERT_EQ(2, arm.collection_root_count);
  ASSERT_EQ(5, arm.collection_array_num);
  EXPECT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  EXPECT_STREQ(bcoll_root_1->name, arm.collection_array[1]->name);
  EXPECT_STREQ(bcoll_r0_child0->name, arm.collection_array[2]->name);
  EXPECT_STREQ(bcoll_r0_child2->name, arm.collection_array[3]->name);
  EXPECT_STREQ(bcoll_r1_child0->name, arm.collection_array[4]->name);

  EXPECT_EQ(2, arm.collection_array[0]->child_index);
  EXPECT_EQ(4, arm.collection_array[1]->child_index);
  EXPECT_EQ(0, arm.collection_array[2]->child_index);
  EXPECT_EQ(0, arm.collection_array[3]->child_index);
  EXPECT_EQ(0, arm.collection_array[4]->child_index);

  EXPECT_EQ(2, arm.collection_array[0]->child_count);
  EXPECT_EQ(1, arm.collection_array[1]->child_count);
  EXPECT_EQ(0, arm.collection_array[2]->child_count);
  EXPECT_EQ(0, arm.collection_array[3]->child_count);
  EXPECT_EQ(0, arm.collection_array[4]->child_count);

  /* Remove the first child of root_0. */
  ANIM_armature_bonecoll_remove(&arm, bcoll_r0_child0);

  ASSERT_EQ(2, arm.collection_root_count);
  ASSERT_EQ(4, arm.collection_array_num);
  EXPECT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  EXPECT_STREQ(bcoll_root_1->name, arm.collection_array[1]->name);
  EXPECT_STREQ(bcoll_r0_child2->name, arm.collection_array[2]->name);
  EXPECT_STREQ(bcoll_r1_child0->name, arm.collection_array[3]->name);

  EXPECT_EQ(2, arm.collection_array[0]->child_index);
  EXPECT_EQ(3, arm.collection_array[1]->child_index);
  EXPECT_EQ(0, arm.collection_array[2]->child_index);
  EXPECT_EQ(0, arm.collection_array[3]->child_index);

  EXPECT_EQ(1, arm.collection_array[0]->child_count);
  EXPECT_EQ(1, arm.collection_array[1]->child_count);
  EXPECT_EQ(0, arm.collection_array[2]->child_count);
  EXPECT_EQ(0, arm.collection_array[3]->child_count);

  /* Remove root_1 itself, which should make its only child a new root. */
  ANIM_armature_bonecoll_remove(&arm, bcoll_root_1);

  ASSERT_EQ(2, arm.collection_root_count);
  ASSERT_EQ(3, arm.collection_array_num);
  EXPECT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  EXPECT_STREQ(bcoll_r1_child0->name, arm.collection_array[1]->name);
  EXPECT_STREQ(bcoll_r0_child2->name, arm.collection_array[2]->name);

  EXPECT_EQ(2, arm.collection_array[0]->child_index);
  EXPECT_EQ(0, arm.collection_array[1]->child_index);
  EXPECT_EQ(0, arm.collection_array[2]->child_index);

  EXPECT_EQ(1, arm.collection_array[0]->child_count);
  EXPECT_EQ(0, arm.collection_array[1]->child_count);
  EXPECT_EQ(0, arm.collection_array[2]->child_count);
}

TEST_F(ANIM_armature_bone_collections,
       collection_hierarchy_removal__more_complex_remove_inner_child)
{
  /* Set up a slightly bigger hierarchy. Contrary to the other tests these are
   * actually declared in array order. */
  BoneCollection *bcoll_root_0 = ANIM_armature_bonecoll_new(&arm, "root_0");
  BoneCollection *bcoll_root_1 = ANIM_armature_bonecoll_new(&arm, "root_1");
  BoneCollection *bcoll_r0_child0 = ANIM_armature_bonecoll_new(&arm, "r0_child0", 0);
  BoneCollection *bcoll_r0_child1 = ANIM_armature_bonecoll_new(&arm, "r0_child1", 0);
  BoneCollection *bcoll_r0_child2 = ANIM_armature_bonecoll_new(&arm, "r0_child2", 0);
  BoneCollection *bcoll_r0c0_child0 = ANIM_armature_bonecoll_new(&arm, "r0c0_child0", 2);
  BoneCollection *bcoll_r0c0_child1 = ANIM_armature_bonecoll_new(&arm, "r0c0_child1", 2);
  BoneCollection *bcoll_r0c0_child2 = ANIM_armature_bonecoll_new(&arm, "r0c0_child2", 2);

  ASSERT_EQ(2, arm.collection_root_count);
  ASSERT_EQ(8, arm.collection_array_num);
  ASSERT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  ASSERT_STREQ(bcoll_root_1->name, arm.collection_array[1]->name);
  ASSERT_STREQ(bcoll_r0_child0->name, arm.collection_array[2]->name);
  ASSERT_STREQ(bcoll_r0_child1->name, arm.collection_array[3]->name);
  ASSERT_STREQ(bcoll_r0_child2->name, arm.collection_array[4]->name);
  ASSERT_STREQ(bcoll_r0c0_child0->name, arm.collection_array[5]->name);
  ASSERT_STREQ(bcoll_r0c0_child1->name, arm.collection_array[6]->name);
  ASSERT_STREQ(bcoll_r0c0_child2->name, arm.collection_array[7]->name);

  ASSERT_EQ(2, arm.collection_array[0]->child_index);
  ASSERT_EQ(0, arm.collection_array[1]->child_index);
  ASSERT_EQ(5, arm.collection_array[2]->child_index);
  ASSERT_EQ(0, arm.collection_array[3]->child_index);
  ASSERT_EQ(0, arm.collection_array[4]->child_index);
  ASSERT_EQ(0, arm.collection_array[5]->child_index);
  ASSERT_EQ(0, arm.collection_array[6]->child_index);
  ASSERT_EQ(0, arm.collection_array[7]->child_index);

  ASSERT_EQ(3, arm.collection_array[0]->child_count);
  ASSERT_EQ(0, arm.collection_array[1]->child_count);
  ASSERT_EQ(3, arm.collection_array[2]->child_count);
  ASSERT_EQ(0, arm.collection_array[3]->child_count);
  ASSERT_EQ(0, arm.collection_array[4]->child_count);
  ASSERT_EQ(0, arm.collection_array[5]->child_count);
  ASSERT_EQ(0, arm.collection_array[6]->child_count);
  ASSERT_EQ(0, arm.collection_array[7]->child_count);

  /* Remove bcoll_r0_child0, which should make all of its children a child of root_0. */
  ANIM_armature_bonecoll_remove(&arm, bcoll_r0_child0);

  ASSERT_EQ(2, arm.collection_root_count);
  ASSERT_EQ(7, arm.collection_array_num);
  EXPECT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  EXPECT_STREQ(bcoll_root_1->name, arm.collection_array[1]->name);
  EXPECT_STREQ(bcoll_r0_child1->name, arm.collection_array[2]->name);
  EXPECT_STREQ(bcoll_r0_child2->name, arm.collection_array[3]->name);
  EXPECT_STREQ(bcoll_r0c0_child0->name, arm.collection_array[4]->name);
  EXPECT_STREQ(bcoll_r0c0_child1->name, arm.collection_array[5]->name);
  EXPECT_STREQ(bcoll_r0c0_child2->name, arm.collection_array[6]->name);

  EXPECT_EQ(2, arm.collection_array[0]->child_index);
  EXPECT_EQ(0, arm.collection_array[1]->child_index);
  EXPECT_EQ(0, arm.collection_array[2]->child_index);
  EXPECT_EQ(0, arm.collection_array[3]->child_index);
  EXPECT_EQ(0, arm.collection_array[4]->child_index);
  EXPECT_EQ(0, arm.collection_array[5]->child_index);
  EXPECT_EQ(0, arm.collection_array[6]->child_count);

  EXPECT_EQ(5, arm.collection_array[0]->child_count);
  EXPECT_EQ(0, arm.collection_array[1]->child_count);
  EXPECT_EQ(0, arm.collection_array[2]->child_count);
  EXPECT_EQ(0, arm.collection_array[3]->child_count);
  EXPECT_EQ(0, arm.collection_array[4]->child_count);
  EXPECT_EQ(0, arm.collection_array[5]->child_count);
  EXPECT_EQ(0, arm.collection_array[6]->child_count);

  /* Remove root_0, which should make all of its children new roots. */
  ANIM_armature_bonecoll_remove(&arm, bcoll_root_0);

  ASSERT_EQ(6, arm.collection_root_count);
  ASSERT_EQ(6, arm.collection_array_num);
  EXPECT_STREQ(bcoll_root_1->name, arm.collection_array[0]->name);
  EXPECT_STREQ(bcoll_r0_child1->name, arm.collection_array[1]->name);
  EXPECT_STREQ(bcoll_r0_child2->name, arm.collection_array[2]->name);
  EXPECT_STREQ(bcoll_r0c0_child0->name, arm.collection_array[3]->name);
  EXPECT_STREQ(bcoll_r0c0_child1->name, arm.collection_array[4]->name);
  EXPECT_STREQ(bcoll_r0c0_child2->name, arm.collection_array[5]->name);

  EXPECT_EQ(0, arm.collection_array[0]->child_index);
  EXPECT_EQ(0, arm.collection_array[1]->child_index);
  EXPECT_EQ(0, arm.collection_array[2]->child_index);
  EXPECT_EQ(0, arm.collection_array[3]->child_index);
  EXPECT_EQ(0, arm.collection_array[4]->child_index);
  EXPECT_EQ(0, arm.collection_array[5]->child_index);

  EXPECT_EQ(0, arm.collection_array[0]->child_count);
  EXPECT_EQ(0, arm.collection_array[1]->child_count);
  EXPECT_EQ(0, arm.collection_array[2]->child_count);
  EXPECT_EQ(0, arm.collection_array[3]->child_count);
  EXPECT_EQ(0, arm.collection_array[4]->child_count);
  EXPECT_EQ(0, arm.collection_array[5]->child_count);
}

TEST_F(ANIM_armature_bone_collections, collection_hierarchy_removal__more_complex_remove_root)
{
  /* Set up a slightly bigger hierarchy. Contrary to the other tests these are
   * actually declared in array order. */
  BoneCollection *bcoll_root_0 = ANIM_armature_bonecoll_new(&arm, "root_0");
  BoneCollection *bcoll_root_1 = ANIM_armature_bonecoll_new(&arm, "root_1");
  BoneCollection *bcoll_r0_child0 = ANIM_armature_bonecoll_new(&arm, "r0_child0", 0);
  BoneCollection *bcoll_r0_child1 = ANIM_armature_bonecoll_new(&arm, "r0_child1", 0);
  BoneCollection *bcoll_r0_child2 = ANIM_armature_bonecoll_new(&arm, "r0_child2", 0);
  BoneCollection *bcoll_r0c0_child0 = ANIM_armature_bonecoll_new(&arm, "r0c0_child0", 2);
  BoneCollection *bcoll_r0c0_child1 = ANIM_armature_bonecoll_new(&arm, "r0c0_child1", 2);
  BoneCollection *bcoll_r0c0_child2 = ANIM_armature_bonecoll_new(&arm, "r0c0_child2", 2);

  ASSERT_EQ(2, arm.collection_root_count);
  ASSERT_EQ(8, arm.collection_array_num);
  ASSERT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  ASSERT_STREQ(bcoll_root_1->name, arm.collection_array[1]->name);
  ASSERT_STREQ(bcoll_r0_child0->name, arm.collection_array[2]->name);
  ASSERT_STREQ(bcoll_r0_child1->name, arm.collection_array[3]->name);
  ASSERT_STREQ(bcoll_r0_child2->name, arm.collection_array[4]->name);
  ASSERT_STREQ(bcoll_r0c0_child0->name, arm.collection_array[5]->name);
  ASSERT_STREQ(bcoll_r0c0_child1->name, arm.collection_array[6]->name);
  ASSERT_STREQ(bcoll_r0c0_child2->name, arm.collection_array[7]->name);

  ASSERT_EQ(2, arm.collection_array[0]->child_index);
  ASSERT_EQ(0, arm.collection_array[1]->child_index);
  ASSERT_EQ(5, arm.collection_array[2]->child_index);
  ASSERT_EQ(0, arm.collection_array[3]->child_index);
  ASSERT_EQ(0, arm.collection_array[4]->child_index);
  ASSERT_EQ(0, arm.collection_array[5]->child_index);
  ASSERT_EQ(0, arm.collection_array[6]->child_index);
  ASSERT_EQ(0, arm.collection_array[7]->child_index);

  ASSERT_EQ(3, arm.collection_array[0]->child_count);
  ASSERT_EQ(0, arm.collection_array[1]->child_count);
  ASSERT_EQ(3, arm.collection_array[2]->child_count);
  ASSERT_EQ(0, arm.collection_array[3]->child_count);
  ASSERT_EQ(0, arm.collection_array[4]->child_count);
  ASSERT_EQ(0, arm.collection_array[5]->child_count);
  ASSERT_EQ(0, arm.collection_array[6]->child_count);
  ASSERT_EQ(0, arm.collection_array[7]->child_count);

  /* Remove root_0, which should make all of its children new roots. */
  ANIM_armature_bonecoll_remove(&arm, bcoll_root_0);

  ASSERT_EQ(4, arm.collection_root_count);
  ASSERT_EQ(7, arm.collection_array_num);
  EXPECT_STREQ(bcoll_root_1->name, arm.collection_array[0]->name);
  EXPECT_STREQ(bcoll_r0_child0->name, arm.collection_array[1]->name);
  EXPECT_STREQ(bcoll_r0_child1->name, arm.collection_array[2]->name);
  EXPECT_STREQ(bcoll_r0_child2->name, arm.collection_array[3]->name);
  EXPECT_STREQ(bcoll_r0c0_child0->name, arm.collection_array[4]->name);
  EXPECT_STREQ(bcoll_r0c0_child1->name, arm.collection_array[5]->name);
  EXPECT_STREQ(bcoll_r0c0_child2->name, arm.collection_array[6]->name);

  EXPECT_EQ(0, arm.collection_array[0]->child_index);
  EXPECT_EQ(4, arm.collection_array[1]->child_index);
  EXPECT_EQ(0, arm.collection_array[2]->child_index);
  EXPECT_EQ(0, arm.collection_array[3]->child_index);
  EXPECT_EQ(0, arm.collection_array[4]->child_index);
  EXPECT_EQ(0, arm.collection_array[5]->child_index);
  EXPECT_EQ(0, arm.collection_array[6]->child_count);

  EXPECT_EQ(0, arm.collection_array[0]->child_count);
  EXPECT_EQ(3, arm.collection_array[1]->child_count);
  EXPECT_EQ(0, arm.collection_array[2]->child_count);
  EXPECT_EQ(0, arm.collection_array[3]->child_count);
  EXPECT_EQ(0, arm.collection_array[4]->child_count);
  EXPECT_EQ(0, arm.collection_array[5]->child_count);
  EXPECT_EQ(0, arm.collection_array[6]->child_count);
}

TEST_F(ANIM_armature_bone_collections, find_parent_index)
{
  /* Set up a small hierarchy. */
  BoneCollection *bcoll_root_0 = ANIM_armature_bonecoll_new(&arm, "root_0");
  BoneCollection *bcoll_root_1 = ANIM_armature_bonecoll_new(&arm, "root_1");
  BoneCollection *bcoll_r0_child0 = ANIM_armature_bonecoll_new(&arm, "r0_child0", 0);
  BoneCollection *bcoll_r1_child0 = ANIM_armature_bonecoll_new(&arm, "r1_child0", 1);
  BoneCollection *bcoll_r0_child1 = ANIM_armature_bonecoll_new(&arm, "r0_child1", 0);
  BoneCollection *bcoll_r0c0_child0 = ANIM_armature_bonecoll_new(&arm, "r0c0_child0", 2);

  ASSERT_EQ(2, arm.collection_root_count);
  ASSERT_EQ(6, arm.collection_array_num);
  ASSERT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  ASSERT_STREQ(bcoll_root_1->name, arm.collection_array[1]->name);
  ASSERT_STREQ(bcoll_r0_child0->name, arm.collection_array[2]->name);
  ASSERT_STREQ(bcoll_r0_child1->name, arm.collection_array[3]->name);
  ASSERT_STREQ(bcoll_r1_child0->name, arm.collection_array[4]->name);
  ASSERT_STREQ(bcoll_r0c0_child0->name, arm.collection_array[5]->name);

  ASSERT_EQ(2, arm.collection_array[0]->child_index);
  ASSERT_EQ(4, arm.collection_array[1]->child_index);
  ASSERT_EQ(5, arm.collection_array[2]->child_index);
  ASSERT_EQ(0, arm.collection_array[3]->child_index);
  ASSERT_EQ(0, arm.collection_array[4]->child_index);
  ASSERT_EQ(0, arm.collection_array[5]->child_index);

  ASSERT_EQ(2, arm.collection_array[0]->child_count);
  ASSERT_EQ(1, arm.collection_array[1]->child_count);
  ASSERT_EQ(1, arm.collection_array[2]->child_count);
  ASSERT_EQ(0, arm.collection_array[3]->child_count);
  ASSERT_EQ(0, arm.collection_array[4]->child_count);
  ASSERT_EQ(0, arm.collection_array[5]->child_count);

  EXPECT_EQ(-1, armature_bonecoll_find_parent_index(&arm, -1));
  EXPECT_EQ(-1, armature_bonecoll_find_parent_index(&arm, 500000));

  EXPECT_EQ(-1, armature_bonecoll_find_parent_index(&arm, 0));
  EXPECT_EQ(-1, armature_bonecoll_find_parent_index(&arm, 1));
  EXPECT_EQ(0, armature_bonecoll_find_parent_index(&arm, 2));
  EXPECT_EQ(0, armature_bonecoll_find_parent_index(&arm, 3));
  EXPECT_EQ(1, armature_bonecoll_find_parent_index(&arm, 4));
  EXPECT_EQ(2, armature_bonecoll_find_parent_index(&arm, 5));
}

TEST_F(ANIM_armature_bone_collections, bones_assign_unassign)
{
  BoneCollection *bcoll = ANIM_armature_bonecoll_new(&arm, "collection");

  ANIM_armature_bonecoll_assign(bcoll, &bone1);
  ANIM_armature_bonecoll_assign(bcoll, &bone2);

  ASSERT_EQ(2, BLI_listbase_count(&bcoll->bones)) << "expecting two bones in collection";
  EXPECT_EQ(&bone1, static_cast<BoneCollectionMember *>(BLI_findlink(&bcoll->bones, 0))->bone);
  EXPECT_EQ(&bone2, static_cast<BoneCollectionMember *>(BLI_findlink(&bcoll->bones, 1))->bone);

  EXPECT_EQ(bcoll, static_cast<BoneCollectionReference *>(bone1.runtime.collections.first)->bcoll)
      << "expecting back-reference to collection in bone1 runtime data";
  EXPECT_EQ(bcoll, static_cast<BoneCollectionReference *>(bone2.runtime.collections.first)->bcoll)
      << "expecting back-reference to collection in bone2 runtime data";

  ANIM_armature_bonecoll_unassign(bcoll, &bone1);
  ANIM_armature_bonecoll_unassign(bcoll, &bone2);

  EXPECT_EQ(0, BLI_listbase_count(&bone1.runtime.collections))
      << "expecting back-references in bone1 runtime data to be cleared when unassigned";
  EXPECT_EQ(0, BLI_listbase_count(&bone2.runtime.collections))
      << "expecting back-references in bone2 runtime data to be cleared when unassigned";

  ANIM_armature_bonecoll_remove(&arm, bcoll);
}

TEST_F(ANIM_armature_bone_collections, bones_assign_remove)
{
  BoneCollection *bcoll = ANIM_armature_bonecoll_new(&arm, "collection");

  ANIM_armature_bonecoll_assign(bcoll, &bone1);
  ANIM_armature_bonecoll_assign(bcoll, &bone2);
  ANIM_armature_bonecoll_remove(&arm, bcoll);

  EXPECT_EQ(0, BLI_listbase_count(&bone1.runtime.collections))
      << "expecting back-references in bone1 runtime data to be cleared when the collection is "
         "removed";
  EXPECT_EQ(0, BLI_listbase_count(&bone2.runtime.collections))
      << "expecting back-references in bone2 runtime data to be cleared when the collection is "
         "removed";
}

TEST_F(ANIM_armature_bone_collections, active_set_clear_by_pointer)
{
  BoneCollection *bcoll1 = ANIM_armature_bonecoll_new(&arm, "Bones 1");
  BoneCollection *bcoll2 = ANIM_armature_bonecoll_new(&arm, "Bones 2");
  BoneCollection *bcoll3 = ANIM_bonecoll_new("Alien Bones");

  ANIM_armature_bonecoll_active_set(&arm, bcoll1);
  EXPECT_EQ(0, arm.runtime.active_collection_index);
  EXPECT_EQ(bcoll1, arm.runtime.active_collection);
  EXPECT_STREQ(bcoll1->name, arm.active_collection_name);

  ANIM_armature_bonecoll_active_set(&arm, nullptr);
  EXPECT_EQ(-1, arm.runtime.active_collection_index);
  EXPECT_EQ(nullptr, arm.runtime.active_collection);
  EXPECT_STREQ("", arm.active_collection_name);

  ANIM_armature_bonecoll_active_set(&arm, bcoll2);
  EXPECT_EQ(1, arm.runtime.active_collection_index);
  EXPECT_EQ(bcoll2, arm.runtime.active_collection);
  EXPECT_STREQ(bcoll2->name, arm.active_collection_name);

  ANIM_armature_bonecoll_active_set(&arm, bcoll3);
  EXPECT_EQ(-1, arm.runtime.active_collection_index);
  EXPECT_EQ(nullptr, arm.runtime.active_collection);
  EXPECT_STREQ("", arm.active_collection_name);

  ANIM_bonecoll_free(bcoll3);
}

TEST_F(ANIM_armature_bone_collections, active_set_clear_by_index)
{
  BoneCollection *bcoll1 = ANIM_armature_bonecoll_new(&arm, "Bones 1");
  BoneCollection *bcoll2 = ANIM_armature_bonecoll_new(&arm, "Bones 2");

  ANIM_armature_bonecoll_active_index_set(&arm, 0);
  EXPECT_EQ(0, arm.runtime.active_collection_index);
  EXPECT_EQ(bcoll1, arm.runtime.active_collection);
  EXPECT_STREQ(bcoll1->name, arm.active_collection_name);

  ANIM_armature_bonecoll_active_index_set(&arm, -1);
  EXPECT_EQ(-1, arm.runtime.active_collection_index);
  EXPECT_EQ(nullptr, arm.runtime.active_collection);
  EXPECT_STREQ("", arm.active_collection_name);

  ANIM_armature_bonecoll_active_index_set(&arm, 1);
  EXPECT_EQ(1, arm.runtime.active_collection_index);
  EXPECT_EQ(bcoll2, arm.runtime.active_collection);
  EXPECT_STREQ(bcoll2->name, arm.active_collection_name);

  ANIM_armature_bonecoll_active_index_set(&arm, 47);
  EXPECT_EQ(-1, arm.runtime.active_collection_index);
  EXPECT_EQ(nullptr, arm.runtime.active_collection);
  EXPECT_STREQ("", arm.active_collection_name);
}

TEST_F(ANIM_armature_bone_collections, bcoll_is_editable)
{
  BoneCollection *bcoll1 = ANIM_armature_bonecoll_new(&arm, "Bones 1");
  BoneCollection *bcoll2 = ANIM_armature_bonecoll_new(&arm, "Bones 2");

  EXPECT_EQ(0, bcoll1->flags & BONE_COLLECTION_OVERRIDE_LIBRARY_LOCAL);
  EXPECT_EQ(0, bcoll2->flags & BONE_COLLECTION_OVERRIDE_LIBRARY_LOCAL);

  EXPECT_TRUE(ANIM_armature_bonecoll_is_editable(&arm, bcoll1))
      << "Expecting local armature to be editable";

  /* Fake that the armature is linked from another blend file. */
  Library fake_lib;
  arm.id.lib = &fake_lib;
  EXPECT_FALSE(ANIM_armature_bonecoll_is_editable(&arm, bcoll1))
      << "Expecting local armature to not be editable";

  /* Fake that the armature is an override, but linked from another blend file. */
  IDOverrideLibrary fake_override;
  bArmature fake_reference;
  fake_override.reference = &fake_reference.id;
  arm.id.override_library = &fake_override;
  EXPECT_FALSE(ANIM_armature_bonecoll_is_editable(&arm, bcoll1))
      << "Expecting linked armature override to not be editable";

  /* Fake that the armature is a local override. */
  arm.id.lib = nullptr;
  bcoll2->flags |= BONE_COLLECTION_OVERRIDE_LIBRARY_LOCAL;
  EXPECT_FALSE(ANIM_armature_bonecoll_is_editable(&arm, bcoll1))
      << "Expecting linked bone collection on local armature override to not be editable";
  EXPECT_TRUE(ANIM_armature_bonecoll_is_editable(&arm, bcoll2))
      << "Expecting local bone collection on local armature override to be editable";
}

TEST_F(ANIM_armature_bone_collections, bcoll_insert_copy_after)
{
  BoneCollection *bcoll1 = ANIM_armature_bonecoll_new(&arm, "collection");
  BoneCollection *bcoll2 = ANIM_armature_bonecoll_new(&arm, "collection");
  BoneCollection *bcoll3 = ANIM_armature_bonecoll_new(&arm, "collection");

  EXPECT_EQ(arm.collection_array[0], bcoll1);
  EXPECT_EQ(arm.collection_array[1], bcoll2);
  EXPECT_EQ(arm.collection_array[2], bcoll3);

  BoneCollection *bcoll4 = ANIM_armature_bonecoll_insert_copy_after(&arm, bcoll2, bcoll2);

  EXPECT_EQ(arm.collection_array[0], bcoll1);
  EXPECT_EQ(arm.collection_array[1], bcoll2);
  EXPECT_EQ(arm.collection_array[2], bcoll4);
  EXPECT_EQ(arm.collection_array[3], bcoll3);
}

TEST_F(ANIM_armature_bone_collections, bcoll_move_to_index)
{
  BoneCollection *bcoll1 = ANIM_armature_bonecoll_new(&arm, "collection");
  BoneCollection *bcoll2 = ANIM_armature_bonecoll_new(&arm, "collection");
  BoneCollection *bcoll3 = ANIM_armature_bonecoll_new(&arm, "collection");
  BoneCollection *bcoll4 = ANIM_armature_bonecoll_new(&arm, "collection");

  EXPECT_EQ(arm.collection_array[0], bcoll1);
  EXPECT_EQ(arm.collection_array[1], bcoll2);
  EXPECT_EQ(arm.collection_array[2], bcoll3);
  EXPECT_EQ(arm.collection_array[3], bcoll4);

  ANIM_armature_bonecoll_move_to_index(&arm, 2, 1);

  EXPECT_EQ(arm.collection_array[0], bcoll1);
  EXPECT_EQ(arm.collection_array[1], bcoll3);
  EXPECT_EQ(arm.collection_array[2], bcoll2);
  EXPECT_EQ(arm.collection_array[3], bcoll4);

  ANIM_armature_bonecoll_move_to_index(&arm, 0, 3);

  EXPECT_EQ(arm.collection_array[0], bcoll3);
  EXPECT_EQ(arm.collection_array[1], bcoll2);
  EXPECT_EQ(arm.collection_array[2], bcoll4);
  EXPECT_EQ(arm.collection_array[3], bcoll1);
}

TEST_F(ANIM_armature_bone_collections, bcoll_move_to_parent)
{
  /* Set up a small hierarchy. */
  BoneCollection *bcoll_root_0 = ANIM_armature_bonecoll_new(&arm, "root_0");
  BoneCollection *bcoll_root_1 = ANIM_armature_bonecoll_new(&arm, "root_1");
  BoneCollection *bcoll_r0_child0 = ANIM_armature_bonecoll_new(&arm, "r0_child0", 0);
  BoneCollection *bcoll_r1_child0 = ANIM_armature_bonecoll_new(&arm, "r1_child0", 1);
  BoneCollection *bcoll_r0_child1 = ANIM_armature_bonecoll_new(&arm, "r0_child1", 0);
  BoneCollection *bcoll_r0_child2 = ANIM_armature_bonecoll_new(&arm, "r0_child2", 0);

  ASSERT_EQ(2, arm.collection_root_count);
  ASSERT_EQ(6, arm.collection_array_num);
  ASSERT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  ASSERT_STREQ(bcoll_root_1->name, arm.collection_array[1]->name);
  ASSERT_STREQ(bcoll_r0_child0->name, arm.collection_array[2]->name);
  ASSERT_STREQ(bcoll_r0_child1->name, arm.collection_array[3]->name);
  ASSERT_STREQ(bcoll_r0_child2->name, arm.collection_array[4]->name);
  ASSERT_STREQ(bcoll_r1_child0->name, arm.collection_array[5]->name);

  ASSERT_EQ(2, arm.collection_array[0]->child_index);
  ASSERT_EQ(5, arm.collection_array[1]->child_index);
  ASSERT_EQ(0, arm.collection_array[2]->child_index);
  ASSERT_EQ(0, arm.collection_array[3]->child_index);
  ASSERT_EQ(0, arm.collection_array[4]->child_index);
  ASSERT_EQ(0, arm.collection_array[5]->child_index);

  ASSERT_EQ(3, arm.collection_array[0]->child_count);
  ASSERT_EQ(1, arm.collection_array[1]->child_count);
  ASSERT_EQ(0, arm.collection_array[2]->child_count);
  ASSERT_EQ(0, arm.collection_array[3]->child_count);
  ASSERT_EQ(0, arm.collection_array[4]->child_count);
  ASSERT_EQ(0, arm.collection_array[5]->child_count);

  /* Move the middle child of root_0 to root_1. */
  EXPECT_EQ(5, armature_bonecoll_move_to_parent(&arm, 3, 0, 1));

  ASSERT_EQ(2, arm.collection_root_count);
  ASSERT_EQ(6, arm.collection_array_num);
  EXPECT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  EXPECT_STREQ(bcoll_root_1->name, arm.collection_array[1]->name);
  EXPECT_STREQ(bcoll_r0_child0->name, arm.collection_array[2]->name);
  EXPECT_STREQ(bcoll_r0_child2->name, arm.collection_array[3]->name);
  EXPECT_STREQ(bcoll_r1_child0->name, arm.collection_array[4]->name);
  EXPECT_STREQ(bcoll_r0_child1->name, arm.collection_array[5]->name);

  EXPECT_EQ(2, arm.collection_array[0]->child_index);
  EXPECT_EQ(4, arm.collection_array[1]->child_index);
  EXPECT_EQ(0, arm.collection_array[2]->child_index);
  EXPECT_EQ(0, arm.collection_array[3]->child_index);
  EXPECT_EQ(0, arm.collection_array[4]->child_index);
  EXPECT_EQ(0, arm.collection_array[5]->child_index);

  EXPECT_EQ(2, arm.collection_array[0]->child_count);
  EXPECT_EQ(2, arm.collection_array[1]->child_count);
  EXPECT_EQ(0, arm.collection_array[2]->child_count);
  EXPECT_EQ(0, arm.collection_array[3]->child_count);
  EXPECT_EQ(0, arm.collection_array[4]->child_count);
  EXPECT_EQ(0, arm.collection_array[5]->child_count);

  /* Move the first child of root_1 to root_0. This shouldn't change its index. */
  EXPECT_EQ(4, armature_bonecoll_move_to_parent(&arm, 4, 1, 0));

  ASSERT_EQ(2, arm.collection_root_count);
  ASSERT_EQ(6, arm.collection_array_num);
  EXPECT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  EXPECT_STREQ(bcoll_root_1->name, arm.collection_array[1]->name);
  EXPECT_STREQ(bcoll_r0_child0->name, arm.collection_array[2]->name);
  EXPECT_STREQ(bcoll_r0_child2->name, arm.collection_array[3]->name);
  EXPECT_STREQ(bcoll_r1_child0->name, arm.collection_array[4]->name);
  EXPECT_STREQ(bcoll_r0_child1->name, arm.collection_array[5]->name);

  EXPECT_EQ(2, arm.collection_array[0]->child_index);
  EXPECT_EQ(5, arm.collection_array[1]->child_index);
  EXPECT_EQ(0, arm.collection_array[2]->child_index);
  EXPECT_EQ(0, arm.collection_array[3]->child_index);
  EXPECT_EQ(0, arm.collection_array[4]->child_index);
  EXPECT_EQ(0, arm.collection_array[5]->child_index);

  EXPECT_EQ(3, arm.collection_array[0]->child_count);
  EXPECT_EQ(1, arm.collection_array[1]->child_count);
  EXPECT_EQ(0, arm.collection_array[2]->child_count);
  EXPECT_EQ(0, arm.collection_array[3]->child_count);
  EXPECT_EQ(0, arm.collection_array[4]->child_count);
  EXPECT_EQ(0, arm.collection_array[5]->child_count);

  /* Move the final child of root_1 to root_0. This shouldn't change its index
   * again, but leave root_1 without children. */
  EXPECT_EQ(5, armature_bonecoll_move_to_parent(&arm, 5, 1, 0));

  ASSERT_EQ(2, arm.collection_root_count);
  ASSERT_EQ(6, arm.collection_array_num);
  EXPECT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  EXPECT_STREQ(bcoll_root_1->name, arm.collection_array[1]->name);
  EXPECT_STREQ(bcoll_r0_child0->name, arm.collection_array[2]->name);
  EXPECT_STREQ(bcoll_r0_child2->name, arm.collection_array[3]->name);
  EXPECT_STREQ(bcoll_r1_child0->name, arm.collection_array[4]->name);
  EXPECT_STREQ(bcoll_r0_child1->name, arm.collection_array[5]->name);

  EXPECT_EQ(2, arm.collection_array[0]->child_index);
  EXPECT_EQ(0, arm.collection_array[1]->child_index);
  EXPECT_EQ(0, arm.collection_array[2]->child_index);
  EXPECT_EQ(0, arm.collection_array[3]->child_index);
  EXPECT_EQ(0, arm.collection_array[4]->child_index);
  EXPECT_EQ(0, arm.collection_array[5]->child_index);

  EXPECT_EQ(4, arm.collection_array[0]->child_count);
  EXPECT_EQ(0, arm.collection_array[1]->child_count);
  EXPECT_EQ(0, arm.collection_array[2]->child_count);
  EXPECT_EQ(0, arm.collection_array[3]->child_count);
  EXPECT_EQ(0, arm.collection_array[4]->child_count);
  EXPECT_EQ(0, arm.collection_array[5]->child_count);

  /* Move the first child of root_0 (bcoll_r0_child0) to bcoll_r0_child2. */
  EXPECT_EQ(5, armature_bonecoll_move_to_parent(&arm, 2, 0, 3));

  ASSERT_EQ(2, arm.collection_root_count);
  ASSERT_EQ(6, arm.collection_array_num);
  EXPECT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  EXPECT_STREQ(bcoll_root_1->name, arm.collection_array[1]->name);
  EXPECT_STREQ(bcoll_r0_child2->name, arm.collection_array[2]->name);
  EXPECT_STREQ(bcoll_r1_child0->name, arm.collection_array[3]->name);
  EXPECT_STREQ(bcoll_r0_child1->name, arm.collection_array[4]->name);
  EXPECT_STREQ(bcoll_r0_child0->name, arm.collection_array[5]->name);

  EXPECT_EQ(2, arm.collection_array[0]->child_index);
  EXPECT_EQ(0, arm.collection_array[1]->child_index);
  EXPECT_EQ(5, arm.collection_array[2]->child_index);
  EXPECT_EQ(0, arm.collection_array[3]->child_index);
  EXPECT_EQ(0, arm.collection_array[4]->child_index);
  EXPECT_EQ(0, arm.collection_array[5]->child_index);

  EXPECT_EQ(3, arm.collection_array[0]->child_count);
  EXPECT_EQ(0, arm.collection_array[1]->child_count);
  EXPECT_EQ(1, arm.collection_array[2]->child_count);
  EXPECT_EQ(0, arm.collection_array[3]->child_count);
  EXPECT_EQ(0, arm.collection_array[4]->child_count);
  EXPECT_EQ(0, arm.collection_array[5]->child_count);
}

TEST_F(ANIM_armature_bone_collections, bcoll_move_to_parent__root_unroot)
{
  /* Set up a small hierarchy. */
  BoneCollection *bcoll_root_0 = ANIM_armature_bonecoll_new(&arm, "root_0");
  BoneCollection *bcoll_root_1 = ANIM_armature_bonecoll_new(&arm, "root_1");
  BoneCollection *bcoll_r0_child0 = ANIM_armature_bonecoll_new(&arm, "r0_child0", 0);
  BoneCollection *bcoll_r1_child0 = ANIM_armature_bonecoll_new(&arm, "r1_child0", 1);
  BoneCollection *bcoll_r0_child1 = ANIM_armature_bonecoll_new(&arm, "r0_child1", 0);
  BoneCollection *bcoll_r0_child2 = ANIM_armature_bonecoll_new(&arm, "r0_child2", 0);

  ASSERT_EQ(2, arm.collection_root_count);
  ASSERT_EQ(6, arm.collection_array_num);
  ASSERT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  ASSERT_STREQ(bcoll_root_1->name, arm.collection_array[1]->name);
  ASSERT_STREQ(bcoll_r0_child0->name, arm.collection_array[2]->name);
  ASSERT_STREQ(bcoll_r0_child1->name, arm.collection_array[3]->name);
  ASSERT_STREQ(bcoll_r0_child2->name, arm.collection_array[4]->name);
  ASSERT_STREQ(bcoll_r1_child0->name, arm.collection_array[5]->name);

  ASSERT_EQ(2, arm.collection_array[0]->child_index);
  ASSERT_EQ(5, arm.collection_array[1]->child_index);
  ASSERT_EQ(0, arm.collection_array[2]->child_index);
  ASSERT_EQ(0, arm.collection_array[3]->child_index);
  ASSERT_EQ(0, arm.collection_array[4]->child_index);
  ASSERT_EQ(0, arm.collection_array[5]->child_index);

  ASSERT_EQ(3, arm.collection_array[0]->child_count);
  ASSERT_EQ(1, arm.collection_array[1]->child_count);
  ASSERT_EQ(0, arm.collection_array[2]->child_count);
  ASSERT_EQ(0, arm.collection_array[3]->child_count);
  ASSERT_EQ(0, arm.collection_array[4]->child_count);
  ASSERT_EQ(0, arm.collection_array[5]->child_count);

  /* Make a leaf node (bcoll_r0_child1) a root. */
  EXPECT_EQ(2, armature_bonecoll_move_to_parent(&arm, 3, 0, -1));

  ASSERT_EQ(3, arm.collection_root_count);
  ASSERT_EQ(6, arm.collection_array_num);
  EXPECT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  EXPECT_STREQ(bcoll_root_1->name, arm.collection_array[1]->name);
  EXPECT_STREQ(bcoll_r0_child1->name, arm.collection_array[2]->name);  // Became a root.
  EXPECT_STREQ(bcoll_r0_child0->name, arm.collection_array[3]->name);
  EXPECT_STREQ(bcoll_r0_child2->name, arm.collection_array[4]->name);
  EXPECT_STREQ(bcoll_r1_child0->name, arm.collection_array[5]->name);

  EXPECT_EQ(3, arm.collection_array[0]->child_index);
  EXPECT_EQ(5, arm.collection_array[1]->child_index);
  EXPECT_EQ(0, arm.collection_array[2]->child_index);
  EXPECT_EQ(0, arm.collection_array[3]->child_index);
  EXPECT_EQ(0, arm.collection_array[4]->child_index);
  EXPECT_EQ(0, arm.collection_array[5]->child_index);

  EXPECT_EQ(2, arm.collection_array[0]->child_count);
  EXPECT_EQ(1, arm.collection_array[1]->child_count);
  EXPECT_EQ(0, arm.collection_array[2]->child_count);
  EXPECT_EQ(0, arm.collection_array[3]->child_count);
  EXPECT_EQ(0, arm.collection_array[4]->child_count);
  EXPECT_EQ(0, arm.collection_array[5]->child_count);

  /* Make a root node (root_1) a child of root_0. */
  EXPECT_EQ(4, armature_bonecoll_move_to_parent(&arm, 1, -1, 0));

  ASSERT_EQ(2, arm.collection_root_count);
  ASSERT_EQ(6, arm.collection_array_num);
  EXPECT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  EXPECT_STREQ(bcoll_r0_child1->name, arm.collection_array[1]->name);  // Actually a root.
  EXPECT_STREQ(bcoll_r0_child0->name, arm.collection_array[2]->name);
  EXPECT_STREQ(bcoll_r0_child2->name, arm.collection_array[3]->name);
  EXPECT_STREQ(bcoll_root_1->name, arm.collection_array[4]->name);  // Became a child.
  EXPECT_STREQ(bcoll_r1_child0->name, arm.collection_array[5]->name);

  EXPECT_EQ(2, arm.collection_array[0]->child_index);
  EXPECT_EQ(0, arm.collection_array[1]->child_index);
  EXPECT_EQ(0, arm.collection_array[2]->child_index);
  EXPECT_EQ(0, arm.collection_array[3]->child_index);
  EXPECT_EQ(5, arm.collection_array[4]->child_index);
  EXPECT_EQ(0, arm.collection_array[5]->child_index);

  EXPECT_EQ(3, arm.collection_array[0]->child_count);
  EXPECT_EQ(0, arm.collection_array[1]->child_count);
  EXPECT_EQ(0, arm.collection_array[2]->child_count);
  EXPECT_EQ(0, arm.collection_array[3]->child_count);
  EXPECT_EQ(1, arm.collection_array[4]->child_count);
  EXPECT_EQ(0, arm.collection_array[5]->child_count);

  // TODO: test with circular parenthood.
}

TEST_F(ANIM_armature_bone_collections, internal__bonecolls_rotate_block)
{
  /* Set up a small hierarchy. */
  BoneCollection *bcoll_root_0 = ANIM_armature_bonecoll_new(&arm, "root_0");
  BoneCollection *bcoll_root_1 = ANIM_armature_bonecoll_new(&arm, "root_1");
  BoneCollection *bcoll_r0_child0 = ANIM_armature_bonecoll_new(&arm, "r0_child0", 0);
  BoneCollection *bcoll_r1_child0 = ANIM_armature_bonecoll_new(&arm, "r1_child0", 1);
  BoneCollection *bcoll_r0_child1 = ANIM_armature_bonecoll_new(&arm, "r0_child1", 0);
  BoneCollection *bcoll_r0_child2 = ANIM_armature_bonecoll_new(&arm, "r0_child2", 0);

  /* The tests below compare the collection names, instead of their pointers, so
   * that we get human-readable messages on failure. */

  /* Unnecessary assertions, just to make it easier to understand in which order
   * the array starts out. */
  ASSERT_EQ(6, arm.collection_array_num);
  ASSERT_STREQ(bcoll_root_0->name, arm.collection_array[0]->name);
  ASSERT_STREQ(bcoll_root_1->name, arm.collection_array[1]->name);
  ASSERT_STREQ(bcoll_r0_child0->name, arm.collection_array[2]->name);
  ASSERT_STREQ(bcoll_r0_child1->name, arm.collection_array[3]->name);
  ASSERT_STREQ(bcoll_r0_child2->name, arm.collection_array[4]->name);
  ASSERT_STREQ(bcoll_r1_child0->name, arm.collection_array[5]->name);

  ASSERT_EQ(2, arm.collection_array[0]->child_index);
  ASSERT_EQ(5, arm.collection_array[1]->child_index);
  ASSERT_EQ(0, arm.collection_array[2]->child_index);
  ASSERT_EQ(0, arm.collection_array[3]->child_index);
  ASSERT_EQ(0, arm.collection_array[4]->child_index);
  ASSERT_EQ(0, arm.collection_array[5]->child_index);

  /* Move [0,1,2] to [1,2,3]. */
  internal::bonecolls_rotate_block(&arm, 0, 3, 1);
  ASSERT_EQ(6, arm.collection_array_num) << "array size should not change";
  EXPECT_STREQ(bcoll_r0_child1->name, arm.collection_array[0]->name);
  EXPECT_STREQ(bcoll_root_0->name, arm.collection_array[1]->name);
  EXPECT_STREQ(bcoll_root_1->name, arm.collection_array[2]->name);
  EXPECT_STREQ(bcoll_r0_child0->name, arm.collection_array[3]->name);
  EXPECT_STREQ(bcoll_r0_child2->name, arm.collection_array[4]->name);
  EXPECT_STREQ(bcoll_r1_child0->name, arm.collection_array[5]->name);

  EXPECT_EQ(0, arm.collection_array[0]->child_index);
  EXPECT_EQ(3, arm.collection_array[1]->child_index);
  EXPECT_EQ(5, arm.collection_array[2]->child_index);
  EXPECT_EQ(0, arm.collection_array[3]->child_index);
  EXPECT_EQ(0, arm.collection_array[4]->child_index);
  EXPECT_EQ(0, arm.collection_array[5]->child_index);

  /* Move [4,5] to [3,4]. */
  internal::bonecolls_rotate_block(&arm, 4, 2, -1);
  ASSERT_EQ(6, arm.collection_array_num) << "array size should not change";
  EXPECT_STREQ(bcoll_r0_child1->name, arm.collection_array[0]->name);
  EXPECT_STREQ(bcoll_root_0->name, arm.collection_array[1]->name);
  EXPECT_STREQ(bcoll_root_1->name, arm.collection_array[2]->name);
  EXPECT_STREQ(bcoll_r0_child2->name, arm.collection_array[3]->name);
  EXPECT_STREQ(bcoll_r1_child0->name, arm.collection_array[4]->name);
  EXPECT_STREQ(bcoll_r0_child0->name, arm.collection_array[5]->name);

  EXPECT_EQ(0, arm.collection_array[0]->child_index);
  EXPECT_EQ(3, arm.collection_array[1]->child_index);
  EXPECT_EQ(4, arm.collection_array[2]->child_index);
  EXPECT_EQ(0, arm.collection_array[3]->child_index);
  EXPECT_EQ(0, arm.collection_array[4]->child_index);
  EXPECT_EQ(0, arm.collection_array[5]->child_index);
}

}  // namespace blender::animrig::tests
