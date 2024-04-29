/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <string.h>

#include "BKE_anim_data.hh"
#include "BKE_animation.hh"
#include "BKE_fcurve.hh"
#include "BKE_idtype.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_object.hh"

#include "DNA_anim_types.h"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "BLI_listbase.h"
#include "BLI_string.h"

#include "ED_keyframing.hh"

#include "ANIM_action.hh"
#include "ANIM_animdata.hh"
#include "ANIM_fcurve.hh"

#include "interface_intern.hh"

#include "CLG_log.h"
#include "testing/testing.h"

namespace blender::interface::tests {

class CopyDriversToSelected : public testing::Test {
 public:
  Main *bmain;

  Object *cube;
  Object *suzanne;

  PointerRNA cube_ptr;
  PropertyRNA *cube_location_prop;
  PropertyRNA *cube_rotation_mode_prop;

  PointerRNA suzanne_ptr;
  PropertyRNA *suzanne_location_prop;
  PropertyRNA *suzanne_rotation_mode_prop;

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

    cube = BKE_object_add_only_object(bmain, OB_EMPTY, "OBCube");
    suzanne = BKE_object_add_only_object(bmain, OB_EMPTY, "OBSuzanne");

    cube_ptr = RNA_pointer_create(&cube->id, &RNA_Object, &cube->id);
    cube_location_prop = RNA_struct_find_property(&cube_ptr, "location");
    cube_rotation_mode_prop = RNA_struct_find_property(&cube_ptr, "rotation_mode");

    suzanne_ptr = RNA_pointer_create(&suzanne->id, &RNA_Object, &suzanne->id);
    suzanne_location_prop = RNA_struct_find_property(&suzanne_ptr, "location");
    suzanne_rotation_mode_prop = RNA_struct_find_property(&suzanne_ptr, "rotation_mode");

    AnimData *adt_cube = BKE_animdata_ensure_id(&cube->id);
    AnimData *adt_suzanne = BKE_animdata_ensure_id(&suzanne->id);

    /* Set up drivers. */
    ReportList tmp_report_list;
    ANIM_add_driver(&tmp_report_list, &cube->id, "location", 0, 0, DRIVER_TYPE_PYTHON);
    ANIM_add_driver(&tmp_report_list, &suzanne->id, "location", 0, 0, DRIVER_TYPE_PYTHON);
    ANIM_add_driver(&tmp_report_list, &suzanne->id, "location", 2, 0, DRIVER_TYPE_PYTHON);
    ANIM_add_driver(&tmp_report_list, &suzanne->id, "rotation_mode", 0, 0, DRIVER_TYPE_PYTHON);
    FCurve *cube_loc_0_driver = static_cast<FCurve *>(BLI_findlink(&adt_cube->drivers, 0));
    FCurve *suzanne_loc_0_driver = static_cast<FCurve *>(BLI_findlink(&adt_suzanne->drivers, 0));
    FCurve *suzanne_loc_2_driver = static_cast<FCurve *>(BLI_findlink(&adt_suzanne->drivers, 1));
    FCurve *suzanne_rotation_mode_driver = static_cast<FCurve *>(
        BLI_findlink(&adt_suzanne->drivers, 2));
    BLI_strncpy(cube_loc_0_driver->driver->expression, "1.0", 256);
    BLI_strncpy(suzanne_loc_0_driver->driver->expression, "2.0", 256);
    BLI_strncpy(suzanne_loc_2_driver->driver->expression, "3.0", 256);
    BLI_strncpy(suzanne_rotation_mode_driver->driver->expression, "4", 256);

    /* Add animation for Cube's Z location. */
    PointerRNA cube_ptr = RNA_pointer_create(&cube->id, &RNA_Object, &cube->id);
    bAction *act = animrig::id_action_ensure(bmain, &cube->id);
    FCurve *fcu = animrig::action_fcurve_ensure(
        bmain, act, "Object Transforms", &cube_ptr, "location", 2);
    animrig::KeyframeSettings keyframe_settings = {BEZT_KEYTYPE_KEYFRAME, HD_AUTO, BEZT_IPO_BEZ};
    insert_vert_fcurve(fcu, {1.0, 1.0}, keyframe_settings, INSERTKEY_NOFLAGS);
  }

  void TearDown() override
  {
    BKE_main_free(bmain);
  }
};

TEST_F(CopyDriversToSelected, get_property_drivers)
{
  /* Cube location: get all drivers. */
  {
    bool is_array_prop;
    blender::Vector<FCurve *> drivers = blender::interface::internal::get_property_drivers(
        &cube_ptr, cube_location_prop, true, -1, &is_array_prop);

    EXPECT_EQ(is_array_prop, true);
    EXPECT_EQ(drivers.size(), 3);

    EXPECT_TRUE(drivers[0] != nullptr);
    EXPECT_EQ(strcmp(drivers[0]->driver->expression, "1.0"), 0);
    EXPECT_EQ(drivers[1], nullptr);
    EXPECT_EQ(drivers[2], nullptr);
  }

  /* Cube location: get first element driver. */
  {
    bool is_array_prop;
    blender::Vector<FCurve *> drivers = blender::interface::internal::get_property_drivers(
        &cube_ptr, cube_location_prop, false, 0, &is_array_prop);

    EXPECT_EQ(is_array_prop, true);
    EXPECT_EQ(drivers.size(), 3);

    EXPECT_TRUE(drivers[0] != nullptr);
    EXPECT_EQ(strcmp(drivers[0]->driver->expression, "1.0"), 0);
    EXPECT_EQ(drivers[1], nullptr);
    EXPECT_EQ(drivers[2], nullptr);
  }

  /* Cube location: get second element driver. */
  {
    bool is_array_prop;
    blender::Vector<FCurve *> drivers = blender::interface::internal::get_property_drivers(
        &cube_ptr, cube_location_prop, false, 1, &is_array_prop);

    EXPECT_EQ(drivers.size(), 0);
  }

  /* Cube rotation mode: get driver. */
  {
    bool is_array_prop;
    blender::Vector<FCurve *> drivers = blender::interface::internal::get_property_drivers(
        &cube_ptr, cube_rotation_mode_prop, false, 0, &is_array_prop);

    EXPECT_EQ(drivers.size(), 0);
  }

  /* Suzanne location: get all drivers. */
  {
    bool is_array_prop;
    blender::Vector<FCurve *> drivers = blender::interface::internal::get_property_drivers(
        &suzanne_ptr, suzanne_location_prop, true, -1, &is_array_prop);

    EXPECT_EQ(is_array_prop, true);
    EXPECT_EQ(drivers.size(), 3);

    EXPECT_TRUE(drivers[0] != nullptr);
    EXPECT_EQ(strcmp(drivers[0]->driver->expression, "2.0"), 0);
    EXPECT_EQ(drivers[1], nullptr);
    EXPECT_TRUE(drivers[2] != nullptr);
    EXPECT_EQ(strcmp(drivers[2]->driver->expression, "3.0"), 0);
  }

  /* Suzanne location: get first element driver. */
  {
    bool is_array_prop;
    blender::Vector<FCurve *> drivers = blender::interface::internal::get_property_drivers(
        &suzanne_ptr, suzanne_location_prop, false, 0, &is_array_prop);

    EXPECT_EQ(is_array_prop, true);
    EXPECT_EQ(drivers.size(), 3);

    EXPECT_TRUE(drivers[0] != nullptr);
    EXPECT_EQ(strcmp(drivers[0]->driver->expression, "2.0"), 0);
    EXPECT_EQ(drivers[1], nullptr);
    EXPECT_EQ(drivers[2], nullptr);
  }

  /* Suzanne location: get second element driver. */
  {
    bool is_array_prop;
    blender::Vector<FCurve *> drivers = blender::interface::internal::get_property_drivers(
        &suzanne_ptr, suzanne_location_prop, false, 1, &is_array_prop);

    EXPECT_EQ(drivers.size(), 0);
  }

  /* Suzanne rotation mode: get driver. */
  {
    bool is_array_prop;
    blender::Vector<FCurve *> drivers = blender::interface::internal::get_property_drivers(
        &suzanne_ptr, suzanne_rotation_mode_prop, false, 0, &is_array_prop);

    EXPECT_EQ(is_array_prop, false);
    EXPECT_EQ(drivers.size(), 1);
    EXPECT_EQ(strcmp(drivers[0]->driver->expression, "4"), 0);
  }
}

TEST_F(CopyDriversToSelected, paste_property_drivers)
{
  /* Copy all location drivers from Suzanne to Cube. Since neither Suzanne nor
   * Cube have a driver on Y, there should still shouldn't be one on Cube after
   * this. And since Cube has animation on Z, Z shouldn't end up with a driver.
   * Only the X driver should get pasted on Cube, replacing the one that's
   * there. */
  {
    bool is_array_prop;
    blender::Vector<FCurve *> suzanne_location_drivers =
        blender::interface::internal::get_property_drivers(
            &suzanne_ptr, suzanne_location_prop, true, -1, &is_array_prop);

    blender::interface::internal::paste_property_drivers(
        suzanne_location_drivers.as_span(), is_array_prop, &cube_ptr, cube_location_prop);

    blender::Vector<FCurve *> cube_location_drivers =
        blender::interface::internal::get_property_drivers(
            &cube_ptr, cube_location_prop, true, -1, &is_array_prop);

    EXPECT_TRUE(cube_location_drivers[0] != nullptr);
    EXPECT_EQ(strcmp(cube_location_drivers[0]->driver->expression, "2.0"), 0);
    EXPECT_EQ(cube_location_drivers[1], nullptr);
    EXPECT_EQ(cube_location_drivers[2], nullptr);
  }

  /* Copy the rotation_mode driver from Suzanne to Cube. */
  {
    bool is_array_prop;
    blender::Vector<FCurve *> suzanne_rotation_mode_driver =
        blender::interface::internal::get_property_drivers(
            &suzanne_ptr, suzanne_rotation_mode_prop, false, 0, &is_array_prop);

    blender::interface::internal::paste_property_drivers(
        suzanne_rotation_mode_driver.as_span(), is_array_prop, &cube_ptr, cube_rotation_mode_prop);

    blender::Vector<FCurve *> cube_rotation_mode_drivers =
        blender::interface::internal::get_property_drivers(
            &cube_ptr, cube_rotation_mode_prop, false, 0, &is_array_prop);

    EXPECT_TRUE(cube_rotation_mode_drivers[0] != nullptr);
    EXPECT_EQ(strcmp(cube_rotation_mode_drivers[0]->driver->expression, "4"), 0);
  }
}

}  // namespace blender::interface::tests
