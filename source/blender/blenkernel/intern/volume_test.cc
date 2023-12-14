/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "DNA_volume_types.h"

#include "BKE_idtype.h"
#include "BKE_lib_id.h"
#include "BKE_main.hh"
#include "BKE_volume.hh"
#include "BKE_volume_grid.hh"

namespace blender {

// namespace {

// struct VolumeTestContext {
//   Main *bmain = nullptr;

//   VolumeTestContext()
//   {
//     BKE_idtype_init();
//     bmain = BKE_main_new();
//   }
//   ~VolumeTestContext()
//   {
//     BKE_main_free(bmain);
//   }
// };

// class VolumeTest : public ::testing::Test {
// };

// }  // namespace

// TEST_F(VolumeTest, volume_grids)
// {
//   /* Empty volume. */
//   {
//     VolumeTestContext context;
//     Volume *volume = static_cast<Volume *>(BKE_id_new(context.bmain, ID_VO, nullptr));
//     EXPECT_EQ(BKE_volume_num_grids(volume), 0);

//     EXPECT_EQ(BKE_volume_grid_get(volume, 0), nullptr);
//     EXPECT_EQ(BKE_volume_grid_get(volume, 1), nullptr);
//     EXPECT_EQ(BKE_volume_grid_get_for_write(volume, 0), nullptr);
//     EXPECT_EQ(BKE_volume_grid_get_for_write(volume, 1), nullptr);

//     Volume *volume_copy = reinterpret_cast<Volume *>(BKE_id_copy(context.bmain, &volume->id));
//     EXPECT_EQ(BKE_volume_num_grids(volume_copy), 0);

//     EXPECT_EQ(BKE_volume_grid_get(volume_copy, 0), nullptr);
//     EXPECT_EQ(BKE_volume_grid_get(volume_copy, 1), nullptr);
//     EXPECT_EQ(BKE_volume_grid_get_for_write(volume_copy, 0), nullptr);
//     EXPECT_EQ(BKE_volume_grid_get_for_write(volume_copy, 1), nullptr);

//     BKE_id_free(context.bmain, volume_copy);
//     BKE_id_free(context.bmain, volume);
//   }

//   /* Empty grid. */
//   {
//     VolumeTestContext context;
//     Volume *volume = static_cast<Volume *>(BKE_id_new(context.bmain, ID_VO, nullptr));

//     BKE_volume_grid_add(volume, "Grid", VOLUME_GRID_FLOAT);
//     EXPECT_EQ(BKE_volume_num_grids(volume), 1);

//     EXPECT_NE(BKE_volume_grid_get(volume, 0), nullptr);
//     EXPECT_NE(BKE_volume_grid_get_for_write(volume, 0), nullptr);

//     const VolumeGrid *grid = BKE_volume_grid_get(volume, 0);
//     EXPECT_TRUE(grid->is_mutable());
//     EXPECT_FALSE(grid->is_expired());

//     BKE_volume_grid_remove(volume, grid);
//     EXPECT_EQ(BKE_volume_num_grids(volume), 0);

//     BKE_id_free(context.bmain, volume);
//   }
// }

// TEST_F(VolumeTest, volume_grid_pointers)
// {
//   /* Grid pointer from volume. */
//   {
//     VolumeTestContext context;
//     Volume *volume = static_cast<Volume *>(BKE_id_new(context.bmain, ID_VO, nullptr));

//     BKE_volume_grid_add(volume, "Grid", VOLUME_GRID_FLOAT);

//     const VolumeGrid *grid = BKE_volume_grid_get(volume, 0);
//     EXPECT_TRUE(grid->is_mutable());
//     EXPECT_FALSE(grid->is_expired());

//     {
//       grid->add_user();
//       EXPECT_FALSE(grid->is_mutable());
//       EXPECT_FALSE(grid->is_expired());
//       bke::GVolumeGridPtr ptr{grid};
//       EXPECT_FALSE(grid->is_mutable());
//       EXPECT_FALSE(grid->is_expired());
//     }
//     EXPECT_TRUE(grid->is_mutable());
//     EXPECT_FALSE(grid->is_expired());

//     BKE_id_free(context.bmain, volume);
//   }

//   /* Shared grid pointer. */
//   {
//     VolumeTestContext context;
//     Volume *volume = static_cast<Volume *>(BKE_id_new(context.bmain, ID_VO, nullptr));

//     BKE_volume_grid_add(volume, "Grid", VOLUME_GRID_FLOAT);

//     const VolumeGrid *grid = BKE_volume_grid_get_for_write(volume, 0);
//     EXPECT_TRUE(grid->is_mutable());
//     EXPECT_FALSE(grid->is_expired());

//     /* Simulate additional user. */
//     grid->add_user();
//     EXPECT_FALSE(grid->is_mutable());
//     EXPECT_FALSE(grid->is_expired());

//     {
//       grid->add_user();
//       EXPECT_FALSE(grid->is_mutable());
//       EXPECT_FALSE(grid->is_expired());
//       bke::GVolumeGridPtr ptr{grid};
//       EXPECT_FALSE(grid->is_mutable());
//       EXPECT_FALSE(grid->is_expired());
//     }
//     EXPECT_FALSE(grid->is_mutable());
//     EXPECT_FALSE(grid->is_expired());

//     /* Mutable grid copy. */
//     VolumeGrid *grid_copy = grid->copy();
//     EXPECT_TRUE(grid_copy->is_mutable());
//     EXPECT_FALSE(grid_copy->is_expired());

//     {
//       bke::GVolumeGridPtr ptr{grid_copy};
//       EXPECT_TRUE(grid_copy->is_mutable());
//       EXPECT_FALSE(grid_copy->is_expired());
//     }
//     grid_copy = nullptr;

//     /* Remove additional user. */
//     grid->remove_user_and_delete_if_last();
//     EXPECT_TRUE(grid->is_mutable());
//     EXPECT_FALSE(grid->is_expired());

//     BKE_id_free(context.bmain, volume);
//   }

//   /* Typed grid pointer upcast. */
//   {
//     VolumeTestContext context;
//     Volume *volume = static_cast<Volume *>(BKE_id_new(context.bmain, ID_VO, nullptr));

//     BKE_volume_grid_add(volume, "Grid", VOLUME_GRID_FLOAT);

//     const VolumeGrid *grid = BKE_volume_grid_get_for_write(volume, 0);
//     EXPECT_TRUE(grid->is_mutable());

//     VolumeGrid *grid_copy = grid->copy();
//     EXPECT_TRUE(grid_copy->is_mutable());

//     {
//       bke::GVolumeGridPtr ptr{grid_copy};
//       EXPECT_TRUE(grid_copy->is_mutable());

//       {
//         /* Cast to typed pointer. */
//         bke::VolumeGridPtr<float> typed_ptr = ptr.typed<float>();
//         EXPECT_FALSE(grid_copy->is_mutable());
//       }
//       EXPECT_TRUE(grid_copy->is_mutable());
//     }
//     grid_copy = nullptr;

//     BKE_id_free(context.bmain, volume);
//   }

//   /* Typed grid pointer downcast. */
//   {
//     VolumeTestContext context;
//     Volume *volume = static_cast<Volume *>(BKE_id_new(context.bmain, ID_VO, nullptr));

//     BKE_volume_grid_add(volume, "Grid", VOLUME_GRID_FLOAT);

//     const VolumeGrid *grid = BKE_volume_grid_get_for_write(volume, 0);
//     EXPECT_TRUE(grid->is_mutable());

//     VolumeGrid *grid_copy = grid->copy();
//     EXPECT_TRUE(grid_copy->is_mutable());

//     {
//       bke::VolumeGridPtr<float> typed_ptr{grid_copy};
//       EXPECT_TRUE(grid_copy->is_mutable());

//       {
//         /* Cast to generic pointer. */
//         bke::GVolumeGridPtr ptr = typed_ptr;
//         EXPECT_FALSE(grid_copy->is_mutable());
//       }
//       EXPECT_TRUE(grid_copy->is_mutable());
//     }
//     grid_copy = nullptr;

//     BKE_id_free(context.bmain, volume);
//   }
// }

}  // namespace blender
