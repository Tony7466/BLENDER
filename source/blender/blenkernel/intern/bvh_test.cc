/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2020 Blender Foundation. */
#include "testing/testing.h"

#include "MEM_guardedalloc.h"

#include "BKE_appdir.h"
#include "BKE_blender.h"
#include "BKE_bvh.hh"
#include "BKE_bvhutils.h"
#include "BKE_callbacks.h"
#include "BKE_global.h"
#include "BKE_idtype.h"
#include "BKE_lib_id.h"
#include "BKE_main.h"
#include "BKE_main_namemap.h"
#include "BKE_mesh.h"
#include "BKE_modifier.h"
#include "BKE_node.h"

#include "DEG_depsgraph.h"
#include "DNA_genfile.h" /* for DNA_sdna_current_init() */
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_windowmanager_types.h"

#include "BLI_fileops.h"
#include "BLI_math_base.hh"
#include "BLI_math_rotation.hh"
#include "BLI_path_util.h"
#include "BLI_rand.hh"
#include "BLI_string_ref.hh"
#include "BLI_utildefines.h"

#include "IMB_imbuf.h"

#include "PIL_time_utildefines.h"

#include "BLO_writefile.h"

#include "RNA_define.h"

#include "GHOST_Path-api.h"

#include "CLG_log.h"

#define DO_PERF_TESTS 0
#define WRITE_DEBUG_MESH_FILES 1

namespace blender::bke::tests {

/* Utility class that records a duration when going out of scope. */
struct ScopedTimer {
 private:
  std::string name_;
  double start_time_;

 public:
  ScopedTimer(StringRef name) : name_(name), start_time_(PIL_check_seconds_timer())
  {
  }

  ~ScopedTimer()
  {
    ::testing::Test::RecordProperty(name_, std::to_string(elapsed()));
  }

  double elapsed() const
  {
    return PIL_check_seconds_timer() - start_time_;
  }
};

class BVHPerfTest : public testing::Test {
 public:
  unsigned int num_rays_ = 1000000;
  unsigned int raycast_seed_ = 121212;

  static void SetUpTestSuite()
  {
    BKE_idtype_init();
    // CLG_init();
    // BKE_callback_global_init();
    // RNA_init();
    // BKE_node_system_init();
  }

  static void TearDownTestSuite()
  {
    // RNA_exit();
    // BKE_node_system_exit();
    // BKE_callback_global_finalize();
    // CLG_exit();
  }

  void init_ray(RandomNumberGenerator &rng,
                float3 &r_origin,
                float3 &r_direction,
                float &r_length)
  {
    r_origin = float3(rng.get_float(), rng.get_float(), rng.get_float()) * 200.0f - 100.0f;
    r_direction = rng.get_unit_float3();
    r_length = rng.get_float() * 200.0f;
  }

  void test_raycasts_classic(const Mesh &mesh)
  {
    BVHTreeFromMesh tree_data;

    {
      ScopedTimer timer("[Classic] BVH Build");
      BKE_bvhtree_from_mesh_get(&tree_data, &mesh, BVHTREE_FROM_LOOPTRI, 4);
    }
    BLI_SCOPED_DEFER([&]() { free_bvhtree_from_mesh(&tree_data); });

    if (tree_data.tree == nullptr) {
      return;
    }

    {
      ScopedTimer timer("[Classic] Raycast");
      RandomNumberGenerator rng(raycast_seed_);
      int hits = 0;
      for (int i = 0; i < num_rays_; ++i) {
        float3 ray_origin;
        float3 ray_direction;
        float ray_length;
        init_ray(rng, ray_origin, ray_direction, ray_length);

        BVHTreeRayHit hit;
        hit.index = -1;
        hit.dist = ray_length;
        bool is_hit = (BLI_bvhtree_ray_cast(tree_data.tree,
                                            ray_origin,
                                            ray_direction,
                                            0.0f,
                                            &hit,
                                            tree_data.raycast_callback,
                                            &tree_data) != -1);
        if (is_hit) {
          ++hits;
        }
      }
      RecordProperty("[Classic] Hits", hits);
    }
  }

  void test_raycasts_embree(const Mesh &mesh)
  {
    bvh::BVHTree tree;

    {
      ScopedTimer timer("[Embree] BVH Build");
      tree.build_single_mesh(mesh);
    }

    {
      ScopedTimer timer("[Embree] Raycast 1");
      RandomNumberGenerator rng(raycast_seed_);
      int hits = 0;
      for (int i = 0; i < num_rays_; ++i) {
        bvh::BVHRay ray;
        init_ray(rng, ray.origin, ray.direction, ray.dist_max);

        bvh::BVHRayHit hit;
        bool is_hit = tree.ray_intersect1(ray, hit);
        if (is_hit) {
          ++hits;
        }
      }
      RecordProperty("[Embree] Hits", hits);
    }
  }

  void test_raycasts(const Mesh &mesh)
  {
    test_raycasts_classic(mesh);
    test_raycasts_embree(mesh);
  }
};

class BVHPerfParamTest : public BVHPerfTest, public testing::WithParamInterface<int> {};

class BVHBlendFileTest : public testing::Test {
 private:
  std::string temp_library_path_;

 public:
  static void SetUpTestSuite()
  {
    /* Minimal code to make loading a blendfile and constructing a depsgraph not crash, copied from
     * main() in creator.c. */
    CLG_init();
    BLI_threadapi_init();

    DNA_sdna_current_init();
    BKE_blender_globals_init();

    BKE_idtype_init();
    BKE_appdir_init();
    IMB_init();
    BKE_modifier_init();
    DEG_register_node_types();
    RNA_init();
    BKE_node_system_init();
    BKE_callback_global_init();
    //BKE_vfont_builtin_register(datatoc_bfont_pfb, datatoc_bfont_pfb_size);

    G.background = true;
    G.factory_startup = true;

    /* Allocate a dummy window manager. The real window manager will try and load Python scripts
     * from the release directory, which it won't be able to find. */
    ASSERT_EQ(G.main->wm.first, nullptr);
    G.main->wm.first = MEM_callocN(sizeof(wmWindowManager), __func__);
  }

  static void TearDownTestSuite()
  {
    if (G.main->wm.first != nullptr) {
      MEM_freeN(G.main->wm.first);
      G.main->wm.first = nullptr;
    }

    /* Copied from WM_exit_ex() in wm_init_exit.cc, and cherry-picked those lines that match the
     * allocation/initialization done in SetUpTestCase(). */
    BKE_blender_free();
    RNA_exit();

    DEG_free_node_types();
    GHOST_DisposeSystemPaths();
    DNA_sdna_current_free();
    BLI_threadapi_exit();

    BKE_blender_atexit();

    BKE_tempdir_session_purge();
    BKE_appdir_exit();
    CLG_exit();
  }

  void SetUp() override
  {
    temp_library_path_ = "";
  }

  void TearDown() override
  {
    if (!temp_library_path_.empty()) {
      BLI_delete(temp_library_path_.c_str(), true, true);
      temp_library_path_ = "";
    }
  }

  /* Register a temporary path, which will be removed at the end of the test.
   * The returned path ends in a slash. */
  std::string use_temp_path()
  {
    BKE_tempdir_init("");
    const std::string tempdir = BKE_tempdir_session();
    temp_library_path_ = tempdir + "test-temporary-path" + SEP_STR;
    return temp_library_path_;
  }

  std::string create_temp_path()
  {
    std::string path = use_temp_path();
    BLI_dir_create_recursive(path.c_str());
    return path;
  }

  /* Register a persistent temporary path. The returned path ends in a slash. */
  std::string use_persistent_temp_path()
  {
    BKE_tempdir_init("");
    const std::string tempdir = BKE_tempdir_base();
    return tempdir + "test-persistent-path" + SEP_STR;
  }

  std::string create_persistent_temp_path()
  {
    std::string path = use_persistent_temp_path();
    BLI_dir_create_recursive(path.c_str());
    return path;
  }

  /* Warning: takes ownership of the mesh and frees it at the end! */
  void write_debug_mesh_file(Mesh *mesh)
  {
    Main *bmain = BKE_main_new();

    Mesh *main_mesh = BKE_mesh_add(bmain, "BVHTest");
    BKE_mesh_nomain_to_mesh(mesh, main_mesh, nullptr);

    char filename[FILE_MAX];
    BLI_strncpy(filename, create_persistent_temp_path().c_str(), sizeof(filename));
    BLI_path_append(filename, sizeof(filename), "bvh_test.blend");

    BlendFileWriteParams write_params{};
    write_params.use_save_versions = 0;
    BLO_write_file(bmain, filename, 0, &write_params, nullptr);
    std::cout << "BVH mesh file written to " << filename << std::endl;

    BKE_main_free(bmain);
  }
};

class UniformAreaTrianglesGenerator {
 public:
  unsigned int mesh_seed_ = 12345;
  float avg_side_length = 0.2f;
  float triangle_area() const
  {
    return 0.5f * avg_side_length * avg_side_length;
  }
  /* Note: don't make this too large, or the rng filter below
   * might reject a large proportion of candidate triangles and take a lot of time.
   */
  float min_angle_ = DEG2RAD(20.0f);
  float min_side_length = 0.333f;

  Mesh *create_mesh(int param) const
  {
    const int num_tris = param;
    const int num_verts = num_tris * 3;
    const int num_loops = num_tris * 3;
    Mesh *mesh = BKE_mesh_new_nomain(num_verts, 0, 0, num_loops, num_tris);
    MutableSpan<float3> positions = mesh->vert_positions_for_write();
    MutableSpan<MPoly> polys = mesh->polys_for_write();
    MutableSpan<MLoop> loops = mesh->loops_for_write();

    RandomNumberGenerator rng(mesh_seed_);
    for (int i = 0; i < num_tris; ++i) {
      float3 p0 = float3(rng.get_float(), rng.get_float(), rng.get_float()) * 10.0f - float3(5.0f);
      float3 u, v;
      float dot_uv;
      do {
        u = rng.get_unit_float3();
        v = rng.get_unit_float3();
        dot_uv = math::dot(u, v);
      } while (math::abs(dot_uv) >= math::cos(min_angle_));
      const float side_scale = math::sqrt(2.0f * triangle_area() / math::sqrt(1.0f - dot_uv * dot_uv));
      const float r = min_side_length + (1.0f - min_side_length) * rng.get_float();
      const float s = 1.0f / r;
      float3 p1 = side_scale * r * u;
      float3 p2 = side_scale * s * v;

      positions[i * 3 + 0] = p0;
      positions[i * 3 + 1] = p1;
      positions[i * 3 + 2] = p2;

      polys[i].loopstart = i * 3;
      polys[i].totloop = 3;

      loops[i * 3 + 0].v = i * 3 + 0;
      loops[i * 3 + 1].v = i * 3 + 1;
      loops[i * 3 + 2].v = i * 3 + 2;
    }
    BKE_mesh_calc_edges(mesh, false, false);

    return mesh;
  }
};

//class DonutCloudTest {
// public:
//  unsigned int mesh_seed_ = 12345;
//  float avg_side_length = 0.2f;
//  float triangle_area() const
//  {
//    return 0.5f * avg_side_length * avg_side_length;
//  }
//  /* Note: don't make this too large, or the rng filter below
//   * might reject a large proportion of candidate triangles and take a lot of time.
//   */
//  float min_angle_ = DEG2RAD(20.0f);
//  float min_side_length = 0.333f;
//
//  Mesh *create_mesh(int param) const
//  {
//    const int num_tris = param;
//    const int num_verts = num_tris * 3;
//    const int num_loops = num_tris * 3;
//    Mesh *mesh = BKE_mesh_new_nomain(num_verts, 0, 0, num_loops, num_tris);
//    MutableSpan<float3> positions = mesh->vert_positions_for_write();
//    MutableSpan<MPoly> polys = mesh->polys_for_write();
//    MutableSpan<MLoop> loops = mesh->loops_for_write();
//
//    RandomNumberGenerator rng(mesh_seed_);
//    for (int i = 0; i < num_tris; ++i) {
//      float3 p0 = float3(rng.get_float(), rng.get_float(), rng.get_float()) * 10.0f - float3(5.0f);
//      float3 u, v;
//      float dot_uv;
//      do {
//        u = rng.get_unit_float3();
//        v = rng.get_unit_float3();
//        dot_uv = math::dot(u, v);
//      } while (math::abs(dot_uv) >= math::cos(min_angle_));
//      const float side_scale = math::sqrt(2.0f * triangle_area() /
//                                          math::sqrt(1.0f - dot_uv * dot_uv));
//      const float r = min_side_length + (1.0f - min_side_length) * rng.get_float();
//      const float s = 1.0f / r;
//      float3 p1 = side_scale * r * u;
//      float3 p2 = side_scale * s * v;
//
//      positions[i * 3 + 0] = p0;
//      positions[i * 3 + 1] = p1;
//      positions[i * 3 + 2] = p2;
//
//      polys[i].loopstart = i * 3;
//      polys[i].totloop = 3;
//
//      loops[i * 3 + 0].v = i * 3 + 0;
//      loops[i * 3 + 1].v = i * 3 + 1;
//      loops[i * 3 + 2].v = i * 3 + 2;
//    }
//    BKE_mesh_calc_edges(mesh, false, false);
//
//    return mesh;
//  }
//};

#if DO_PERF_TESTS
TEST_P(BVHPerfParamTest, raycast_uniform_area_triangles)
{
  UniformAreaTrianglesGenerator gen;
  Mesh *mesh = gen.create_mesh(GetParam());

  test_raycasts(*mesh);

  BKE_id_free(nullptr, mesh);
}

INSTANTIATE_TEST_SUITE_P(
    MeshSizeTests,
    BVHPerfParamTest,
    testing::Values(
        100000, 200000, 300000, 400000, 500000, 1000000, 2000000, 3000000, 4000000, 5000000));
#endif

#if WRITE_DEBUG_MESH_FILES
TEST_F(BVHBlendFileTest, write_mesh_debug_files)
{
  {
    UniformAreaTrianglesGenerator gen;
    Mesh *mesh = gen.create_mesh(10000);
    write_debug_mesh_file(mesh);
  }
}
#endif

}  // namespace blender::bke::tests
