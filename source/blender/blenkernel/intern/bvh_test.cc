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
#include "BKE_mesh.hh"
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

#include "GHOST_Path-api.hh"

#include "CLG_log.h"

#define DO_PERF_TESTS 1
#define WRITE_DEBUG_MESH_FILES 0

namespace blender::bke::tests {

/* Utility class that records a duration when going out of scope. */
struct ScopedTimer {
 private:
  std::string name_;
  double start_time_;

 public:
  ScopedTimer(StringRef name) : name_(name), start_time_(PIL_check_seconds_timer()) {}

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

  void init_ray(RandomNumberGenerator &rng, float3 &r_origin, float3 &r_direction, float &r_length)
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
    // BKE_vfont_builtin_register(datatoc_bfont_pfb, datatoc_bfont_pfb_size);

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
  void write_debug_mesh_file(Mesh *mesh, const char *filename)
  {
    Main *bmain = BKE_main_new();

    Mesh *main_mesh = BKE_mesh_add(bmain, "BVHTest");
    BKE_mesh_nomain_to_mesh(mesh, main_mesh, nullptr);

    char filepath[FILE_MAX];
    BLI_strncpy(filepath, create_persistent_temp_path().c_str(), sizeof(filepath));
    BLI_path_append(filepath, sizeof(filepath), filename);

    BlendFileWriteParams write_params{};
    write_params.use_save_versions = 0;
    BLO_write_file(bmain, filepath, 0, &write_params, nullptr);
    std::cout << "BVH mesh file written to " << filepath << std::endl;

    BKE_main_free(bmain);
  }
};

class MeshGenerator {
 public:
  int num_triangles_ = 100;
  float size_ = 10.0f;
  float scale_ = 1.0f;

  MeshGenerator(int num_triangles) : num_triangles_(num_triangles)
  {
    scale_ = size_ / powf(num_triangles_, 0.333333f);
  }
};

class UniformAreaTrianglesGenerator : public MeshGenerator {
 public:
  unsigned int mesh_seed_ = 12345;
  float triangle_area() const
  {
    const float avg_side_length = 0.5f * scale_;
    return 0.5f * avg_side_length * avg_side_length;
  }
  /* Note: don't make this too large, or the rng filter below
   * might reject a large proportion of candidate triangles and take a lot of time.
   */
  float min_angle_ = DEG2RAD(20.0f);
  float min_side_factor = 0.333f;

  UniformAreaTrianglesGenerator(int num_triangles) : MeshGenerator(num_triangles) {}

  Mesh *create_mesh() const
  {
    const int num_tris = num_triangles_;
    const int num_verts = num_tris * 3;
    const int num_corners = num_tris * 3;
    Mesh *mesh = BKE_mesh_new_nomain(num_verts, 0, num_tris, num_corners);
    MutableSpan<float3> positions = mesh->vert_positions_for_write();
    MutableSpan<int> poly_offsets = mesh->poly_offsets_for_write();
    MutableSpan<int> corner_verts = mesh->corner_verts_for_write();

    RandomNumberGenerator rng(mesh_seed_);
    for (int i = 0; i < num_tris; ++i) {
      float3 p0 = (float3(rng.get_float(), rng.get_float(), rng.get_float()) - float3(0.5f)) *
                  size_;
      float3 u, v;
      float dot_uv;
      do {
        u = rng.get_unit_float3();
        v = rng.get_unit_float3();
        dot_uv = math::dot(u, v);
      } while (math::abs(dot_uv) >= math::cos(min_angle_));
      const float side_scale = math::sqrt(2.0f * triangle_area() /
                                          math::sqrt(1.0f - dot_uv * dot_uv));
      const float r = min_side_factor + (1.0f - min_side_factor) * rng.get_float();
      const float s = 1.0f / r;
      float3 p1 = p0 + side_scale * r * u;
      float3 p2 = p0 + side_scale * s * v;

      positions[i * 3 + 0] = p0;
      positions[i * 3 + 1] = p1;
      positions[i * 3 + 2] = p2;

      poly_offsets[i] = i * 3;

      corner_verts[i * 3 + 0] = i * 3 + 0;
      corner_verts[i * 3 + 1] = i * 3 + 1;
      corner_verts[i * 3 + 2] = i * 3 + 2;
    }
    poly_offsets[num_tris] = num_tris * 3;
    BKE_mesh_calc_edges(mesh, false, false);

    return mesh;
  }
};

class DonutCloudGenerator : MeshGenerator {
 public:
  unsigned int mesh_seed_ = 12345;
  int min_segments_ = 4;
  int max_segments_ = 128;

  DonutCloudGenerator(int num_triangles) : MeshGenerator(num_triangles) {}

  void get_donut_params(RandomNumberGenerator &rng,
                        int &r_major_segments,
                        int &r_minor_segments,
                        float &r_major_radius,
                        float &r_minor_radius,
                        float3 &r_center,
                        math::Quaternion &r_rotation) const
  {
    const float avg_major_radius_ = 4.0f;
    r_major_radius = scale_ * avg_major_radius_ * (0.5f + 1.0f * rng.get_float());
    r_minor_radius = r_major_radius * (0.25f + 0.5f * rng.get_float());
    r_major_segments = min_segments_ + rng.get_int32(max_segments_ - min_segments_ + 1);
    r_minor_segments = math::max((int)(r_major_segments * r_minor_radius / r_major_radius),
                                 min_segments_);
    r_center = (float3(rng.get_float(), rng.get_float(), rng.get_float()) - float3(0.5f)) * size_;
    const float a0 = rng.get_float() * 2.0f * M_PI;
    const float a1 = rng.get_float() * 2.0f * M_PI;
    const float a2 = rng.get_float() * 2.0f * M_PI;
    mul_qt_qtqt((float4 &)r_rotation,
                float4(math::sin(a0), 0.0f, 0.0f, math::cos(a0)),
                float4(0.0f, math::sin(a1), 0.0f, math::cos(a1)));
    mul_qt_qtqt((float4 &)r_rotation,
                (float4)r_rotation,
                float4(0.0f, 0.0f, math::sin(a2), math::cos(a2)));
  }

  void generate_donut(MutableSpan<int> poly_offsets,
                      MutableSpan<int> corner_verts,
                      MutableSpan<float3> positions,
                      int verts_start,
                      int corners_start,
                      int major_segments,
                      int minor_segments,
                      float major_radius,
                      float minor_radius,
                      const float3 &center,
                      const math::Quaternion &rotation) const
  {
    BLI_assert(poly_offsets.size() == major_segments * minor_segments + 1);
    BLI_assert(corner_verts.size() == 4 * major_segments * minor_segments);
    BLI_assert(positions.size() == major_segments * minor_segments);

    const float major_delta = M_PI * 2.0f / (float)major_segments;
    const float minor_delta = M_PI * 2.0f / (float)minor_segments;
    for (const int i : IndexRange(major_segments)) {
      for (const int j : IndexRange(minor_segments)) {
        const int index = i * minor_segments + j;

        poly_offsets[index] = corners_start + index * 4;

        const int vert00 = verts_start + index;
        const int vert10 = j + 1 < minor_segments ? vert00 + 1 : vert00 - j;
        const int vert01 = i + 1 < major_segments ? vert00 + minor_segments :
                                                    vert00 - i * minor_segments;
        const int vert11 = j + 1 < minor_segments ? vert01 + 1 : vert01 - j;
        // BLI_assert(vert00 < poly_start + polys.size());
        // BLI_assert(vert10 < poly_start + polys.size());
        // BLI_assert(vert01 < poly_start + polys.size());
        // BLI_assert(vert11 < poly_start + polys.size());
        corner_verts[index * 4 + 0] = vert00;
        corner_verts[index * 4 + 1] = vert10;
        corner_verts[index * 4 + 2] = vert11;
        corner_verts[index * 4 + 3] = vert01;

        const float major_angle = i * major_delta;
        const float minor_angle = j * minor_delta;
        const float3 minor_pos = float3(math::cos(minor_angle), 0.0f, math::sin(minor_angle)) *
                                     minor_radius +
                                 float3(major_radius, 0.0f, 0.0f);
        const float3 major_pos(minor_pos.x * math::cos(major_angle),
                               minor_pos.x * math::sin(major_angle),
                               minor_pos.z);
        float3 pos = major_pos;
        mul_qt_v3((float4)rotation, pos);
        pos += center;
        positions[index] = pos;
      }
    }
    poly_offsets[major_segments * minor_segments] = corners_start +
                                                    major_segments * minor_segments * 4;
  }

  Mesh *create_mesh() const
  {
    /* Dry run for counting triangles */
    RandomNumberGenerator rng(mesh_seed_);
    int num_donuts = 0;
    int num_polys = 0;
    const int max_polys = num_triangles_ / 2;
    while (true) {
      float r_maj, r_min;
      int seg_maj, seg_min;
      float3 center;
      math::Quaternion rotation;
      get_donut_params(rng, seg_maj, seg_min, r_maj, r_min, center, rotation);
      const int donut_polys = seg_min * seg_maj;

      if (num_polys + donut_polys >= max_polys) {
        break;
      }
      ++num_donuts;
      num_polys += donut_polys;
    }

    /* Torus has same number of verts as polys, 4 loops per poly */
    Mesh *mesh = BKE_mesh_new_nomain(num_polys, 0, num_polys, num_polys * 4);
    MutableSpan<float3> positions = mesh->vert_positions_for_write();
    MutableSpan<int> poly_offsets = mesh->poly_offsets_for_write();
    MutableSpan<int> corner_verts = mesh->corner_verts_for_write();

    /* Reset RNG */
    rng.seed(mesh_seed_);
    int polys_start = 0;
    int corners_start = 0;
    int verts_start = 0;
    for (const int idonut : IndexRange(num_donuts)) {
      float r_maj, r_min;
      int seg_maj, seg_min;
      float3 center;
      math::Quaternion rotation;
      get_donut_params(rng, seg_maj, seg_min, r_maj, r_min, center, rotation);
      const int donut_polys = seg_min * seg_maj;

      generate_donut(poly_offsets.slice(polys_start, donut_polys + 1),
                     corner_verts.slice(corners_start, donut_polys * 4),
                     positions.slice(verts_start, donut_polys),
                     verts_start,
                     corners_start,
                     seg_maj,
                     seg_min,
                     r_maj,
                     r_min,
                     center,
                     rotation);

      polys_start += donut_polys;
      corners_start += donut_polys * 4;
      verts_start += donut_polys;
    }
    BKE_mesh_calc_edges(mesh, false, false);

    // BKE_mesh_validate(mesh, true, true);

    return mesh;
  }
};

#if DO_PERF_TESTS
TEST_P(BVHPerfParamTest, raycast_uniform_area_triangles)
{
  UniformAreaTrianglesGenerator gen(GetParam());
  Mesh *mesh = gen.create_mesh();

  test_raycasts(*mesh);

  BKE_id_free(nullptr, mesh);
}

TEST_P(BVHPerfParamTest, raycast_donut_cloud)
{
  DonutCloudGenerator gen(GetParam());
  Mesh *mesh = gen.create_mesh();

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
    UniformAreaTrianglesGenerator gen(10000);
    Mesh *mesh = gen.create_mesh();
    write_debug_mesh_file(mesh, "triangle_soup.blend");
  }
  {
    DonutCloudGenerator gen(1000000);
    Mesh *mesh = gen.create_mesh();
    write_debug_mesh_file(mesh, "donut_cloud.blend");
  }
}
#endif

}  // namespace blender::bke::tests
