/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2020 Blender Foundation. */
#include "testing/testing.h"

#include "MEM_guardedalloc.h"

#include "BKE_bvh.hh"
#include "BKE_bvhutils.h"
#include "BKE_idtype.h"
#include "BKE_lib_id.h"
#include "BKE_mesh.h"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"

#include "BLI_math_base.hh"
#include "BLI_math_rotation.hh"
#include "BLI_rand.hh"
#include "BLI_string_ref.hh"
#include "BLI_utildefines.h"

#include "PIL_time_utildefines.h"

#define DO_PERF_TESTS 1

#if DO_PERF_TESTS
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

class UniformAreaTrianglesTest : public BVHPerfTest, public testing::WithParamInterface<int> {
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

  void SetUp() override
  {
  }

  void TearDown() override
  {
  }

  Mesh *create_mesh() const
  {
    const int num_tris = GetParam();
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

TEST_P(UniformAreaTrianglesTest, raycast_uniform_area_triangles)
{
  Mesh *mesh = create_mesh();

  test_raycasts(*mesh);

  BKE_id_free(nullptr, mesh);
}

INSTANTIATE_TEST_SUITE_P(
    MeshSizeTests,
    UniformAreaTrianglesTest,
    testing::Values(
        100000, 200000, 300000, 400000, 500000, 1000000, 2000000, 3000000, 4000000, 5000000));

}  // namespace blender::bke::tests
#endif
