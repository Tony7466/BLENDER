#pragma once

#include "BLI_compiler_compat.h"
#include "BLI_map.hh"
#include "BLI_math.h"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_set.hh"
#include "BLI_utildefines.h"
#include "BLI_vector.hh"

#include "BKE_paint.h"
#include "BKE_pbvh.h"
#include "BKE_pbvh_iter.hh"

#include "sculpt_intern.hh"
#include <functional>

namespace blender::editors::sculpt {
/* Abstract class. */
class BrushTester {
  struct Result {
    float dist_squared;
    bool in_range;
    float calc_falloff() {}
  };

  Result operator()(PBVHVertRef vertex, const float3 &co, const float3 &no) {}
};

class SphereTester {
  struct Result {
    float dist_squared;
    bool in_range;
    float3 co, no;
    PBVHVertRef vertex;

    Result(SphereTester &owner_) : owner(owner_) {}

    float calc_falloff(int thread_id, AutomaskingNodeData *automask_data)
    {
      float mask = SCULPT_vertex_mask_get(owner.ss, vertex);

      SCULPT_brush_strength_factor(owner.ss,
                                   owner.brush,
                                   co,
                                   sqrtf(dist_squared),
                                   no,
                                   no,
                                   mask,
                                   vertex,
                                   thread_id,
                                   &automask_data);
    }

   private:
    SphereTester &owner;
  };

  SculptBrushTest test;
  SculptBrushTestFn sculpt_brush_test_sq_fn;
  SculptSession *ss;
  Brush *brush;

  SphereTester(SculptSession *ss_, Brush *brush_) : ss(ss_), brush(brush_) {}

  Result operator()(PBVHVertRef vertex, const float3 &co, const float3 &no)
  {
    Result res(*this);

    res.dist_squared = len_squared_v3v3(test.location, co);
    res.in_range = res.dist_squared < ss->cache->radius_squared;
    res.co = co;
    res.no = no;
    res.vertex = vertex;

    return res;
  }
};

template<typename Tester = BrushTester, typename NodeData, typename ExecFunc>
void exec_brush_intern(SculptSession *ss,
                       Span<PBVHNode *> nodes,
                       bool threaded,         //
                       Tester &brush_tester,  //
                       std::function<NodeData(PBVHNode *node)> node_visit_pre,
                       ExecFunc &exec, /* [&](auto &vertex_range) {} */
                       std::function<void(PBVHNode *node, NodeData *node_data)> node_visit_post,
                       bool needs_original = false,
                       SculptUndoType original_type = SCULPT_UNDO_COORDS

)
{
  template<typename VertexRange> struct ForwardVertexIter {
    using base_iterator = std::invoke_result_t<VertexRange::begin>::type;

    struct iterator {
      PBVHVertRef vertex;
      int index;

      float falloff;

      float *co;
      const float *no;
      float *mask;

      int vertex_node_index;
      int thread_id;

      iterator(base_iterator vd, base_iterator end_vd) : vd_(vd), end_(end_vd)
      {
        const int thread_id = BLI_task_parallel_thread_id(nullptr);

        if (vd_ != end_) {
          load_data();
          find_valid_item();
        }
      }

      iterator(const iterator &b)
      {
        vd_ = b.vd_;
        end_ = b.end_;
        thread_id = b.thread_id;
      }

      iterator &operator++()
      {
        ++vd_;

        if (vd_ != end_) {
          load_data();
          find_valid_item();
        }

        return *this;
      }

      bool operator==(const iterator &b)
      {
        return b.vd_ == vd_;
      }

      bool operator!=(const iterator &b)
      {
        return b.vd_ != vd_;
      }

     private:
      void load_data()
      {
        //
        vertex = vd_.vertex;
        vertex_node_index = vd_.vertex_node_index;
        index = vd_.index;

        co = vd_.co;
        no = vd_.no;
        mask = vd_.mask;

        falloff = vd_.result.calc_falloff(thread_id, &vd_.node_data.automask_data);
      }

      void find_valid_iter()
      {
        while (vd_ != end_ && falloff == 0.0f) {
          ++vd_;

          if (vd_ != end_) {
            load_data();
          }
        }
      }

      base_iterator vd_, end_;
    }

    ForwardVertexIter(VertexRange &range)
        : range_(range)
    {
    }

    iterator begin()
    {
      return iterator(range_.begin(), range_.end());
    }

    iterator end()
    {
      return iterator(range_.end(), range_.end());
    }

   private:
    VertexRange &range_;
  }

  struct BaseNodeData {
    AutomaskingNodeData automask_data;
    SculptOrigVertData orig_data;
    NodeData user_data;
  };

  blender::bke::pbvh::foreach_brush_verts<BaseNodeData>(  //
      ss->pbvh,
      nodes,
      threaded,
      brush_tester,
      [&](PBVHNode *node) {
        BaseNodeData data = {};

        SCULPT_automasking_node_begin(ob, ss, ss->cache->automasking, &data.automask_data, node);

        if (needs_original) {
          if (data.automask_data.have_orig_data && original_type == SCULPT_UNDO_COORDS) {
            data.orig_data = data.automask_data.orig_data;
          }
          else {
            SCULPT_orig_vert_data_init(&automask_data->orig_data, ob, node, original_type);
          }
        }

        if (node_visit_pre) {
          data.user_data = node_visit_pre(node);
        }

        return data;
      },
      [&](auto &range) { exec(ForwardVertexIter(range)); },
      [&](PBVHNode *node) {
        if (node_visit_post) {
          node_visit_post(node);
        }
      });
}

template<typename NodeData, typename ExecFunc>
void exec_brush(SculptSession *ss,
                Span<PBVHNode *> nodes,
                bool threaded,  //
                std::function<NodeData(PBVHNode *node)> node_visit_pre,
                ExecFunc &exec, /* [&](auto &vertex_range) {} */
                std::function<void(PBVHNode *node, NodeData *node_data)> node_visit_post,
                bool needs_original = false,
                SculptUndoType original_type = SCULPT_UNDO_COORDS

)
{
  SphereTester brush_tester(ss, ss->cache->brush);

  exec_brush_intern<NodeData>(
      ss, nodes, threaded, node_visit_pre, exec, node_visit_post, needs_original, original_type);
}

}  // namespace blender::editors::sculpt
