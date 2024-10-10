/* SPDX-FileCopyrightText: 2024 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#ifndef __OCTREE_H__
#define __OCTREE_H__

#include "util/boundbox.h"
#include "util/task.h"

CCL_NAMESPACE_BEGIN

class BoundBox;
class Object;
class Progress;
class Scene;
struct KernelOctreeNode;

struct OctreeNode {
  BoundBox bbox;
  vector<Object *> objects;
  int level;

  /* Set the maximall resolution to be 128 to reduce traversing overhead. */
  /* TODO(weizhen): tweak this threshold. 128 is a reference from PBRT. */
  static const int max_level = 7;

  /* TODO(weizhen): we need visibility for shadow, camera, and indirect. */
  float sigma_min = 0.0f;
  float sigma_max = 0.0f;

  OctreeNode() : bbox(BoundBox::empty), level(0) {}
  OctreeNode(BoundBox bbox_, int level_) : bbox(bbox_), level(level_) {}
  virtual ~OctreeNode() = default;

  bool should_split();
  /* TODO(weizhen): this is only for testing. Need to support procedural shaders. */
  float volume_density_scale(const Object *object);
};

struct OctreeInternalNode : public OctreeNode {
  OctreeInternalNode(OctreeNode &node) : children_(8)
  {
    bbox = node.bbox;
    level = node.level;
    objects = std::move(node.objects);
  }

  vector<std::shared_ptr<OctreeNode>> children_;
};

class Octree {
 public:
  void build(Progress &progress);
  void visualize();
  Octree(const Scene *scene);
  ~Octree();

  void flatten(KernelOctreeNode *knodes);
  bool is_empty();
  int get_num_nodes();

 private:
  std::shared_ptr<OctreeInternalNode> make_internal(std::shared_ptr<OctreeNode> &node);
  void recursive_build_(std::shared_ptr<OctreeNode> &node);
  int flatten_(KernelOctreeNode *knodes, std::shared_ptr<OctreeNode> &node, int &index);
  /* TODO(weizhen): this is only for testing. Need to support multiple shaders. */
  uint get_object_shader(const Object *object);

  /* Root node. */
  std::shared_ptr<OctreeNode> root_;
  std::atomic<int> num_nodes = 1;

  TaskPool task_pool;
};

CCL_NAMESPACE_END

#endif /* __OCTREE_H__ */
