/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_bvh.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#ifdef WITH_BVH_EMBREE

namespace blender {

BVHTree::BVHTree()
{
}

BVHTree::~BVHTree()
{
}

void BVHTree::build_single_mesh(const Mesh &mesh)
{
}

}  // namespace blender

#else /* WITH_BVH_EMBREE */

namespace blender {

BVHTree::BVHTree()
{
}

BVHTree::~BVHTree()
{
}

void BVHTree::build_single_mesh(const Mesh &mesh)
{
}

}  // namespace blender

#endif /* WITH_BVH_EMBREE */
