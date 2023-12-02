/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2021 NVIDIA Corporation.
 * All rights reserved.
 */

#include "usd_reader_pointinstancer.h"

#include "BKE_node_tree_update.hh"
#include "BKE_modifier.hh"
#include "BKE_pointcloud.h"
#include "BKE_object.hh"
#include "BKE_attribute.hh"
#include "BKE_node_runtime.hh"
#include "BLI_math_quaternion.hh"

#include <iostream>

#include <pxr/usd/usdGeom/pointInstancer.h>

namespace blender::io::usd {

USDPointInstancerReader::USDPointInstancerReader(const pxr::UsdPrim &prim,
                                     const USDImportParams &import_params,
                                     const ImportSettings &settings)
    : USDXformReader(prim, import_params, settings)
{
}

bool USDPointInstancerReader::valid() const
{
  return prim_.IsValid() && prim_.IsA<pxr::UsdGeomPointInstancer>();
}

void USDPointInstancerReader::create_object(Main *bmain, const double /* motionSampleTime */)
{
  void* point_cloud = BKE_pointcloud_add(bmain, name_.c_str());
  this->object_ = BKE_object_add_only_object(bmain, OB_POINTCLOUD, name_.c_str());
  this->object_->data = point_cloud;
}

void USDPointInstancerReader::read_object_data(Main *bmain, const double motionSampleTime) {
    PointCloud *base_point_cloud = static_cast<PointCloud *>(object_->data);

    pxr::UsdGeomPointInstancer point_instancer_prim(prim_);

    if (!point_instancer_prim) {
        return;
    }

    pxr::VtArray<pxr::GfVec3f> positions;
    pxr::VtArray<pxr::GfVec3f> scales;
    pxr::VtArray<pxr::GfQuath> orientations;
    pxr::VtArray<int> proto_indices;
    std::vector<double> time_samples;

    point_instancer_prim.GetPositionsAttr().GetTimeSamples(&time_samples);

    double sample_time = motionSampleTime;

    if (!time_samples.empty()) {
        sample_time = time_samples[0];
    }

    point_instancer_prim.GetPositionsAttr().Get(&positions, sample_time);
    point_instancer_prim.GetScalesAttr().Get(&scales, sample_time);
    point_instancer_prim.GetOrientationsAttr().Get(&orientations, sample_time);
    point_instancer_prim.GetProtoIndicesAttr().Get(&proto_indices, sample_time);

    PointCloud *point_cloud = BKE_pointcloud_new_nomain(positions.size());

    auto positions_span = point_cloud->positions_for_write();

    for (int i = 0; i < positions.size(); i++) {
        positions_span[i] = float3(positions[i][0], positions[i][1], positions[i][2]);
    }

    auto scales_attribute = point_cloud->attributes_for_write().lookup_or_add_for_write_only_span<float3>("scales", ATTR_DOMAIN_POINT);

    for (int i = 0; i < positions.size(); i++) {
        if (i < scales.size()) {
            scales_attribute.span[i] = float3(scales[i][0], scales[i][1], scales[i][2]);
        } else {
            scales_attribute.span[i] = float3(1.0);
        }
    }

    scales_attribute.span.save();

    auto orientations_attribute = point_cloud->attributes_for_write().lookup_or_add_for_write_only_span<math::Quaternion>("orientations", ATTR_DOMAIN_POINT);

    for (int i = 0; i < positions.size(); i++) {
        if (i < orientations.size()) {
            orientations_attribute.span[i] = math::Quaternion(orientations[i].GetImaginary()[0], orientations[i].GetImaginary()[1], orientations[i].GetImaginary()[2], orientations[i].GetReal());
        } else {
            orientations_attribute.span[i] = math::Quaternion();
        }
    }

    orientations_attribute.span.save();

    auto proto_indices_attribute = point_cloud->attributes_for_write().lookup_or_add_for_write_only_span<int>("proto_indices", ATTR_DOMAIN_POINT);

    for (int i = 0; i < std::min(positions.size(), proto_indices.size()); i++) {
        proto_indices_attribute.span[i] = proto_indices[i];
    }

    proto_indices_attribute.span.save();

    BKE_pointcloud_nomain_to_pointcloud(point_cloud, base_point_cloud);

    /*
    ModifierData *md = BKE_modifier_new(eModifierType_Nodes);
    BLI_addtail(&object_->modifiers, md);
    NodesModifierData &nmd = *reinterpret_cast<NodesModifierData *>(md);
    nmd.node_group = ntreeAddTree(NULL, "Instances", "GeometryNodeTree");
    IDProperty *nmd_properties = nmd.settings.properties;

    bNodeTree *ntree = nmd.node_group;

    ntree->tree_interface.add_socket("Geometry",
        "",
        "NodeSocketGeometry",
        NODE_INTERFACE_SOCKET_INPUT | NODE_INTERFACE_SOCKET_OUTPUT,
        nullptr
    );
    bNode *group_input = nodeAddStaticNode(NULL, ntree, NODE_GROUP_INPUT);
    group_input->locx = -400.0f;
    bNode *group_output = nodeAddStaticNode(NULL, ntree, NODE_GROUP_OUTPUT);
    group_output->locx = 500.0f;
    group_output->flag |= NODE_DO_OUTPUT;

    bNode *instance_on_points_node = nodeAddStaticNode(NULL, ntree, GEO_NODE_INSTANCE_ON_POINTS);
    instance_on_points_node->locx = 300.0f;
    bNode *rotation_to_euler_node = nodeAddStaticNode(NULL, ntree, FN_NODE_ROTATION_TO_EULER);
    rotation_to_euler_node->locx = 100.0f;
    rotation_to_euler_node->locy = -100.0f;

    bNode *collection_info_node = nodeAddStaticNode(NULL, ntree, GEO_NODE_COLLECTION_INFO);
    collection_info_node->locx = 100.0f;
    collection_info_node->locy = 100.0f;
    bool seperate_children = true;
    nodeFindSocket(collection_info_node, SOCK_IN, "Separate Children")->default_value = (void*)&seperate_children;

    BKE_ntree_update_main_tree(bmain, ntree, nullptr);

    nodeAddLink(ntree,
                group_input,
                static_cast<bNodeSocket *>(group_input->outputs.first),
                instance_on_points_node,
                nodeFindSocket(instance_on_points_node, SOCK_IN, "Points")
    );

    nodeAddLink(ntree,
                rotation_to_euler_node,
                nodeFindSocket(rotation_to_euler_node, SOCK_OUT, "Euler"),
                instance_on_points_node,
                nodeFindSocket(instance_on_points_node, SOCK_IN, "Rotation"));

    nodeAddLink(ntree,
                collection_info_node,
                nodeFindSocket(collection_info_node, SOCK_OUT, "Instances"),
                instance_on_points_node,
                nodeFindSocket(instance_on_points_node, SOCK_IN, "Instance"));

    nodeAddLink(ntree,
                instance_on_points_node,
                nodeFindSocket(instance_on_points_node, SOCK_OUT, "Instances"),
                group_output,
                static_cast<bNodeSocket *>(group_output->inputs.first));

    BKE_ntree_update_main_tree(bmain, ntree, nullptr);
    */

    USDXformReader::read_object_data(bmain, motionSampleTime);
}

pxr::SdfPathVector USDPointInstancerReader::proto_paths() const {
    pxr::SdfPathVector paths;

    pxr::UsdGeomPointInstancer point_instancer_prim(prim_);

    if (!point_instancer_prim) {
        return paths;
    }

    point_instancer_prim.GetPrototypesRel().GetTargets(&paths);

    return paths;
}


}  // namespace blender::io::usd
