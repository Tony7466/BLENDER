/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_noise.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_position_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Vector>("Position").field_source();
}

static void test_generic(const IndexMask &mask, GMutableSpan src, GMutableSpan dst)
{
  const CPPType &type = src.type();

  type.default_construct_indices(dst.data(), mask);
  type.destruct_indices(dst.data(), mask);
  type.default_construct_indices(src.data(), mask);
  type.copy_assign_indices(src.data(), dst.data(), mask);
  type.copy_assign_compressed(src.data(), dst.data(), mask);
  type.default_construct_indices(dst.data(), mask);
  type.copy_construct_indices(src.data(), dst.data(), mask);
}

template<typename T> static void test(const int64_t size, const float probability)
{
  const CPPType &type = CPPType::get<T>();
  std::cout << type.name() << ";\n";
  const uint64_t hash = type.hash();

  IndexMaskMemory memory;
  const IndexMask mask = IndexMask::from_predicate(
      IndexMask(size), GrainSize(1024), memory, [&](const int index) {
        return noise::hash_to_float(index, hash) <= probability;
      });

  Array<T> src(size);
  Array<T> dst(size);

  test_generic(mask, src.as_mutable_span(), dst.as_mutable_span());
}

static void node_geo_exec(GeoNodeExecParams params)
{
  for ([[maybe_unused]] const int index : IndexRange(10)) {
    // const int64_t size = int64_t(index + 5) * int64_t(index * 10);
    const int64_t size = 10000000;
    std::cout << "| Size: " << size << ";\n";
    // for (const int probability : IndexRange(20)) {
    // const float probability_ = (float(probability) * 5.0f) / 1.0f;
    const float probability_ = 0.99f;
    std::cout << "|  Probability: " << probability_ << ";\n";
    test<bool>(size, probability_);
    test<int>(size, probability_);
    test<int2>(size, probability_);
    test<float>(size, probability_);
    test<float2>(size, probability_);
    test<float3>(size, probability_);
    test<ColorGeometry4f>(size, probability_);
    test<ColorGeometry4b>(size, probability_);
    test<math::Quaternion>(size, probability_);
    test<std::string>(size, probability_);
    std::cout << std::endl;
    //}
  }

  Field<float3> position_field{AttributeFieldInput::Create<float3>("position")};
  params.set_output("Position", std::move(position_field));
}

}  // namespace blender::nodes::node_geo_input_position_cc

void register_node_type_geo_input_position()
{
  namespace file_ns = blender::nodes::node_geo_input_position_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_INPUT_POSITION, "Position", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
