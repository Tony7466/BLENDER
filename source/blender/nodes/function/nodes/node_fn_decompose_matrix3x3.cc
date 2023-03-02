/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"

#include "BLI_math_matrix.h"
#include "BLI_math_rotation.h"

namespace blender::nodes::node_fn_decompose_matrix3x3_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Matrix3x3>(N_("Matrix"));
  b.add_output<decl::Vector>(N_("Rotation"));
  b.add_output<decl::Vector>(N_("Scale"));
};

class DecomposeMatrix3x3Function : public mf::MultiFunction {
 public:
  DecomposeMatrix3x3Function()
  {
    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Decompose Matrix 3x3", signature};
      builder.single_input<float3x3>("Matrix");
      builder.single_output<float3>("Rotation");
      builder.single_output<float3>("Scale");
      return signature;
    }();
    this->set_signature(&signature);
  }

  void call(IndexMask mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float3x3> &matrices = params.readonly_single_input<float3x3>(0, "Matrix");
    MutableSpan<float3> rotations = params.uninitialized_single_output<float3>(0, "Rotation");
    MutableSpan<float3> scales = params.uninitialized_single_output<float3>(1, "Scale");

    for (int64_t i : mask) {
      const float3x3 &mat = matrices[i];
      mat3_to_eul(rotations[i], mat.ptr());
      mat3_to_size(scales[i], mat.ptr());
    }
  }
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static DecomposeMatrix3x3Function decompose_matrix_fn;
  builder.set_matching_fn(&decompose_matrix_fn);
}

}  // namespace blender::nodes::node_fn_decompose_matrix3x3_cc

void register_node_type_fn_decompose_matrix_3x3(void)
{
  namespace file_ns = blender::nodes::node_fn_decompose_matrix3x3_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_DECOMPOSE_MATRIX_3X3, "Decompose 3x3 Matrix", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.build_multi_function = file_ns::node_build_multi_function;

  nodeRegisterType(&ntype);
}
