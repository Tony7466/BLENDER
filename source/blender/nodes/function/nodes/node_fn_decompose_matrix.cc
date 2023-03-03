/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"

#include "BLI_math_matrix.h"
#include "BLI_math_rotation.h"
#include "BLI_math_vector.h"

namespace blender::nodes::node_fn_decompose_matrix_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Matrix>(N_("Matrix"));
  b.add_output<decl::Vector>(N_("Translation"));
  b.add_output<decl::Vector>(N_("Rotation"));
  b.add_output<decl::Vector>(N_("Scale"));
};

class DecomposeMatrixFunction : public mf::MultiFunction {
 public:
  DecomposeMatrixFunction()
  {
    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Separate Matrix 3x3", signature};
      builder.single_input<float4x4>("Matrix");
      builder.single_output<float3>("Translation");
      builder.single_output<float3>("Rotation");
      builder.single_output<float3>("Scale");
      return signature;
    }();
    this->set_signature(&signature);
  }

  void call(IndexMask mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float4x4> &matrices = params.readonly_single_input<float4x4>(0, "Matrix");
    MutableSpan<float3> translations = params.uninitialized_single_output<float3>(1, "Translation");
    MutableSpan<float3> rotations = params.uninitialized_single_output<float3>(2, "Rotation");
    MutableSpan<float3> scales = params.uninitialized_single_output<float3>(3, "Scale");

    for (int64_t i : mask) {
      const float4x4 &mat = matrices[i];
      copy_v3_v3(translations[i], mat[3]);
      mat4_to_eul(rotations[i], mat.ptr());
      mat4_to_size(scales[i], mat.ptr());
    }
  }
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static DecomposeMatrixFunction decompose_matrix_fn;
  builder.set_matching_fn(&decompose_matrix_fn);
}

}  // namespace blender::nodes::node_fn_decompose_matrix_cc

void register_node_type_fn_decompose_matrix(void)
{
  namespace file_ns = blender::nodes::node_fn_decompose_matrix_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_DECOMPOSE_MATRIX, "Decompose Matrix", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.build_multi_function = file_ns::node_build_multi_function;

  nodeRegisterType(&ntype);
}
