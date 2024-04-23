/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_string_utf8.h"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_character_at_index_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::String>("String");
  b.add_input<decl::Int>("Index");
  b.add_output<decl::String>("Character");
  b.add_output<decl::Int>("Unicode");
}

class CharacterAtIndexFunction : public mf::MultiFunction {
 public:
  CharacterAtIndexFunction()
  {
    static mf::Signature signature_;
    mf::SignatureBuilder builder{"Character at Index", signature_};
    builder.single_input<std::string>("String");
    builder.single_input<int>("Index");
    builder.single_output<std::string>("Character");
    builder.single_output<int>("Unicode");
    this->set_signature(&signature_);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<std::string> strings = params.readonly_single_input<std::string>(0, "String");
    const VArray<int> indices = params.readonly_single_input<int>(1, "Index");
    MutableSpan<std::string> characters = params.uninitialized_single_output<std::string>(
        2, "Character");
    MutableSpan<int> unicodes = params.uninitialized_single_output<int>(3, "Unicode");

    Vector<char32_t> char_codes;
    mask.foreach_index([&](const int64_t i) {
      const std::string string = strings[i];
      const int index = indices[i];

      size_t len_bytes;
      const size_t len_chars = BLI_strlen_utf8_ex(string.c_str(), &len_bytes);

      char_codes.resize(len_chars + 1);
      BLI_str_utf8_as_utf32(char_codes.data(), string.c_str(), char_codes.size());

      if (!char_codes.index_range().contains(index)) {
        unicodes[i] = -1;
        new (&characters[i]) std::string();
        return;
      }

      char ch_utf8[BLI_UTF8_MAX + 1];
      const size_t ch_utf8_len = BLI_str_utf8_from_unicode(
          char_codes[index], ch_utf8, sizeof(ch_utf8) - 1);

      unicodes[i] = char_codes[index];
      new (&characters[i]) std::string(ch_utf8, ch_utf8_len);
    });
  }
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static CharacterAtIndexFunction slice_fn;
  builder.set_matching_fn(&slice_fn);
}

static void node_register()
{
  static bNodeType ntype;
  fn_node_type_base(
      &ntype, FN_NODE_CHARACTER_AT_INDEX, "Character at Index", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_character_at_index_cc
