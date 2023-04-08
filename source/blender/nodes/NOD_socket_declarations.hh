/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "NOD_node_declaration.hh"

#include "RNA_types.h"

#include "BLI_color.hh"
#include "BLI_function_ref.hh"
#include "BLI_math_vector_types.hh"

using BlenderObjectPtr = Object *;

namespace blender::nodes::decl {

class FloatBuilder;

class Float : public SocketDeclaration {
 public:
  float default_value = 0.0f;
  float soft_min_value = -FLT_MAX;
  float soft_max_value = FLT_MAX;
  PropertySubType subtype = PROP_NONE;

  friend FloatBuilder;

  using Builder = FloatBuilder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bNodeSocket &update_or_build(bNodeTree &ntree, bNode &node, bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class FloatBuilder : public SocketDeclarationBuilder<Float> {
 public:
  FloatBuilder &min(float value);
  FloatBuilder &max(float value);
  FloatBuilder &default_value(float value);
  FloatBuilder &subtype(PropertySubType subtype);
};

class IntBuilder;

class Int : public SocketDeclaration {
 public:
  int default_value = 0;
  int soft_min_value = INT32_MIN;
  int soft_max_value = INT32_MAX;
  PropertySubType subtype = PROP_NONE;

  friend IntBuilder;

  using Builder = IntBuilder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bNodeSocket &update_or_build(bNodeTree &ntree, bNode &node, bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class IntBuilder : public SocketDeclarationBuilder<Int> {
 public:
  IntBuilder &min(int value);
  IntBuilder &max(int value);
  IntBuilder &default_value(int value);
  IntBuilder &subtype(PropertySubType subtype);
};

class VectorBuilder;

class Vector : public SocketDeclaration {
 public:
  float3 default_value = {0, 0, 0};
  float soft_min_value = -FLT_MAX;
  float soft_max_value = FLT_MAX;
  PropertySubType subtype = PROP_NONE;

  friend VectorBuilder;

  using Builder = VectorBuilder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bNodeSocket &update_or_build(bNodeTree &ntree, bNode &node, bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class VectorBuilder : public SocketDeclarationBuilder<Vector> {
 public:
  VectorBuilder &default_value(const float3 value);
  VectorBuilder &subtype(PropertySubType subtype);
  VectorBuilder &min(float min);
  VectorBuilder &max(float max);
  VectorBuilder &compact();
};

class BoolBuilder;

class Bool : public SocketDeclaration {
 public:
  bool default_value = false;
  friend BoolBuilder;

  using Builder = BoolBuilder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bNodeSocket &update_or_build(bNodeTree &ntree, bNode &node, bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class BoolBuilder : public SocketDeclarationBuilder<Bool> {
 public:
  BoolBuilder &default_value(bool value);
};

class ColorBuilder;

class Color : public SocketDeclaration {
 public:
  ColorGeometry4f default_value;

  friend ColorBuilder;

  using Builder = ColorBuilder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bNodeSocket &update_or_build(bNodeTree &ntree, bNode &node, bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class ColorBuilder : public SocketDeclarationBuilder<Color> {
 public:
  ColorBuilder &default_value(const ColorGeometry4f value);
};

class StringBuilder;

class String : public SocketDeclaration {
 public:
  std::string default_value;

  friend StringBuilder;

  using Builder = StringBuilder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bNodeSocket &update_or_build(bNodeTree &ntree, bNode &node, bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class StringBuilder : public SocketDeclarationBuilder<String> {
 public:
  StringBuilder &default_value(const std::string value);
};

class IDSocketDeclaration : public SocketDeclaration {
 public:
  const char *idname;

 public:
  IDSocketDeclaration(const char *idname);

  virtual void construct_default_value(bNode &node, bNodeSocket &socket) const = 0;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bNodeSocket &update_or_build(bNodeTree &ntree, bNode &node, bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class ObjectBuilder;

class Object : public IDSocketDeclaration {
 public:
  std::function<BlenderObjectPtr(const bNode *node)> object_cb;

  void construct_default_value(bNode &node, bNodeSocket &socket) const
  {
    printf("1!\n");
    if (!this->object_cb) {
      return;
    }
    printf("2!\n");
    printf("%p, %p, %p;\n", &socket, this->object_cb(&node), nullptr);
    printf("3!\n");
    socket.default_value_typed<bNodeSocketValueObject>()->value = this->object_cb(&node);
    printf("4!\n");
  }

  friend ObjectBuilder;

 public:
  using Builder = ObjectBuilder;

  Object();
};

class ObjectBuilder : public SocketDeclarationBuilder<Object> {
 public:
  ObjectBuilder &default_value_cb(std::function<BlenderObjectPtr(const bNode *node)> object_cb);
};

class Material : public IDSocketDeclaration {
 public:
  using Builder = SocketDeclarationBuilder<Material>;
  void construct_default_value(bNode & /*node*/, bNodeSocket & /*socket*/) const
  {
    return;
  }

  Material();
};

class Collection : public IDSocketDeclaration {
 public:
  using Builder = SocketDeclarationBuilder<Collection>;
  void construct_default_value(bNode & /*node*/, bNodeSocket & /*socket*/) const
  {
    return;
  }

  Collection();
};

class Texture : public IDSocketDeclaration {
 public:
  using Builder = SocketDeclarationBuilder<Texture>;
  void construct_default_value(bNode & /*node*/, bNodeSocket & /*socket*/) const
  {
    return;
  }

  Texture();
};

class Image : public IDSocketDeclaration {
 public:
  using Builder = SocketDeclarationBuilder<Image>;
  void construct_default_value(bNode & /*node*/, bNodeSocket & /*socket*/) const
  {
    return;
  }
  Image();
};

class ShaderBuilder;

class Shader : public SocketDeclaration {
 public:
  friend ShaderBuilder;

  using Builder = ShaderBuilder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class ShaderBuilder : public SocketDeclarationBuilder<Shader> {
};

class ExtendBuilder;

class Extend : public SocketDeclaration {
 private:
  friend ExtendBuilder;

 public:
  using Builder = ExtendBuilder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bNodeSocket &update_or_build(bNodeTree &ntree, bNode &node, bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class ExtendBuilder : public SocketDeclarationBuilder<Extend> {
};

class Custom : public SocketDeclaration {
 public:
  const char *idname_;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bNodeSocket &update_or_build(bNodeTree &ntree, bNode &node, bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

/* -------------------------------------------------------------------- */
/** \name #FloatBuilder Inline Methods
 * \{ */

inline FloatBuilder &FloatBuilder::min(const float value)
{
  decl_->soft_min_value = value;
  return *this;
}

inline FloatBuilder &FloatBuilder::max(const float value)
{
  decl_->soft_max_value = value;
  return *this;
}

inline FloatBuilder &FloatBuilder::default_value(const float value)
{
  decl_->default_value = value;
  return *this;
}

inline FloatBuilder &FloatBuilder::subtype(PropertySubType subtype)
{
  decl_->subtype = subtype;
  return *this;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #IntBuilder Inline Methods
 * \{ */

inline IntBuilder &IntBuilder::min(const int value)
{
  decl_->soft_min_value = value;
  return *this;
}

inline IntBuilder &IntBuilder::max(const int value)
{
  decl_->soft_max_value = value;
  return *this;
}

inline IntBuilder &IntBuilder::default_value(const int value)
{
  decl_->default_value = value;
  return *this;
}

inline IntBuilder &IntBuilder::subtype(PropertySubType subtype)
{
  decl_->subtype = subtype;
  return *this;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #VectorBuilder Inline Methods
 * \{ */

inline VectorBuilder &VectorBuilder::default_value(const float3 value)
{
  decl_->default_value = value;
  return *this;
}

inline VectorBuilder &VectorBuilder::subtype(PropertySubType subtype)
{
  decl_->subtype = subtype;
  return *this;
}

inline VectorBuilder &VectorBuilder::min(const float min)
{
  decl_->soft_min_value = min;
  return *this;
}

inline VectorBuilder &VectorBuilder::max(const float max)
{
  decl_->soft_max_value = max;
  return *this;
}

inline VectorBuilder &VectorBuilder::compact()
{
  decl_->compact = true;
  return *this;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #BoolBuilder Inline Methods
 * \{ */

inline BoolBuilder &BoolBuilder::default_value(const bool value)
{
  decl_->default_value = value;
  return *this;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #ColorBuilder Inline Methods
 * \{ */

inline ColorBuilder &ColorBuilder::default_value(const ColorGeometry4f value)
{
  decl_->default_value = value;
  return *this;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #ColorBuilder Inline Methods
 * \{ */

inline ObjectBuilder &ObjectBuilder::default_value_cb(
    std::function<BlenderObjectPtr(const bNode *node)> object_cb)
{
  decl_->object_cb = std::move(object_cb);
  return *this;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #StringBuilder Inline Methods
 * \{ */

inline StringBuilder &StringBuilder::default_value(std::string value)
{
  decl_->default_value = std::move(value);
  return *this;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #IDSocketDeclaration and Children Inline Methods
 * \{ */

inline IDSocketDeclaration::IDSocketDeclaration(const char *idname) : idname(idname) {}

inline Object::Object() : IDSocketDeclaration("NodeSocketObject") {}

inline Material::Material() : IDSocketDeclaration("NodeSocketMaterial") {}

inline Collection::Collection() : IDSocketDeclaration("NodeSocketCollection") {}

inline Texture::Texture() : IDSocketDeclaration("NodeSocketTexture") {}

inline Image::Image() : IDSocketDeclaration("NodeSocketImage") {}

/** \} */

}  // namespace blender::nodes::decl
