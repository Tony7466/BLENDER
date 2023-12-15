/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_node_socket_value.hh"

#include "BLI_color.hh"
#include "BLI_math_rotation_types.hh"
#include "BLI_math_vector_types.hh"

#include "FN_field.hh"

namespace blender::bke {

template<typename T> T SocketValueVariant::extract_as()
{
  /* TODO */
  return T();
}
template<typename T> T SocketValueVariant::get_as() const
{
  return T();
}
template<typename T> void SocketValueVariant::store_as_impl(T /*value*/)
{
  /* TODO */
}

void SocketValueVariant::store_single(eNodeSocketDatatype socket_type, const void *value)
{
  UNUSED_VARS(socket_type, value);
}

bool SocketValueVariant::is_context_dependent_field() const
{
  if (!value_.is<fn::GField>()) {
    return false;
  }
  const fn::GField &field = value_.get<fn::GField>();
  if (!field) {
    return false;
  }
  return field.node().depends_on_input();
}

void *SocketValueVariant::ensure_uninitialized_single(eNodeSocketDatatype /*socket_type*/)
{
  return nullptr;
}

void *SocketValueVariant::ensure_uninitialized_single(const CPPType & /*cpp_type*/)
{
  return nullptr;
}

void SocketValueVariant::convert_to_single() {}

GPointer SocketValueVariant::get_single() const
{
  return {};
}
GMutablePointer SocketValueVariant::get_single()
{
  return {};
}

#define INSTANTIATE(TYPE) \
  template TYPE SocketValueVariant::extract_as(); \
  template TYPE SocketValueVariant::get_as() const; \
  template void SocketValueVariant::store_as_impl(TYPE);

#define INSTANTIATE_SINGLE_AND_FIELD(TYPE) \
  INSTANTIATE(TYPE) \
  INSTANTIATE(fn::Field<TYPE>)

INSTANTIATE_SINGLE_AND_FIELD(int)
INSTANTIATE_SINGLE_AND_FIELD(bool)
INSTANTIATE_SINGLE_AND_FIELD(float)
INSTANTIATE_SINGLE_AND_FIELD(blender::float3)
INSTANTIATE_SINGLE_AND_FIELD(blender::ColorGeometry4f)
INSTANTIATE_SINGLE_AND_FIELD(blender::math::Quaternion)

INSTANTIATE(std::string)
INSTANTIATE(fn::GField)

}  // namespace blender::bke
