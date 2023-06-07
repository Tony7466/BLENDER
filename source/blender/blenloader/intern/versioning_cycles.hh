/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup blenloader
 */

#include "DNA_node_types.h"

#include "BKE_idprop.h"
#include "BKE_node.h"

static bool socket_is_used(bNodeSocket *sock)
{
  return sock->flag & SOCK_IS_LINKED;
}

static float *cycles_node_socket_float_value(bNodeSocket *socket)
{
  bNodeSocketValueFloat *socket_data = socket->default_value;
  return &socket_data->value;
}

static float *cycles_node_socket_rgba_value(bNodeSocket *socket)
{
  bNodeSocketValueRGBA *socket_data = socket->default_value;
  return socket_data->value;
}

static float *cycles_node_socket_vector_value(bNodeSocket *socket)
{
  bNodeSocketValueVector *socket_data = socket->default_value;
  return socket_data->value;
}

static IDProperty *cycles_properties_from_ID(ID *id)
{
  IDProperty *idprop = IDP_GetProperties(id, false);
  return (idprop) ? IDP_GetPropertyTypeFromGroup(idprop, "cycles", IDP_GROUP) : NULL;
}

static IDProperty *cycles_properties_from_view_layer(ViewLayer *view_layer)
{
  IDProperty *idprop = view_layer->id_properties;
  return (idprop) ? IDP_GetPropertyTypeFromGroup(idprop, "cycles", IDP_GROUP) : NULL;
}

static float cycles_property_float(IDProperty *idprop, const char *name, float default_value)
{
  IDProperty *prop = IDP_GetPropertyTypeFromGroup(idprop, name, IDP_FLOAT);
  return (prop) ? IDP_Float(prop) : default_value;
}

static int cycles_property_int(IDProperty *idprop, const char *name, int default_value)
{
  IDProperty *prop = IDP_GetPropertyTypeFromGroup(idprop, name, IDP_INT);
  return (prop) ? IDP_Int(prop) : default_value;
}

static void cycles_property_int_set(IDProperty *idprop, const char *name, int value)
{
  IDProperty *prop = IDP_GetPropertyTypeFromGroup(idprop, name, IDP_INT);
  if (prop) {
    IDP_Int(prop) = value;
  }
  else {
    IDPropertyTemplate val = {0};
    val.i = value;
    IDP_AddToGroup(idprop, IDP_New(IDP_INT, &val, name));
  }
}

static bool cycles_property_boolean(IDProperty *idprop, const char *name, bool default_value)
{
  return cycles_property_int(idprop, name, default_value);
}

static void cycles_property_boolean_set(IDProperty *idprop, const char *name, bool value)
{
  cycles_property_int_set(idprop, name, value);
}

static IDProperty *cycles_visibility_properties_from_ID(ID *id)
{
  IDProperty *idprop = IDP_GetProperties(id, false);
  return (idprop) ? IDP_GetPropertyTypeFromGroup(idprop, "cycles_visibility", IDP_GROUP) : NULL;
}
