//
// Created by Charles Wardlaw on 2024-02-29.
//

#include "usd_reader_utils.hh"

#include <pxr/usd/usd/attribute.h>

#include <iostream>

#include "BLI_utildefines.h"


namespace {
template<typename VECT>
void set_array_prop(IDProperty *idgroup,
                    const char *prop_name,
                    const pxr::UsdAttribute &attr,
                    const pxr::UsdTimeCode motionSampleTime)
{
  if (!idgroup || !attr) {
    return;
  }

  VECT vec;
  if (!attr.Get<VECT>(&vec, motionSampleTime)) {
    return;
  }

  IDPropertyTemplate val = {0};
  val.array.len = static_cast<int>(vec.dimension);

  if (val.array.len <= 0) {
    /* Should never happen. */
    std::cout << "Invalid array length for prop " << prop_name << std::endl;
    return;
  }

  if (std::is_same<float, typename VECT::ScalarType>()) {
    val.array.type = IDP_FLOAT;
  }
  else if (std::is_same<double, typename VECT::ScalarType>()) {
    val.array.type = IDP_DOUBLE;
  }
  else if (std::is_same<int, typename VECT::ScalarType>()) {
    val.array.type = IDP_INT;
  }
  else {
    std::cout << "Couldn't determine array type for prop " << prop_name << std::endl;
    return;
  }

  IDProperty *prop = IDP_New(IDP_ARRAY, &val, prop_name);

  if (!prop) {
    std::cout << "Couldn't create array prop " << prop_name << std::endl;
    return;
  }

  typename VECT::ScalarType *prop_data = static_cast<typename VECT::ScalarType *>(
      prop->data.pointer);

  for (int i = 0; i < val.array.len; ++i) {
    prop_data[i] = vec[i];
  }

  IDP_AddToGroup(idgroup, prop);
}
}  // anonymous namespace


namespace blender::io::usd {

/* TfToken objects are not cheap to construct, so we do it once. */
namespace usdtokens {
static const pxr::TfToken userProperties("userProperties", pxr::TfToken::Immortal);
}  // namespace usdtokens

static void set_string_prop(IDProperty *idgroup, const char *prop_name, const char *str_val)
{
  if (!idgroup) {
    return;
  }

  IDPropertyTemplate val = {0};
  val.string.str = str_val;
  /* Note length includes null terminator. */
  val.string.len = strlen(str_val) + 1;
  val.string.subtype = IDP_STRING_SUB_UTF8;

  IDProperty *prop = IDP_New(IDP_STRING, &val, prop_name);

  IDP_AddToGroup(idgroup, prop);
}

static void set_int_prop(IDProperty *idgroup, const char *prop_name, const int ival)
{
  if (!idgroup) {
    return;
  }

  IDPropertyTemplate val = {0};
  val.i = ival;
  IDProperty *prop = IDP_New(IDP_INT, &val, prop_name);

  IDP_AddToGroup(idgroup, prop);
}

static void set_float_prop(IDProperty *idgroup, const char *prop_name, const float fval)
{
  if (!idgroup) {
    return;
  }

  IDPropertyTemplate val = {0};
  val.f = fval;
  IDProperty *prop = IDP_New(IDP_FLOAT, &val, prop_name);

  IDP_AddToGroup(idgroup, prop);
}

static void set_double_prop(IDProperty *idgroup, const char *prop_name, const double dval)
{
  if (!idgroup) {
    return;
  }

  IDPropertyTemplate val = {0};
  val.d = dval;
  IDProperty *prop = IDP_New(IDP_DOUBLE, &val, prop_name);

  IDP_AddToGroup(idgroup, prop);
}


void set_id_props_from_prim(ID *id,
               const pxr::UsdPrim &prim,
               const eUSDAttrImportMode attr_import_mode,
               const std::optional<double> motionSampleTime)
{
  pxr::UsdAttributeVector attribs = prim.GetAuthoredAttributes();
  if (attribs.empty()) {
    return;
  }

  IDProperty *idgroup = IDP_EnsureProperties(id);

  bool all_custom_attrs = (attr_import_mode == USD_ATTR_IMPORT_ALL);

  const pxr::UsdTimeCode time_code(motionSampleTime.has_value() ? motionSampleTime.value() :
                                                                  pxr::UsdTimeCode::Default());

  for (const pxr::UsdAttribute &attr : attribs) {
    if (!attr.IsCustom()) {
      continue;
    }

    std::vector<std::string> attr_names = attr.SplitName();

    bool is_user_prop = attr_names[0] == "userProperties";

    if (attr_names.size() > 2 && is_user_prop && attr_names[1] == "blenderName") {
      /* Skip the deprecated userProperties:blenderName namespace attribs. */
      continue;
    }

    if (!all_custom_attrs && !is_user_prop) {
      continue;
    }

    /* When importing user properties, strip the namespace. */
    pxr::TfToken attr_name;
    if (is_user_prop) {
      /* We strip the userProperties namespace, but leave others in case
       * someone's custom attribute namespace is important in their pipeline. */
      const std::string token = "userProperties:";
      const std::string name = attr.GetName().GetString();
      attr_name = pxr::TfToken(name.substr(token.size(), name.size() - token.size()));
    }
    else {
      attr_name = attr.GetName();
    }

    pxr::SdfValueTypeName type_name = attr.GetTypeName();

    if (type_name == pxr::SdfValueTypeNames->Int) {
      int ival = 0;
      if (attr.Get<int>(&ival, time_code)) {
        set_int_prop(idgroup, attr_name.GetString().c_str(), ival);
      }
    }
    else if (type_name == pxr::SdfValueTypeNames->Float) {
      float fval = 0.0f;
      if (attr.Get<float>(&fval, time_code)) {
        set_float_prop(idgroup, attr_name.GetString().c_str(), fval);
      }
    }
    else if (type_name == pxr::SdfValueTypeNames->Double) {
      double dval = 0.0;
      if (attr.Get<double>(&dval, time_code)) {
        set_double_prop(idgroup, attr_name.GetString().c_str(), dval);
      }
    }
    else if (type_name == pxr::SdfValueTypeNames->String) {
      std::string sval;
      if (attr.Get<std::string>(&sval, time_code)) {
        set_string_prop(idgroup, attr_name.GetString().c_str(), sval.c_str());
      }
    }
    else if (type_name == pxr::SdfValueTypeNames->Token) {
      pxr::TfToken tval;
      if (attr.Get<pxr::TfToken>(&tval, time_code)) {
        set_string_prop(idgroup, attr_name.GetString().c_str(), tval.GetString().c_str());
      }
    }
    else if (type_name == pxr::SdfValueTypeNames->Float2) {
      set_array_prop<pxr::GfVec2f>(idgroup, attr_name.GetString().c_str(), attr, time_code);
    }
    else if (type_name == pxr::SdfValueTypeNames->Float3) {
      set_array_prop<pxr::GfVec3f>(idgroup, attr_name.GetString().c_str(), attr, time_code);
    }
    else if (type_name == pxr::SdfValueTypeNames->Float4) {
      set_array_prop<pxr::GfVec4f>(idgroup, attr_name.GetString().c_str(), attr, time_code);
    }
    else if (type_name == pxr::SdfValueTypeNames->Double2) {
      set_array_prop<pxr::GfVec2d>(idgroup, attr_name.GetString().c_str(), attr, time_code);
    }
    else if (type_name == pxr::SdfValueTypeNames->Double3) {
      set_array_prop<pxr::GfVec3d>(idgroup, attr_name.GetString().c_str(), attr, time_code);
    }
    else if (type_name == pxr::SdfValueTypeNames->Double4) {
      set_array_prop<pxr::GfVec4d>(idgroup, attr_name.GetString().c_str(), attr, time_code);
    }
    else if (type_name == pxr::SdfValueTypeNames->Int2) {
      set_array_prop<pxr::GfVec2i>(idgroup, attr_name.GetString().c_str(), attr, time_code);
    }
    else if (type_name == pxr::SdfValueTypeNames->Int3) {
      set_array_prop<pxr::GfVec3i>(idgroup, attr_name.GetString().c_str(), attr, time_code);
    }
    else if (type_name == pxr::SdfValueTypeNames->Int4) {
      set_array_prop<pxr::GfVec4i>(idgroup, attr_name.GetString().c_str(), attr, time_code);
    }
  }
}

} // end namespace blender:io::usd
