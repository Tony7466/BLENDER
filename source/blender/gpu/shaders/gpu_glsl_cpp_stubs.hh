/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <type_traits>

template<typename T, int Sz> struct VecOp {
  T operator[](int) {}

  VecOp operator+() const {}
  VecOp operator-() const {}

  VecOp operator+(VecOp b) const {}
  VecOp operator-(VecOp b) const {}
  VecOp operator/(VecOp b) const {}
  VecOp operator*(VecOp b) const {}

  VecOp operator+=(VecOp b) const {}
  VecOp operator-=(VecOp b) const {}
  VecOp operator/=(VecOp b) const {}
  VecOp operator*=(VecOp b) const {}

  VecOp operator+(T b) const {}
  VecOp operator-(T b) const {}
  VecOp operator/(T b) const {}
  VecOp operator*(T b) const {}

  VecOp operator+=(T b) const {}
  VecOp operator-=(T b) const {}
  VecOp operator/=(T b) const {}
  VecOp operator*=(T b) const {}

  friend VecOp operator+(T a, VecOp b) {}
  friend VecOp operator-(T a, VecOp b) {}
  friend VecOp operator/(T a, VecOp b) {}
  friend VecOp operator*(T a, VecOp b) {}

#define INT_OP \
  template<typename U = _T, typename std::enable_if_t<std::is_integral_v<U>> * = nullptr>

  INT_OP VecOp operator%(VecOp b) {}
  INT_OP VecOp operator&(VecOp b) {}
  INT_OP VecOp operator|(VecOp b) {}
  INT_OP VecOp operator^(VecOp b) {}

  INT_OP VecOp operator%=(VecOp b) {}
  INT_OP VecOp operator&=(VecOp b) {}
  INT_OP VecOp operator|=(VecOp b) {}
  INT_OP VecOp operator^=(VecOp b) {}

  INT_OP VecOp operator%(T b) {}
  INT_OP VecOp operator&(T b) {}
  INT_OP VecOp operator|(T b) {}
  INT_OP VecOp operator^(T b) {}

  INT_OP VecOp operator%=(T b) {}
  INT_OP VecOp operator&=(T b) {}
  INT_OP VecOp operator|=(T b) {}
  INT_OP VecOp operator^=(T b) {}

  INT_OP friend VecOp operator%(T a, VecOp<T, Sz> b) {}
  INT_OP friend VecOp operator&(T a, VecOp<T, Sz> b) {}
  INT_OP friend VecOp operator|(T a, VecOp<T, Sz> b) {}
  INT_OP friend VecOp operator^(T a, VecOp<T, Sz> b) {}

#undef INT_OP
};

template<typename T, int Sz> struct VecBase {};

template<typename T> struct VecSwizzle2 {
  static VecBase<T, 2> xx, xy, yx, yy;
  static VecBase<T, 3> xxx, xxy, xyx, xyy, yxx, yxy, yyx, yyy;
  static VecBase<T, 4> xxxx, xxxy, xxyx, xxyy, xyxx, xyxy, xyyx, xyyy, yxxx, yxxy, yxyx, yxyy,
      yyxx, yyxy, yyyx, yyyy;
};

template<typename T> struct VecSwizzle3 : VecSwizzle2<T> {
  static VecBase<T, 2> xz, yz, zx, zy, zz, zw;
  static VecBase<T, 3> xxz, xyz, xzx, xzy, xzz, yxz, yyz, yzx, yzy, yzz, zxx, zxy, zxz, zyx, zyy,
      zyz, zzx, zzy, zzz;
  static VecBase<T, 4> xxxz, xxyz, xxzx, xxzy, xxzz, xyxz, xyyz, xyzx, xyzy, xyzz, xzxx, xzxy,
      xzxz, xzyx, xzyy, xzyz, xzzx, xzzy, xzzz, yxxz, yxyz, yxzx, yxzy, yxzz, yyxz, yyyz, yyzx,
      yyzy, yyzz, yzxx, yzxy, yzxz, yzyx, yzyy, yzyz, yzzx, yzzy, yzzz, zxxx, zxxy, zxxz, zxyx,
      zxyy, zxyz, zxzx, zxzy, zxzz, zyxx, zyxy, zyxz, zyyx, zyyy, zyyz, zyzx, zyzy, zyzz, zzxx,
      zzxy, zzxz, zzyx, zzyy, zzyz, zzzx, zzzy, zzzz;
};

template<typename T> struct VecSwizzle4 : VecSwizzle3<T> {
  static VecBase<T, 2> xw, yw, wx, wy, wz, ww;
  static VecBase<T, 3> xxw, xyw, xzw, xwx, xwy, xwz, xww, yxw, yyw, yzw, ywx, ywy, ywz, yww, zxw,
      zyw, zzw, zwx, zwy, zwz, zww, wxx, wxy, wxz, wxw, wyx, wyy, wyz, wyw, wzx, wzy, wzz, wzw,
      wwx, wwy, wwz, www;
  static VecBase<T, 4> xxxw, xxyw, xxzw, xxwx, xxwy, xxwz, xxww, xyxw, xyyw, xyzw, xywx, xywy,
      xywz, xyww, xzxw, xzyw, xzzw, xzwx, xzwy, xzwz, xzww, xwxx, xwxy, xwxz, xwxw, xwyx, xwyy,
      xwyz, xwyw, xwzx, xwzy, xwzz, xwzw, xwwx, xwwy, xwwz, xwww, yxxw, yxyw, yxzw, yxwx, yxwy,
      yxwz, yxww, yyxw, yyyw, yyzw, yywx, yywy, yywz, yyww, yzxw, yzyw, yzzw, yzwx, yzwy, yzwz,
      yzww, ywxx, ywxy, ywxz, ywxw, ywyx, ywyy, ywyz, ywyw, ywzx, ywzy, ywzz, ywzw, ywwx, ywwy,
      ywwz, ywww, zxxw, zxyw, zxzw, zxwx, zxwy, zxwz, zxww, zyxw, zyyw, zyzw, zywx, zywy, zywz,
      zyww, zzxw, zzyw, zzzw, zzwx, zzwy, zzwz, zzww, zwxx, zwxy, zwxz, zwxw, zwyx, zwyy, zwyz,
      zwyw, zwzx, zwzy, zwzz, zwzw, zwwx, zwwy, zwwz, zwww, wxxx, wxxy, wxxz, wxxw, wxyx, wxyy,
      wxyz, wxyw, wxzx, wxzy, wxzz, wxzw, wxwx, wxwy, wxwz, wxww, wyxx, wyxy, wyxz, wyxw, wyyx,
      wyyy, wyyz, wyyw, wyzx, wyzy, wyzz, wyzw, wywx, wywy, wywz, wyww, wzxx, wzxy, wzxz, wzxw,
      wzyx, wzyy, wzyz, wzyw, wzzx, wzzy, wzzz, wzzw, wzwx, wzwy, wzwz, wzww, wwxx, wwxy, wwxz,
      wwxw, wwyx, wwyy, wwyz, wwyw, wwzx, wwzy, wwzz, wwzw, wwwx, wwwy, wwwz, wwww;
};

template<typename T> struct VecBase<T, 1> {
  T x;

  VecBase() = default;
  VecBase(VecOp<T, 1>) {}
  operator T() const {}
};

template<typename T> struct VecBase<T, 2> : VecOp<T, 2>, VecSwizzle2<T> {
  T x, y;

  VecBase() = default;
  VecBase(VecOp<T, 2>) {}
  explicit VecBase(T) {}
  explicit VecBase(T, T) {}
};

template<typename T> struct VecBase<T, 3> : VecOp<T, 3>, VecSwizzle3<T> {
  T x, y, z;

  VecBase() = default;
  VecBase(VecOp<T, 3>) {}
  explicit VecBase(T) {}
  explicit VecBase(T, T, T) {}
  explicit VecBase(VecBase<T, 2>, T) {}
  explicit VecBase(T, VecBase<T, 2>) {}
};

template<typename T> struct VecBase<T, 4> : VecOp<T, 4>, VecSwizzle4<T> {
  T x, y, z, w;

  VecBase() = default;
  VecBase(VecOp<T, 4>) {}
  explicit VecBase(T) {}
  explicit VecBase(T, T, T, T) {}
  explicit VecBase(VecBase<T, 2>, T, T) {}
  explicit VecBase(T, VecBase<T, 2>, T) {}
  explicit VecBase(T, T, VecBase<T, 2>) {}
  explicit VecBase(VecBase<T, 2>, VecBase<T, 2>) {}
  explicit VecBase(VecBase<T, 3>, T) {}
  explicit VecBase(T, VecBase<T, 3>) {}
};

using uint = unsigned int;

using float2 = VecBase<double, 2>;
using float3 = VecBase<double, 3>;
using float4 = VecBase<double, 4>;

using uint2 = VecBase<uint, 2>;
using uint3 = VecBase<uint, 3>;
using uint4 = VecBase<uint, 4>;

using int2 = VecBase<int, 2>;
using int3 = VecBase<int, 3>;
using int4 = VecBase<int, 4>;

using vec2 = float2;
using vec3 = float3;
using vec4 = float4;

#define inout
#define in
#define out
