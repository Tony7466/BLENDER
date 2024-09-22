/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <type_traits>

#define BLI_ENABLE_IF(condition) typename std::enable_if_t<(condition)> * = nullptr
#define BLI_ENABLE_IF_VEC(_size, _test) int S = _size, BLI_ENABLE_IF((S _test))

template<typename T, int Size, bool at_least_vec3 = Size >= 3, bool at_least_vec4 = Size >= 4>
struct swizzle {
  T x;
  T y;
  std::enable_if_t<at_least_vec3, T> z;
  std::enable_if_t<at_least_vec4, T> w;
};

template<typename T, int Size, bool at_least_vec3 = Size >= 3, bool at_least_vec4 = Size >= 4>
struct VecBase : swizzle<T, Size> {

  using swizzleT_vec2 = swizzle<T, 2>;
  using swizzleT_vec3 = swizzle<T, 3>;
  using swizzleT_vec4 = swizzle<T, 4>;

  using swizzle2_vec3 = std::enable_if_t<at_least_vec3, swizzleT_vec2>;
  using swizzle3_vec3 = std::enable_if_t<at_least_vec3, swizzleT_vec3>;
  using swizzle4_vec3 = std::enable_if_t<at_least_vec3, swizzleT_vec4>;

  using swizzle2_vec4 = std::enable_if_t<at_least_vec4, swizzleT_vec2>;
  using swizzle3_vec4 = std::enable_if_t<at_least_vec4, swizzleT_vec3>;
  using swizzle4_vec4 = std::enable_if_t<at_least_vec4, swizzleT_vec4>;

  union {
    swizzleT_vec2 xx, xy, yx, yy;
    swizzleT_vec3 xxx, xxy, xyx, xyy, yxx, yxy, yyx, yyy;
    swizzleT_vec4 xxxx, xxxy, xxyx, xxyy, xyxx, xyxy, xyyx, xyyy, yxxx, yxxy, yxyx, yxyy, yyxx,
        yyxy, yyyx, yyyy;

    swizzle2_vec3 xz, yz, zx, zy, zz, zw;
    swizzle3_vec3 xxz, xyz, xzx, xzy, xzz, yxz, yyz, yzx, yzy, yzz, zxx, zxy, zxz, zyx, zyy, zyz,
        zzx, zzy, zzz;
    swizzle4_vec3 xxxz, xxyz, xxzx, xxzy, xxzz, xyxz, xyyz, xyzx, xyzy, xyzz, xzxx, xzxy, xzxz,
        xzyx, xzyy, xzyz, xzzx, xzzy, xzzz, yxxz, yxyz, yxzx, yxzy, yxzz, yyxz, yyyz, yyzx, yyzy,
        yyzz, yzxx, yzxy, yzxz, yzyx, yzyy, yzyz, yzzx, yzzy, yzzz, zxxx, zxxy, zxxz, zxyx, zxyy,
        zxyz, zxzx, zxzy, zxzz, zyxx, zyxy, zyxz, zyyx, zyyy, zyyz, zyzx, zyzy, zyzz, zzxx, zzxy,
        zzxz, zzyx, zzyy, zzyz, zzzx, zzzy, zzzz;

    swizzle2_vec4 xw, yw, wx, wy, wz, ww;
    swizzle3_vec4 xxw, xyw, xzw, xwx, xwy, xwz, xww, yxw, yyw, yzw, ywx, ywy, ywz, yww, zxw, zyw,
        zzw, zwx, zwy, zwz, zww, wxx, wxy, wxz, wxw, wyx, wyy, wyz, wyw, wzx, wzy, wzz, wzw, wwx,
        wwy, wwz, www;
    swizzle4_vec4 xxxw, xxyw, xxzw, xxwx, xxwy, xxwz, xxww, xyxw, xyyw, xyzw, xywx, xywy, xywz,
        xyww, xzxw, xzyw, xzzw, xzwx, xzwy, xzwz, xzww, xwxx, xwxy, xwxz, xwxw, xwyx, xwyy, xwyz,
        xwyw, xwzx, xwzy, xwzz, xwzw, xwwx, xwwy, xwwz, xwww, yxxw, yxyw, yxzw, yxwx, yxwy, yxwz,
        yxww, yyxw, yyyw, yyzw, yywx, yywy, yywz, yyww, yzxw, yzyw, yzzw, yzwx, yzwy, yzwz, yzww,
        ywxx, ywxy, ywxz, ywxw, ywyx, ywyy, ywyz, ywyw, ywzx, ywzy, ywzz, ywzw, ywwx, ywwy, ywwz,
        ywww, zxxw, zxyw, zxzw, zxwx, zxwy, zxwz, zxww, zyxw, zyyw, zyzw, zywx, zywy, zywz, zyww,
        zzxw, zzyw, zzzw, zzwx, zzwy, zzwz, zzww, zwxx, zwxy, zwxz, zwxw, zwyx, zwyy, zwyz, zwyw,
        zwzx, zwzy, zwzz, zwzw, zwwx, zwwy, zwwz, zwww, wxxx, wxxy, wxxz, wxxw, wxyx, wxyy, wxyz,
        wxyw, wxzx, wxzy, wxzz, wxzw, wxwx, wxwy, wxwz, wxww, wyxx, wyxy, wyxz, wyxw, wyyx, wyyy,
        wyyz, wyyw, wyzx, wyzy, wyzz, wyzw, wywx, wywy, wywz, wyww, wzxx, wzxy, wzxz, wzxw, wzyx,
        wzyy, wzyz, wzyw, wzzx, wzzy, wzzz, wzzw, wzwx, wzwy, wzwz, wzww, wwxx, wwxy, wwxz, wwxw,
        wwyx, wwyy, wwyz, wwyw, wwzx, wwzy, wwzz, wwzw, wwwx, wwwy, wwwz, wwww;
  };

  VecBase() {}
  VecBase(const VecBase &) {}

  VecBase(T) {}
  VecBase(const swizzle<T, Size> &) {}

  template<BLI_ENABLE_IF_VEC(Size, == 2)> VecBase(T, T) {}
  template<BLI_ENABLE_IF_VEC(Size, == 3)> VecBase(T, T, T) {}
  template<BLI_ENABLE_IF_VEC(Size, == 4)> VecBase(T, T, T, T) {}

  template<BLI_ENABLE_IF_VEC(Size, == 3)> VecBase(swizzle<T, 2>, T) {}
  template<BLI_ENABLE_IF_VEC(Size, == 3)> VecBase(T, swizzle<T, 2>) {}

  template<BLI_ENABLE_IF_VEC(Size, == 4)> VecBase(swizzle<T, 2>, T, T) {}
  template<BLI_ENABLE_IF_VEC(Size, == 4)> VecBase(T, swizzle<T, 2>, T) {}
  template<BLI_ENABLE_IF_VEC(Size, == 4)> VecBase(T, T, swizzle<T, 2>) {}
  template<BLI_ENABLE_IF_VEC(Size, == 4)> VecBase(swizzle<T, 2>, swizzle<T, 2>) {}
  template<BLI_ENABLE_IF_VEC(Size, == 4)> VecBase(swizzle<T, 3>, T) {}
  template<BLI_ENABLE_IF_VEC(Size, == 4)> VecBase(T, swizzle<T, 3>) {}

  T operator[](int) {}
};

template<typename T, int Sz> VecBase<T, Sz> operator+(swizzle<T, Sz> a, swizzle<T, Sz> b) {}
template<typename T, int Sz> VecBase<T, Sz> operator-(swizzle<T, Sz> a, swizzle<T, Sz> b) {}
template<typename T, int Sz> VecBase<T, Sz> operator/(swizzle<T, Sz> a, swizzle<T, Sz> b) {}
template<typename T, int Sz> VecBase<T, Sz> operator*(swizzle<T, Sz> a, swizzle<T, Sz> b) {}

template<typename T, int Sz> VecBase<T, Sz> operator+(swizzle<T, Sz> a, T b) {}
template<typename T, int Sz> VecBase<T, Sz> operator-(swizzle<T, Sz> a, T b) {}
template<typename T, int Sz> VecBase<T, Sz> operator/(swizzle<T, Sz> a, T b) {}
template<typename T, int Sz> VecBase<T, Sz> operator*(swizzle<T, Sz> a, T b) {}

template<typename T, int Sz> VecBase<T, Sz> operator+(T a, swizzle<T, Sz> b) {}
template<typename T, int Sz> VecBase<T, Sz> operator-(T a, swizzle<T, Sz> b) {}
template<typename T, int Sz> VecBase<T, Sz> operator/(T a, swizzle<T, Sz> b) {}
template<typename T, int Sz> VecBase<T, Sz> operator*(T a, swizzle<T, Sz> b) {}

/* Needed to support double literals. */
template<int Sz> VecBase<float, Sz> operator+(swizzle<float, Sz> a, double b) {}
template<int Sz> VecBase<float, Sz> operator-(swizzle<float, Sz> a, double b) {}
template<int Sz> VecBase<float, Sz> operator/(swizzle<float, Sz> a, double b) {}
template<int Sz> VecBase<float, Sz> operator*(swizzle<float, Sz> a, double b) {}

template<int Sz> VecBase<float, Sz> operator+(double a, swizzle<float, Sz> b) {}
template<int Sz> VecBase<float, Sz> operator-(double a, swizzle<float, Sz> b) {}
template<int Sz> VecBase<float, Sz> operator/(double a, swizzle<float, Sz> b) {}
template<int Sz> VecBase<float, Sz> operator*(double a, swizzle<float, Sz> b) {}

#define BLI_INT_OP template<typename T, int Sz, BLI_ENABLE_IF((std::is_integral_v<T>))>
BLI_INT_OP VecBase<T, Sz> operator%(swizzle<T, Sz> a, swizzle<T, Sz> b) {}
BLI_INT_OP VecBase<T, Sz> operator&(swizzle<T, Sz> a, swizzle<T, Sz> b) {}
BLI_INT_OP VecBase<T, Sz> operator|(swizzle<T, Sz> a, swizzle<T, Sz> b) {}
BLI_INT_OP VecBase<T, Sz> operator^(swizzle<T, Sz> a, swizzle<T, Sz> b) {}

BLI_INT_OP VecBase<T, Sz> operator%(swizzle<T, Sz> a, T b) {}
BLI_INT_OP VecBase<T, Sz> operator&(swizzle<T, Sz> a, T b) {}
BLI_INT_OP VecBase<T, Sz> operator|(swizzle<T, Sz> a, T b) {}
BLI_INT_OP VecBase<T, Sz> operator^(swizzle<T, Sz> a, T b) {}

BLI_INT_OP VecBase<T, Sz> operator%(T a, swizzle<T, Sz> b) {}
BLI_INT_OP VecBase<T, Sz> operator&(T a, swizzle<T, Sz> b) {}
BLI_INT_OP VecBase<T, Sz> operator|(T a, swizzle<T, Sz> b) {}
BLI_INT_OP VecBase<T, Sz> operator^(T a, swizzle<T, Sz> b) {}

using uint = unsigned int;

using float2 = VecBase<float, 2>;
using float3 = VecBase<float, 3>;
using float4 = VecBase<float, 4>;

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
