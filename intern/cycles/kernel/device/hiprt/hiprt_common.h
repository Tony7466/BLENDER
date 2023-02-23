#pragma once

#if (defined(__CUDACC__) || defined(__HIPCC__))
#  define __KERNELCC__
#endif

#if !defined(__KERNELCC__)
#  include <algorithm>
#  include <cfloat>
#  include <cmath>
#  include <cstring>
#  include <fstream>
#  include <iostream>
#  include <map>
#  include <string>
#  include <type_traits>
#  include <vector>

#  define __host__
#  define __device__
#endif

#ifdef __CUDACC__
// Switch to sync counterparts as CUDA recently deprecated the non-sync ones
#  define __shfl(x, y) __shfl_sync(__activemask(), (x), (y))
#  define __shfl_up(x, y) __shfl_up_sync(__activemask(), (x), (y))
#  define __ballot(x) __ballot_sync(__activemask(), (x))
#  define __any(x) __any_sync(__activemask(), (x))
#endif

#define HIPRT_ALIGN(N) alignas(N)

#ifdef __KERNELCC__
#  define HIPRT_INLINE __forceinline__
#else
#  define HIPRT_INLINE inline
#endif

#define HIPRT_HOST __host__
#define HIPRT_DEVICE __device__
#define HIPRT_HOST_DEVICE __host__ __device__

#define HIPRT_MIN(a, b) (((b) < (a)) ? (b) : (a))
#define HIPRT_MAX(a, b) (((b) > (a)) ? (b) : (a))

#define HIPRT_LOG2(n) ((n & 2) ? 1 : 0)
#define HIPRT_LOG4(n) ((n & (0xC)) ? (2 + HIPRT_LOG2(n >> 2)) : (HIPRT_LOG2(n)))
#define HIPRT_LOG8(n) ((n & 0xF0) ? (4 + HIPRT_LOG4(n >> 4)) : (HIPRT_LOG4(n)))
#define HIPRT_LOG16(n) ((n & 0xFF00) ? (8 + HIPRT_LOG8(n >> 8)) : (HIPRT_LOG8(n)))
#define HIPRT_LOG32(n) ((n & 0xFFFF0000) ? (16 + HIPRT_LOG16(n >> 16)) : (HIPRT_LOG16(n)))
#define HIPRT_LOG(n) (n == 0 ? 0 : HIPRT_LOG32(n))

typedef unsigned long long u64;
typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

namespace hiprt {
constexpr float Pi = 3.14159265358979323846f;
constexpr float TwoPi = 2.0f * Pi;
constexpr float FltMin = 1.175494351e-38f;
constexpr float FltMax = 3.402823466e+38f;
constexpr int IntMin = -2147483648;
constexpr int IntMax = 2147483647;

constexpr u32 InvalidValue = ~0u;
constexpr u32 FullRayMask = ~0u;

template<class T, class U> struct is_same {
  enum { value = 0 };
};

template<class T> struct is_same<T, T> {
  enum { value = 1 };
};

template<class T> struct remove_reference {
  using type = T;
};

template<class T> struct remove_reference<T &> {
  using type = T;
};

template<class T> struct remove_reference<T &&> {
  using type = T;
};

template<class Ty, Ty Val> struct integral_constant {
  static constexpr Ty value = Val;
  using value_type = Ty;
  using type = integral_constant;

  HIPRT_DEVICE constexpr operator value_type() const noexcept
  {
    return value;
  }
  [[nodiscard]] HIPRT_DEVICE constexpr value_type operator()() const noexcept
  {
    return value;
  }
};

template<bool _Val> using bool_constant = integral_constant<bool, _Val>;

using true_type = bool_constant<true>;
using false_type = bool_constant<false>;

template<class T> struct is_lvalue_reference : false_type {
};

template<class T> struct is_lvalue_reference<T &> : true_type {
};

template<class T> HIPRT_DEVICE constexpr typename remove_reference<T>::type &&move(T &&t) noexcept
{
  return static_cast<typename remove_reference<T>::type &&>(t);
}

template<class T>
HIPRT_DEVICE constexpr T &&forward(typename remove_reference<T>::type &t) noexcept
{
  return static_cast<T &&>(t);
}

template<class T>
HIPRT_DEVICE constexpr T &&forward(typename remove_reference<T>::type &&t) noexcept
{
  static_assert(!is_lvalue_reference<T>::value,
                "template argument T must not be an lvalue reference type");
  return static_cast<T &&>(t);
}

template<class T> struct alignment_of : integral_constant<size_t, alignof(T)> {
};

template<bool B, class T, class F> struct conditional {
  using type = T;
};

template<class T, class F> struct conditional<false, T, F> {
  using type = F;
};

template<size_t Size, u32 Align> struct aligned_storage {
  struct type {
    alignas(Align) unsigned char data[Size];
  };
};

// template <size_t Size>
// struct aligned_storage<Size, -1>
//{
//	struct type
//	{
//		unsigned char data[Size];
//	};
//};

template<typename T> HIPRT_HOST_DEVICE HIPRT_INLINE void swap(T &a, T &b)
{
  T t = a;
  a = b;
  b = t;
}

#if !defined(__KERNELCC__)
template<typename T> struct Traits {
  static const std::string TYPE_NAME;
};

#  define DECLARE_TYPE_TRAITS(name) template<> const std::string Traits<name>::TYPE_NAME = #  name;
#endif
}  // namespace hiprt

template<typename T, size_t Size, u32 Align> class hiprtPimpl {
  typename hiprt::aligned_storage<Size, Align>::type data;

 public:
  template<size_t T_size> HIPRT_DEVICE static constexpr void PIMPL_SIZE_CHECK_()
  {
    static_assert(T_size == Size, "hiprtPimpl: T has a different size than given");
  };

  HIPRT_DEVICE static constexpr void PIMPL_PTR_CHECK_()
  {
    PIMPL_SIZE_CHECK_<sizeof(T)>();
    static_assert(alignof(T) >= hiprt::alignment_of<T>::value,
                  "hiprtPimpl: T has a more restrict alignment than given");
  }

  template<typename... Args> HIPRT_DEVICE hiprtPimpl(Args &&...args)
  {
    PIMPL_PTR_CHECK_();
    new (&data) T(hiprt::forward<Args>(args)...);
  }

  HIPRT_DEVICE hiprtPimpl(hiprtPimpl const &o)
  {
    PIMPL_PTR_CHECK_();
    new (&data) T(*o);
  }

  HIPRT_DEVICE hiprtPimpl(hiprtPimpl &o)
  {
    PIMPL_PTR_CHECK_();
    new (&data) T(*o);
  }

  HIPRT_DEVICE hiprtPimpl(hiprtPimpl &&o)
  {
    PIMPL_PTR_CHECK_();
    new (&data) T(hiprt::move(*o));
  }

  HIPRT_DEVICE ~hiprtPimpl()
  {
  }

  HIPRT_DEVICE hiprtPimpl &operator=(hiprtPimpl const &o)
  {
    PIMPL_PTR_CHECK_();
    **this = *o;
    return *this;
  }

  HIPRT_DEVICE hiprtPimpl &operator=(hiprtPimpl &&o)
  {
    PIMPL_PTR_CHECK_();
    **this = hiprt::move(*o);
    return *this;
  }

  HIPRT_DEVICE T &operator*()
  {
    PIMPL_PTR_CHECK_();
    return *reinterpret_cast<T *>(&data);
  }

  HIPRT_DEVICE T const &operator*() const
  {
    PIMPL_PTR_CHECK_();
    return *reinterpret_cast<T const *>(&data);
  }

  HIPRT_DEVICE T *operator->()
  {
    PIMPL_PTR_CHECK_();
    return &**this;
  }

  HIPRT_DEVICE T const *operator->() const
  {
    PIMPL_PTR_CHECK_();
    return &**this;
  }
};

enum TraversalObjSize {
  SIZE_GEOM_TRAVERSAL_CUSTOM_STACK = 128,
  SIZE_SCENE_TRAVERSAL_CUSTOM_STACK = 160,
  SIZE_GEOM_TRAVERSAL_PRIVATE_STACK = 416,
  SIZE_SCENE_TRAVERSAL_PRIVATE_STACK = 448,
};

enum TraversalObjAlignment {
  ALIGNMENT_GEOM_TRAVERSAL_CUSTOM_STACK = 16,
  ALIGNMENT_SCENE_TRAVERSAL_CUSTOM_STACK = 16,
  ALIGNMENT_GEOM_TRAVERSAL_PRIVATE_STACK = 16,
  ALIGNMENT_SCENE_TRAVERSAL_PRIVATE_STACK = 32
};
