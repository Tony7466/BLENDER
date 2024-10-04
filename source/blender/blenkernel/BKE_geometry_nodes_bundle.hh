/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_geometry_nodes_bundle_fwd.hh"
#include "BKE_node.hh"

#include "BLI_cpp_type.hh"
#include "BLI_implicit_sharing_ptr.hh"
#include "BLI_math_base.h"

#include "DNA_node_types.h"

namespace blender::bke {

class SocketListSignature {
 public:
  struct Item {
    const bNodeSocketType *socket_type = nullptr;
    std::string name;
  };

  Vector<Item> items;

  static const std::shared_ptr<SocketListSignature> &empty()
  {
    static const std::shared_ptr<SocketListSignature> empty_signature =
        std::make_shared<SocketListSignature>();
    return empty_signature;
  }
};

class BundleSignature {
 private:
  std::shared_ptr<SocketListSignature> sockets_;
  Vector<int64_t> offsets_;
  int64_t size_in_bytes_ = 0;

 public:
  BundleSignature(std::shared_ptr<SocketListSignature> sockets)
      : sockets_(sockets), offsets_(sockets->items.size())
  {
    size_in_bytes_ = 0;
    int64_t max_alignment = 0;
    for (const int64_t i : offsets_.index_range()) {
      const SocketListSignature::Item &socket_item = sockets_->items[i];
      const CPPType &cpp_type = *socket_item.socket_type->geometry_nodes_cpp_type;
      size_in_bytes_ = ceil_to_multiple_ul(size_in_bytes_, cpp_type.alignment());
      offsets_[i] = size_in_bytes_;
      size_in_bytes_ += cpp_type.size();
      max_alignment = std::max(max_alignment, cpp_type.alignment());
    }
    size_in_bytes_ = ceil_to_multiple_ul(size_in_bytes_, max_alignment);
  }

  static const std::shared_ptr<BundleSignature> &empty()
  {
    static const std::shared_ptr<BundleSignature> empty_signature =
        std::make_shared<BundleSignature>(SocketListSignature::empty());
    return empty_signature;
  }

  const std::shared_ptr<SocketListSignature> &sockets_ptr() const
  {
    return sockets_;
  }

  const SocketListSignature &sockets() const
  {
    return *sockets_;
  }

  int64_t size_in_bytes() const
  {
    return size_in_bytes_;
  }

  Span<int64_t> offsets() const
  {
    return offsets_;
  }

  int64_t offset(const int64_t index) const
  {
    return offsets_[index];
  }

  const CPPType &cpp_type(const int64_t index) const
  {
    return *sockets_->items[index].socket_type->geometry_nodes_cpp_type;
  }
};

inline void get_socket_list_signature_map(const SocketListSignature &signature_a,
                                          const SocketListSignature &signature_b,
                                          MutableSpan<std::optional<int>> r_a_by_b)
{
  BLI_assert(r_a_by_b.size() == signature_b.items.size());

  for (const int a_i : signature_a.items.index_range()) {
    const SocketListSignature::Item &item_a = signature_a.items[a_i];
    for (const int b_i : signature_b.items.index_range()) {
      const SocketListSignature::Item &item_b = signature_b.items[b_i];
      if (item_a.name == item_b.name && item_a.socket_type == item_b.socket_type) {
        r_a_by_b[b_i] = a_i;
      }
    }
  }
}

class Bundle : public ImplicitSharingInfo {
 private:
  std::shared_ptr<BundleSignature> signature_;
  void *data_;

 public:
  Bundle() : signature_(BundleSignature::empty()), data_(nullptr) {}

  explicit Bundle(std::shared_ptr<BundleSignature> signature, void *data)
      : signature_(std::move(signature)), data_(data)
  {
  }

  ~Bundle()
  {
    for (const int i : signature_->sockets().items.index_range()) {
      void *ptr = POINTER_OFFSET(data_, signature_->offset(i));
      signature_->cpp_type(i).destruct(ptr);
    }
    MEM_SAFE_FREE(data_);
  }

  const void *data() const
  {
    return data_;
  }

  const void *data(const int64_t index) const
  {
    return POINTER_OFFSET(data_, signature_->offset(index));
  }

  const BundleSignature &signature() const
  {
    return *signature_;
  }

  void delete_self_with_data() override
  {
    MEM_delete(this);
  }
};

}  // namespace blender::bke
