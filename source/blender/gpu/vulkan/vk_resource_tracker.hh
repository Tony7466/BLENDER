/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_utility_mixins.hh"
#include "BLI_vector.hh"

#include "vk_common.hh"

namespace blender::gpu {
class VKContext;
class VKCommandBuffer;

/**
 * In vulkan multiple commands can be in flight simultaneously.
 *
 * These commands can share the same resources like descriptor sets
 * or push constants. When between commands these resources are updated
 * a new version of these resources should be created.
 *
 * When a resource is updated it should check the submission id of the
 * command buffer. If this is different than resources can be reused.
 * If they the submission id is the same that the resource uses it
 * should create a new version of the resource to now intervene with
 * other commands that uses the resource.
 *
 * SubmissionID is the identifier to keep track if a new submission is
 * being recorded.
 */
struct SubmissionID {
 private:
  int64_t id_ = -1;

 public:
  SubmissionID() = default;

 private:
  /**
   * Reset the submission id.
   *
   * This should only be called during initialization of the command buffer.
   * As it leads to undesired behavior after resources are already tracking
   * the submission id.
   */
  void reset()
  {
    id_ = 0;
  }

  /**
   * Change the submission id.
   *
   * Is called when submitting a command buffer to the queue. In this case resource
   * known that the next time it is used that it can free its sub resources used by
   * the previous submission.
   */
  void next()
  {
    id_++;
  }

 public:
  const SubmissionID &operator=(const SubmissionID &other)
  {
    id_ = other.id_;
    return *this;
  }

  bool operator==(const SubmissionID &other)
  {
    return id_ == other.id_;
  }

  bool operator!=(const SubmissionID &other)
  {
    return id_ != other.id_;
  }

  friend class VKCommandBuffer;
};

class SubmissionTracker {
 public:
  enum class Result {
    FREE_AND_CREATE_NEW_RESOURCE,
    CREATE_NEW_RESOURCE,
    USE_LAST_RESOURCE,
  };

 private:
  SubmissionID last_known_id_;

 public:
  Result submission_tracker_pre_update(VKContext &context, bool is_dirty);
};

template<typename Resource> class ResourceTracker : NonCopyable {
  SubmissionTracker submission_tracker_;
  Vector<std::unique_ptr<Resource>> tracked_resources_;

 protected:
  ResourceTracker<Resource>()
  {
  }
  ResourceTracker<Resource>(ResourceTracker<Resource> &&other)
      : submission_tracker_(other.submission_tracker_),
        tracked_resources_(std::move(other.tracked_resources_))
  {
  }

  ResourceTracker<Resource> &operator=(ResourceTracker<Resource> &&other)
  {
    submission_tracker_ = other.submission_tracker_;
    tracked_resources_ = std::move(other.tracked_resources_);
    return *this;
  }

  virtual ~ResourceTracker()
  {
    free_tracked_resources();
  }

  std::unique_ptr<Resource> &handle_pre_update(VKContext &context, const bool is_dirty)
  {
    SubmissionTracker::Result resource_action = submission_tracker_.submission_tracker_pre_update(
        context, is_dirty);
    switch (resource_action) {
      case SubmissionTracker::Result::FREE_AND_CREATE_NEW_RESOURCE:
        free_tracked_resources();
        tracked_resources_.append(create_new_resource(context));
        break;

      case SubmissionTracker::Result::CREATE_NEW_RESOURCE:
        tracked_resources_.append(create_new_resource(context));
        break;

      case SubmissionTracker::Result::USE_LAST_RESOURCE:
        break;
    }
    return active_resource();
  }

  virtual std::unique_ptr<Resource> create_new_resource(VKContext &context) = 0;

  std::unique_ptr<Resource> &active_resource()
  {
    BLI_assert(!tracked_resources_.is_empty());
    return tracked_resources_.last();
  }

 private:
  void free_tracked_resources()
  {
    tracked_resources_.clear();
  }
};

}  // namespace blender::gpu
