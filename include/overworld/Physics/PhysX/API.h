#ifndef OWDS_PHYSICS_PHYSX_API_H
#define OWDS_PHYSICS_PHYSX_API_H

#include <PxPhysicsAPI.h>
#include <cassert>
#include <utility>

namespace owds::physx {
  template<typename T>
  class PxPtr
  {
  public:
    PxPtr() = default;
    PxPtr(T* ptr) : internal_handle_(ptr) {}

    PxPtr(const PxPtr& other) = delete;
    PxPtr& operator=(const PxPtr& other) = delete;

    PxPtr(PxPtr&& other) noexcept : internal_handle_(std::exchange(other.internal_handle_, nullptr)){}
    PxPtr& operator=(PxPtr&& other) noexcept
    {
      std::swap(internal_handle_, other.internal_handle_);
      return *this;
    }

    ~PxPtr()
    {
      if(internal_handle_)
      {
        internal_handle_->release();
        internal_handle_ = nullptr;
      }
    }

    T* get() const
    {
      return internal_handle_;
    }

    T* operator->() const
    {
      assert(internal_handle_ && "Attempted to dereference null pointer");
      return internal_handle_;
    }

    T& operator*() const
    {
      assert(internal_handle_ && "Attempted to dereference null pointer");
      // NOLINTNEXTLINE yes yes I know the pointer may be null in release builds, but we only care about these sorts of checks in debug builds.
      return *internal_handle_;
    }

    explicit operator bool() const
    {
      return !!internal_handle_;
    }

  protected:
    T* internal_handle_{};
  };
} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_API_H