#ifndef OWDS_PHYSICS_BULLET3_JOINTS_JOINTFIXED_H
#define OWDS_PHYSICS_BULLET3_JOINTS_JOINTFIXED_H

#include <memory>

#include "overworld/Physics/Base/Joints/JointFixed.h"
#include "overworld/Physics/Bullet3/API.h"

namespace owds::bullet3 {
  class Context;

  class JointFixed final : public owds::JointFixed
  {
  public:
    using owds::JointFixed::JointFixed;

    JointFixed(
      owds::bullet3::Context& ctx,
      owds::JointLocation location);
    ~JointFixed() noexcept override;

    void setup() override;
    void remove() override;

  protected:
    owds::bullet3::Context& ctx_;
    std::unique_ptr<btGeneric6DofConstraint> joint_;
  };
} // namespace owds::bullet3

#endif // OWDS_PHYSICS_BULLET3_JOINTS_JOINTFIXED_H
