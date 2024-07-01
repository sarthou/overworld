#ifndef OWDS_PHYSICS_BULLET3_JOINTS_JOINTPRISMATIC_H
#define OWDS_PHYSICS_BULLET3_JOINTS_JOINTPRISMATIC_H

#include <memory>

#include "overworld/Physics/Base/Joints/JointPrismatic.h"
#include "overworld/Physics/Bullet3/API.h"

namespace owds::bullet3 {
  class Context;

  class JointPrismatic final : public owds::JointPrismatic
  {
  public:
    using owds::JointPrismatic::JointPrismatic;

    JointPrismatic(
      owds::bullet3::Context& ctx,
      owds::JointLocation location);
    ~JointPrismatic() noexcept override;

    void setup() override;
    void remove() override;

    void setLimits(float lower_distance_limit, float upper_distance_limit) override;
  protected:
    owds::bullet3::Context& ctx_;
    std::unique_ptr<btGeneric6DofConstraint> joint_;
  };
} // namespace owds::bullet3

#endif // OWDS_PHYSICS_BULLET3_JOINTS_JOINTPRISMATIC_H
