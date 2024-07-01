#ifndef OWDS_PHYSICS_PHYSX_JOINTS_JOINTFIXED_H
#define OWDS_PHYSICS_PHYSX_JOINTS_JOINTFIXED_H

#include "overworld/Physics/Base/Joints/JointFixed.h"
#include "overworld/Physics/PhysX/API.h"

namespace owds::physx {
    class Context;

    class JointFixed final : public owds::JointFixed {
    public:
        using owds::JointFixed::JointFixed;

        JointFixed(
            owds::physx::Context& ctx,
            owds::JointLocation location);

        void setup() override;
        void remove() override;
    protected:
        owds::physx::Context& ctx_;
        PxPtr<::physx::PxFixedJoint> joint_{};
    };
}

#endif // OWDS_PHYSICS_PHYSX_JOINTS_JOINTFIXED_H
