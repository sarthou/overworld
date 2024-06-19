#ifndef OWDS_PHYSICS_PHYSX_WORLD_H
#define OWDS_PHYSICS_PHYSX_WORLD_H

#include <memory>

#include "overworld/Physics/Base/World.h"

namespace owds::physx {
  class Context;
  class Actor;

  class World final : public owds::World
  {
  public:
    explicit World(const std::filesystem::path& base_assets_path);
    ~World() override;

    World(const World& other) = delete;
    World& operator=(const World& other) = delete;

    // todo: deal with move ctor / assignment move operator
    World(World&& other) = delete;
    World& operator=(World&& other) = delete;

    [[nodiscard]] std::string getBackendName() const override;
    [[nodiscard]] std::string getFullyQualifiedBackendName() const override;

    using owds::World::createActor;

    [[nodiscard]] owds::Actor& createActor(
      const owds::Shape& collision_shape,
      const std::vector<owds::Shape>& visual_shapes) override;

    [[nodiscard]] const std::vector<std::reference_wrapper<owds::Actor>>& getActors() const override;

    owds::JointRevolute& createJointRevolute [[nodiscard]] (
      owds::Actor& actor0,
      const std::array<float, 3>& joint0_position,
      const std::array<float, 3>& joint0_orientation,
      owds::Actor& actor1,
      const std::array<float, 3>& joint1_position,
      const std::array<float, 3>& joint1_orientation) override;

    owds::JointContinuous& createJointContinuous [[nodiscard]] (
      owds::Actor& actor0,
      const std::array<float, 3>& joint0_position,
      const std::array<float, 3>& joint0_orientation,
      owds::Actor& actor1,
      const std::array<float, 3>& joint1_position,
      const std::array<float, 3>& joint1_orientation) override;

    owds::JointPrismatic& createJointPrismatic [[nodiscard]] (
      owds::Actor& actor0,
      const std::array<float, 3>& joint0_position,
      const std::array<float, 3>& joint0_orientation,
      owds::Actor& actor1,
      const std::array<float, 3>& joint1_position,
      const std::array<float, 3>& joint1_orientation) override;

    owds::JointFixed& createJointFixed [[nodiscard]] (
      owds::Actor& actor0,
      const std::array<float, 3>& joint0_position,
      const std::array<float, 3>& joint0_orientation,
      owds::Actor& actor1,
      const std::array<float, 3>& joint1_position,
      const std::array<float, 3>& joint1_orientation) override;

    owds::JointFloating& createJointFloating [[nodiscard]] (
      owds::Actor& actor0,
      const std::array<float, 3>& joint0_position,
      const std::array<float, 3>& joint0_orientation,
      owds::Actor& actor1,
      const std::array<float, 3>& joint1_position,
      const std::array<float, 3>& joint1_orientation) override;

    owds::JointPlanar& createJointPlanar [[nodiscard]] (
      owds::Actor& actor0,
      const std::array<float, 3>& joint0_position,
      const std::array<float, 3>& joint0_orientation,
      owds::Actor& actor1,
      const std::array<float, 3>& joint1_position,
      const std::array<float, 3>& joint1_orientation) override;

    void setGravity(const std::array<float, 3>& gravity) override;
    void stepSimulation(float delta) override;

  protected:
    std::unique_ptr<owds::physx::Context> ctx_;
  };
} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_WORLD_H
