#ifndef OWDS_PHYSICS_BULLET3_WORLD_H
#define OWDS_PHYSICS_BULLET3_WORLD_H

#include "overworld/Engine/Common/World.h"

namespace owds::bullet3 {
  class Context;

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
      owds::Actor& parent,
      const std::array<float, 3>& origin_position,
      const std::array<float, 3>& origin_position,
      owds::Actor& child,
      const std::array<float, 3>& joint_position,
      const std::array<float, 3>& joint_orientation) override;

    owds::JointContinuous& createJointContinuous [[nodiscard]] (
      owds::Actor& parent,
      const std::array<float, 3>& origin_position,
      const std::array<float, 3>& origin_position,
      owds::Actor& child,
      const std::array<float, 3>& joint_position,
      const std::array<float, 3>& joint_orientation) override;

    owds::JointPrismatic& createJointPrismatic [[nodiscard]] (
      owds::Actor& parent,
      const std::array<float, 3>& origin_position,
      const std::array<float, 3>& origin_position,
      owds::Actor& child,
      const std::array<float, 3>& joint_position,
      const std::array<float, 3>& joint_orientation) override;

    owds::JointFixed& createJointFixed [[nodiscard]] (
      owds::Actor& parent,
      const std::array<float, 3>& origin_position,
      const std::array<float, 3>& origin_position,
      owds::Actor& child,
      const std::array<float, 3>& joint_position,
      const std::array<float, 3>& joint_orientation) override;

    owds::JointFloating& createJointFloating [[nodiscard]] (
      owds::Actor& parent,
      const std::array<float, 3>& origin_position,
      const std::array<float, 3>& origin_position,
      owds::Actor& child,
      const std::array<float, 3>& joint_position,
      const std::array<float, 3>& joint_orientation) override;

    owds::JointPlanar& createJointPlanar [[nodiscard]] (
      owds::Actor& parent,
      const std::array<float, 3>& origin_position,
      const std::array<float, 3>& origin_position,
      owds::Actor& child,
      const std::array<float, 3>& joint_position,
      const std::array<float, 3>& joint_orientation) override;

    void setGravity(const std::array<float, 3>& gravity) override;
    void stepSimulation(float delta) override;

  protected:
    std::unique_ptr<Context> ctx_;
  };
} // namespace owds::bullet3

#endif // OWDS_PHYSICS_BULLET3_WORLD_H
