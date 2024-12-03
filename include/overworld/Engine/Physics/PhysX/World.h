#ifndef OWDS_PHYSICS_PHYSX_WORLD_H
#define OWDS_PHYSICS_PHYSX_WORLD_H

#include <array>
#include <cstddef>
#include <glm/vec3.hpp>
#include <memory>
#include <string>
#include <vector>

#include "overworld/Engine/Common/World.h"

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
    using owds::World::createStaticActor;

    size_t createActor(const owds::Shape& collision_shape,
                       const std::vector<owds::Shape>& visual_shapes) override;

    size_t createStaticActor(const owds::Shape& collision_shape,
                             const std::vector<owds::Shape>& visual_shapes,
                             const glm::vec3& position,
                             const glm::quat& orientation) override;

    void setGravity(const std::array<float, 3>& gravity) override;
    void stepSimulation(float delta) override;

  protected:
    std::unique_ptr<owds::physx::Context> ctx_;

  private:
  };
} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_WORLD_H
