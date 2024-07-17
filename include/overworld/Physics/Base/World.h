#ifndef OWDS_PHYSICS_BASE_PHYSICSINTERFACE_H
#define OWDS_PHYSICS_BASE_PHYSICSINTERFACE_H

#include <array>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include "overworld/Engine/Common/Shapes/Shape.h"

namespace urdf {
  class Material;
  class Link;
  class Joint;
  class Geometry;
} // namespace urdf

namespace owds {
  class Actor;
  class Urdf;
  class JointRevolute;
  class JointContinuous;
  class JointPrismatic;
  class JointFixed;
  class JointPlanar;
  class JointFloating;
  class Model;
  class Material;

  class World
  {
  protected:
    explicit World(const std::filesystem::path& base_assets_path);

  public:
    virtual ~World();

    [[nodiscard]] virtual std::string getBackendName() const = 0;
    [[nodiscard]] virtual std::string getFullyQualifiedBackendName() const = 0;

    /**
     * The Unified Robotics Description Format specification mandates support for the following geometries:
     * - Sphere(s)
     * - Box(es)
     * - Cylinder(s)
     * - Custom mesh(es)
     *
     * Additionally, there are non-standard but commonly used geometries such as:
     * - Capsule(s) / Pill(s)
     */
    [[nodiscard]] owds::Shape createShapeBox(const owds::Color& color, const std::array<float, 3>& half_extents);
    [[nodiscard]] owds::Shape createShapeCapsule(const owds::Color& color, float radius, float height);
    [[nodiscard]] owds::Shape createShapeCylinder(const owds::Color& color, float radius, float height);
    [[nodiscard]] owds::Shape createShapeSphere(const owds::Color& color, float radius);
    [[nodiscard]] owds::Shape createShapeFromModel(const owds::Material& material, const std::string& path, const std::array<float, 3>& scale);

    [[nodiscard]] virtual owds::Actor& createActor(
      const owds::Shape& collision_shape,
      const std::vector<owds::Shape>& visual_shapes) = 0;

    [[nodiscard]] owds::Urdf& loadRobotFromDescription(const std::string& path);

    /**
     * @return
     */
    [[nodiscard]] virtual const std::vector<std::reference_wrapper<owds::Actor>>& getActors() const = 0;

    /**
     * The Unified Robotics Description Format specification mandates support for the following joint types:
     * - Revolute joint(s)
     * - Continuous joint(s)
     * - Prismatic joint(s)
     * - Fixed joint(s)
     * - Floating joint(s)
     * - Planar joint(s)
     */

    /**
     * A hinge joint that rotates along the axis and has a limited range specified by the upper and lower limits.
     * @param actor0 First actor
     * @param joint0_position Position of the joint relative to actor0
     * @param joint0_orientation XYZ rotations, in radians
     * @param actor1 Second actor
     * @param joint1_position Same as joint0_position but relative to actor1
     * @param joint1_orientation Same as joint0_orientation but relative to actor1
     */
    virtual owds::JointRevolute& createJointRevolute [[nodiscard]] (
      owds::Actor& actor0,
      const std::array<float, 3>& joint0_position,
      const std::array<float, 4>& joint0_orientation,
      owds::Actor& actor1,
      const std::array<float, 3>& joint1_position,
      const std::array<float, 4>& joint1_orientation) = 0;

    /**
     * A continuous hinge joint that rotates around the axis and has no upper and lower limits.
     * @param actor0 First actor
     * @param joint0_position Position of the joint relative to actor0
     * @param joint0_orientation XYZ rotations, in radians
     * @param actor1 Second actor
     * @param joint1_position Same as joint0_position but relative to actor1
     * @param joint1_orientation Same as joint0_orientation but relative to actor1
     */
    virtual owds::JointContinuous& createJointContinuous [[nodiscard]] (
      owds::Actor& actor0,
      const std::array<float, 3>& joint0_position,
      const std::array<float, 4>& joint0_orientation,
      owds::Actor& actor1,
      const std::array<float, 3>& joint1_position,
      const std::array<float, 4>& joint1_orientation) = 0;

    /**
     * A sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits.
     *
     * @param actor0 First actor
     * @param joint0_position Position of the joint relative to actor0
     * @param joint0_orientation XYZ rotations, in radians
     * @param actor1 Second actor
     * @param joint1_position Same as joint0_position but relative to actor1
     * @param joint1_orientation Same as joint0_orientation but relative to actor1
     */
    virtual owds::JointPrismatic& createJointPrismatic [[nodiscard]] (
      owds::Actor& actor0,
      const std::array<float, 3>& joint0_position,
      const std::array<float, 4>& joint0_orientation,
      owds::Actor& actor1,
      const std::array<float, 3>& joint1_position,
      const std::array<float, 4>& joint1_orientation) = 0;

    /**
     * All degrees of freedom are locked.
     *
     * @param actor0 First actor
     * @param joint0_position Position of the joint relative to actor0
     * @param joint0_orientation XYZ rotations, in radians
     * @param actor1 Second actor
     * @param joint1_position Same as joint0_position but relative to actor1
     * @param joint1_orientation Same as joint0_orientation but relative to actor1
     */
    virtual owds::JointFixed& createJointFixed [[nodiscard]] (
      owds::Actor& actor0,
      const std::array<float, 3>& joint0_position,
      const std::array<float, 4>& joint0_orientation,
      owds::Actor& actor1,
      const std::array<float, 3>& joint1_position,
      const std::array<float, 4>& joint1_orientation) = 0;

    /**
     * This joint allows motion for all 6 degrees of freedom.
     *
     * @param actor0 First actor
     * @param joint0_position Position of the joint relative to actor0
     * @param joint0_orientation XYZ rotations, in radians
     * @param actor1 Second actor
     * @param joint1_position Same as joint0_position but relative to actor1
     * @param joint1_orientation Same as joint0_orientation but relative to actor1
     */
    virtual owds::JointFloating& createJointFloating [[nodiscard]] (
      owds::Actor& actor0,
      const std::array<float, 3>& joint0_position,
      const std::array<float, 4>& joint0_orientation,
      owds::Actor& actor1,
      const std::array<float, 3>& joint1_position,
      const std::array<float, 4>& joint1_orientation) = 0;

    /**
     * This joint allows motion in a plane perpendicular to the axis.
     * @param actor0 First actor
     * @param joint0_position Position of the joint relative to actor0
     * @param joint0_orientation XYZ rotations, in radians
     * @param actor1 Second actor
     * @param joint1_position Same as joint0_position but relative to actor1
     * @param joint1_orientation Same as joint0_orientation but relative to actor1
     */
    virtual owds::JointPlanar& createJointPlanar [[nodiscard]] (
      owds::Actor& actor0,
      const std::array<float, 3>& joint0_position,
      const std::array<float, 4>& joint0_orientation,
      owds::Actor& actor1,
      const std::array<float, 3>& joint1_position,
      const std::array<float, 4>& joint1_orientation) = 0;

    /**
     * @param gravity Self-explanatory.
     */
    virtual void setGravity(const std::array<float, 3>& gravity) = 0;

    /**
     * @param delta_ms Time elapsed since last physics update, in milliseconds.
     */
    virtual void stepSimulation(float delta_ms) = 0;

  protected:
    void processMaterial(owds::Urdf& robot, const urdf::Material& urdf_material);
    void processLinks(owds::Urdf& robot, const urdf::Link& urdf_link);
    void processLink(owds::Urdf& robot, const urdf::Link& urdf_link);
    void processJoints(owds::Urdf& robot, const urdf::Link& urdf_link);
    void processJoint(owds::Urdf& robot, const urdf::Joint& urdf_joint);
    owds::Shape convertShape(const owds::Material& material, const urdf::Geometry& urdf_shape);

    std::filesystem::path base_assets_path_;
    owds::Model& preloaded_box_model_;
    owds::Model& preloaded_cylinder_model_;
    owds::Model& preloaded_sphere_model_;
    std::unordered_map<owds::Urdf*, std::unique_ptr<owds::Urdf>> loaded_robots_;
  };
} // namespace owds

#endif // OWDS_PHYSICS_BASE_PHYSICSINTERFACE_H
