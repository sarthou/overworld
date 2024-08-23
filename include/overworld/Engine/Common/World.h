#ifndef OWDS_COMMON_WORLD_H
#define OWDS_COMMON_WORLD_H

#include <array>
#include <atomic>
#include <filesystem>
#include <glm/matrix.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "overworld/Engine/Common/Camera/VirtualCamera.h"
#include "overworld/Engine/Common/Debug/DebugLine.h"
#include "overworld/Engine/Common/Debug/DebugText.h"
#include "overworld/Engine/Common/Lights/AmbientLight.h"
#include "overworld/Engine/Common/Lights/PointLights.h"
#include "overworld/Engine/Common/Shapes/Shape.h"
#include "overworld/Engine/Common/Urdf/UrdfLoader.h"

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

  class Renderer;

  class World
  {
    friend Renderer;

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
    [[nodiscard]] owds::Shape createShapeBox(const owds::Color& color, const std::array<float, 3>& half_extents, glm::mat4& transform);
    [[nodiscard]] owds::Shape createShapeCapsule(const owds::Color& color, float radius, float height, glm::mat4& transform);
    [[nodiscard]] owds::Shape createShapeCylinder(const owds::Color& color, float radius, float height, glm::mat4& transform);
    [[nodiscard]] owds::Shape createShapeSphere(const owds::Color& color, float radius, glm::mat4& transform);
    [[nodiscard]] owds::Shape createShapeFromModel(const owds::Material& material, const std::string& path, const std::array<float, 3>& scale, glm::mat4& transform);

    [[nodiscard]] virtual owds::Actor& createActor(
      const owds::Shape& collision_shape,
      const std::vector<owds::Shape>& visual_shapes) = 0;

    [[nodiscard]] owds::Urdf& loadRobotFromDescription(const std::string& path, bool from_base_path = true);

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
     * @param parent First actor
     * @param origin_position Position of the joint relative to parent
     * @param origin_position XYZ rotations, in radians
     * @param child Second actor
     * @param joint_position Same as origin_position but relative to child
     * @param joint_orientation Same as origin_position but relative to child
     */
    virtual owds::JointRevolute& createJointRevolute [[nodiscard]] (
      owds::Actor& parent,
      const std::array<float, 3>& origin_position,
      const std::array<float, 4>& origin_orientation,
      owds::Actor& child,
      const std::array<float, 3>& joint_position,
      const std::array<float, 4>& joint_orientation) = 0;

    /**
     * A continuous hinge joint that rotates around the axis and has no upper and lower limits.
     * @param parent First actor
     * @param origin_position Position of the joint relative to parent
     * @param origin_position XYZ rotations, in radians
     * @param child Second actor
     * @param joint_position Same as origin_position but relative to child
     * @param joint_orientation Same as origin_position but relative to child
     */
    virtual owds::JointContinuous& createJointContinuous [[nodiscard]] (
      owds::Actor& parent,
      const std::array<float, 3>& origin_position,
      const std::array<float, 4>& origin_orientation,
      owds::Actor& child,
      const std::array<float, 3>& joint_position,
      const std::array<float, 4>& joint_orientation) = 0;

    /**
     * A sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits.
     *
     * @param parent First actor
     * @param origin_position Position of the joint relative to parent
     * @param origin_position XYZ rotations, in radians
     * @param child Second actor
     * @param joint_position Same as origin_position but relative to child
     * @param joint_orientation Same as origin_position but relative to child
     */
    virtual owds::JointPrismatic& createJointPrismatic [[nodiscard]] (
      owds::Actor& parent,
      const std::array<float, 3>& origin_position,
      const std::array<float, 4>& origin_orientation,
      owds::Actor& child,
      const std::array<float, 3>& joint_position,
      const std::array<float, 4>& joint_orientation) = 0;

    /**
     * All degrees of freedom are locked.
     *
     * @param parent First actor
     * @param origin_position Position of the joint relative to parent
     * @param origin_position XYZ rotations, in radians
     * @param child Second actor
     * @param joint_position Same as origin_position but relative to child
     * @param joint_orientation Same as origin_position but relative to child
     */
    virtual owds::JointFixed& createJointFixed [[nodiscard]] (
      owds::Actor& parent,
      const std::array<float, 3>& origin_position,
      const std::array<float, 4>& origin_orientation,
      owds::Actor& child,
      const std::array<float, 3>& joint_position,
      const std::array<float, 4>& joint_orientation) = 0;

    /**
     * This joint allows motion for all 6 degrees of freedom.
     *
     * @param parent First actor
     * @param origin_position Position of the joint relative to parent
     * @param origin_position XYZ rotations, in radians
     * @param child Second actor
     * @param joint_position Same as origin_position but relative to child
     * @param joint_orientation Same as origin_position but relative to child
     */
    virtual owds::JointFloating& createJointFloating [[nodiscard]] (
      owds::Actor& parent,
      const std::array<float, 3>& origin_position,
      const std::array<float, 4>& origin_orientation,
      owds::Actor& child,
      const std::array<float, 3>& joint_position,
      const std::array<float, 4>& joint_orientation) = 0;

    /**
     * This joint allows motion in a plane perpendicular to the axis.
     * @param parent First actor
     * @param origin_position Position of the joint relative to parent
     * @param origin_position XYZ rotations, in radians
     * @param child Second actor
     * @param joint_position Same as origin_position but relative to child
     * @param joint_orientation Same as origin_position but relative to child
     */
    virtual owds::JointPlanar& createJointPlanar [[nodiscard]] (
      owds::Actor& parent,
      const std::array<float, 3>& origin_position,
      const std::array<float, 4>& origin_orientation,
      owds::Actor& child,
      const std::array<float, 3>& joint_position,
      const std::array<float, 4>& joint_orientation) = 0;

    void setAmbientLight(const std::array<float, 3>& direction,
                         const std::array<float, 3>& color = {1.0, 1.0, 1.0},
                         float ambient_strength = 1.0f,
                         float diffuse_strength = 1.0f,
                         float specular_strength = 1.0f);
    void setAmbientLightDirection(const std::array<float, 3>& direction);
    void setAmbientLightColor(const std::array<float, 3>& color);
    void setAmbientLightAmbientStrength(float ambient_strength);

    std::size_t addPointLight(const std::array<float, 3>& position,
                              const std::array<float, 3>& color = {1.0, 1.0, 1.0},
                              float ambient_strength = 1.0f,
                              float diffuse_strength = 1.0f,
                              float specular_strength = 1.0f,
                              float attenuation_radius = 10.f);
    void removePointLight(std::size_t id);
    void setPointLightColor(std::size_t id, const glm::vec3& color);
    void setPointLightPosition(std::size_t id, const glm::vec3& position);
    void setPointLightAmbientStrength(std::size_t id, float strength);

    int addDebugText(const std::string& text,
                     const std::array<float, 3>& position,
                     float height,
                     const std::array<float, 3>& color = {1.0, 1.0, 1.0},
                     bool centered = true,
                     int replace_id = -1);
    void removeDebugText(int id);

    int addDebugLine(const std::array<float, 3>& position_from,
                     const std::array<float, 3>& position_to,
                     const std::array<float, 3>& color = {1.0, 1.0, 1.0},
                     int replace_id = -1);
    int addDebugLine(const std::vector<std::array<float, 3>>& vertices,
                     const std::vector<unsigned int>& indices,
                     const std::array<float, 3>& color = {1.0, 1.0, 1.0},
                     int replace_id = -1);
    void removeDebugLine(int id);

    int addCamera(unsigned int width, unsigned int height, float fov, owds::CameraView_e view_type, float near_plane, float far_plane);
    bool setCameraPositionAndLookAt(int id, const std::array<float, 3>& eye_position, const std::array<float, 3>& dst_position);
    bool setCameraPositionAndDirection(int id, const std::array<float, 3>& eye_position, const std::array<float, 3>& eye_direction);
    // requestCameraRender blocks until the request has been treated. Do not cahnge the cameras during this time
    void requestCameraRender(const std::vector<int>& ids);
    void getCameraImage(int id, uint32_t** image, unsigned int& width, unsigned int& height);
    std::unordered_set<uint32_t> getCameraSementation(int id);

    /**
     * @param gravity Self-explanatory.
     */
    virtual void setGravity(const std::array<float, 3>& gravity) = 0;

    /**
     * @param delta_ms Time elapsed since last physics update, in milliseconds.
     */
    virtual void stepSimulation(float delta_ms) = 0;

  protected:
    void processLink(owds::Urdf& robot, const urdf::Link_t& urdf_link);
    void processJoint(owds::Urdf& robot, const urdf::Joint_t& urdf_joint);
    owds::Shape convertShape(const urdf::Geometry_t& urdf_shape, glm::mat4& transform);

    std::filesystem::path base_assets_path_;
    owds::Model& preloaded_box_model_;
    owds::Model& preloaded_cylinder_model_;
    owds::Model& preloaded_sphere_model_;
    std::unordered_map<owds::Urdf*, std::unique_ptr<owds::Urdf>> loaded_urdfs_;

    AmbientLight ambient_light_;
    PointLights point_lights_;
    std::vector<DebugText_t> debug_texts_;
    std::vector<DebugLine> debug_lines_;

    std::vector<VirtualCamera> cameras_;
    std::atomic<bool> has_render_request_;
  };
} // namespace owds

#endif // OWDS_COMMON_WORLD_H
