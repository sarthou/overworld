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
#include "overworld/Engine/Common/WorldTypes.h"

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

    std::filesystem::path base_assets_path_;
    owds::Model& preloaded_box_model_;
    owds::Model& preloaded_cylinder_model_;
    owds::Model& preloaded_sphere_model_;

    AmbientLight ambient_light_;
    PointLights point_lights_;
    std::vector<DebugText_t> debug_texts_;
    std::vector<DebugLine> debug_lines_;

    std::vector<VirtualCamera> cameras_;
    std::atomic<bool> has_render_request_;

    std::unordered_map<std::size_t, Actor*> actors_;
    std::unordered_map<std::size_t, Urdf*> urdfs_;

  public:
    virtual ~World();

    [[nodiscard]] virtual std::string getBackendName() const = 0;
    [[nodiscard]] virtual std::string getFullyQualifiedBackendName() const = 0;

    /* ACTORS */

    size_t createStaticActor(const owds::urdf::Geometry_t& collision_geometry,
                             const std::vector<owds::urdf::Geometry_t>& visual_geometries,
                             const std::array<float, 3>& position = {0.f, 0.f, 0.f},
                             const std::array<float, 4>& rotation = {0., 0., 0., 1.});

    size_t createActor(const owds::urdf::Geometry_t& collision_geometry,
                       const std::vector<owds::urdf::Geometry_t>& visual_geometries,
                       const std::array<float, 3>& position = {0.f, 0.f, 0.f},
                       const std::array<float, 4>& rotation = {0., 0., 0., 1.});

    size_t createVisualActor(const std::vector<owds::urdf::Geometry_t>& visual_geometries,
                             const std::array<float, 3>& position = {0.f, 0.f, 0.f},
                             const std::array<float, 4>& rotation = {0., 0., 0., 1.});

    size_t loadUrdf(const std::string& path,
                    const std::array<float, 3>& position,
                    const std::array<float, 3>& orientation,
                    bool from_base_path = true);
    int getNumJoints(size_t urdf_id) const;
    std::pair<std::array<float, 3>, std::array<float, 4>> getBasePositionAndOrientation(int body_id) const;
    void setBasePositionAndOrientation(int body_id, const std::array<float, 3>& position, const std::array<float, 4>& orientation);
    void setBaseVelocity(int body_id, const std::array<float, 3>& linear_velocity, const std::array<float, 3>& angular_velocity);
    bool setJointState(int body_id, const std::string& joint_name, double position, double velocity = 0);
    int getLinkId(int body_id, const std::string& link_name);

    void setMass(int body_id, int link_index, double mass_kg);
    void setStaticFriction(int body_id, int link_index, double friction);
    void setDynamicFriction(int body_id, int link_index, double friction);
    void setRestitution(int body_id, int link_index, double restitution);

    const std::unordered_map<std::size_t, Actor*>& getActors() const { return actors_; }

    /* COLISIONS */

    std::vector<RaycastHitResult_t> raycasts(const std::vector<std::array<float, 3>>& origins,
                                             const std::vector<std::array<float, 3>>& destinations,
                                             float max_distance);

    AABB_t getAABB(int body_id, int link_index = -1);
    AABB_t getLocalAABB(int body_id, int link_index = -1);

    std::unordered_set<int> getOverlappingObjects(int body_id, int link_index);

    /* LIGHTS */

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

    /* DEBUG */

    int addDebugText(const std::string& text,
                     const std::array<float, 3>& position,
                     float height,
                     const std::array<float, 3>& color = {1.0, 1.0, 1.0},
                     bool centered = true,
                     double life_time = 0,
                     int replace_id = -1,
                     int body_id = -1, int link_index = -1);
    void removeDebugText(int id);

    int addDebugLine(const std::array<float, 3>& position_from,
                     const std::array<float, 3>& position_to,
                     const std::array<float, 3>& color = {1.0, 1.0, 1.0},
                     double life_time = 0,
                     int replace_id = -1,
                     int body_id = -1, int link_index = -1);
    int addDebugLine(const std::vector<std::array<float, 3>>& vertices,
                     const std::vector<unsigned int>& indices,
                     const std::array<float, 3>& color = {1.0, 1.0, 1.0},
                     double life_time = 0,
                     int replace_id = -1,
                     int body_id = -1, int link_index = -1);
    void removeDebugLine(int id);

    void processDebugLifeTime(double delta);

    /* CAMERAS */

    int addCamera(unsigned int width, unsigned int height, float fov, owds::CameraView_e view_type, float near_plane, float far_plane);
    bool setCameraPositionAndLookAt(int id, const std::array<float, 3>& eye_position, const std::array<float, 3>& dst_position);
    bool setCameraPositionAndDirection(int id, const std::array<float, 3>& eye_position, const std::array<float, 3>& eye_direction);
    // requestCameraRender blocks until the request has been treated. Do not cahnge the cameras during this time
    void requestCameraRender(const std::vector<int>& ids);
    void getCameraImage(int id, uint32_t** image, unsigned int& width, unsigned int& height);
    std::unordered_set<uint32_t> getCameraSementation(int id);

    /* PHYSICS */

    /**
     * @param gravity Self-explanatory.
     */
    virtual void setGravity(const std::array<float, 3>& gravity) = 0;

    void setTimeStep(double delta_ms) { time_step_ = delta_ms; }
    virtual void stepSimulation(float delta_ms = 0) = 0;

  protected:
    double time_step_;

    Actor* getActor(int body_id, int link_index = -1);

    virtual size_t createActor(const owds::Shape& collision_shape,
                               const std::vector<owds::Shape>& visual_shapes,
                               const std::array<float, 3>& position,
                               const std::array<float, 4>& orientation) = 0;

    virtual size_t createStaticActor(const owds::Shape& collision_shape,
                                     const std::vector<owds::Shape>& visual_shapes,
                                     const std::array<float, 3>& position,
                                     const std::array<float, 4>& orientation) = 0;

    virtual owds::Urdf* loadUrdf(const urdf::Urdf_t& urdf) = 0;
    virtual void insertUrdf(owds::Urdf* urdf) = 0;

    urdf::Urdf_t getUrdf(const std::string& path, bool from_base_path);
    void loadUrdfLink(owds::Urdf* urdf, const urdf::Urdf_t& model,
                      const std::string& parent,
                      const std::string& link_name,
                      const std::array<float, 3>& position,
                      const std::array<float, 3>& orientation);

    virtual std::unordered_set<int> getOverlappingActors(Actor* actor) = 0;

    owds::Shape createShapeBox(const owds::Color& color, const std::array<float, 3>& half_extents, glm::mat4& transform);
    owds::Shape createShapeCapsule(const owds::Color& color, float radius, float height, glm::mat4& transform);
    owds::Shape createShapeCylinder(const owds::Color& color, float radius, float height, glm::mat4& transform);
    owds::Shape createShapeSphere(const owds::Color& color, float radius, glm::mat4& transform);
    owds::Shape createShapeFromModel(const owds::Material& material, const std::string& path, const std::array<float, 3>& scale, glm::mat4& transform);

    owds::Shape convertShape(const urdf::Geometry_t& geometry);
    owds::Shape convertShape(const urdf::Geometry_t& urdf_shape, glm::mat4& transform);

    virtual void performRaycastsInParallel(const std::vector<std::array<float, 3>>& origins,
                                           const std::vector<std::array<float, 3>>& destinations,
                                           float max_distance,
                                           std::vector<RaycastHitResult_t>& results) = 0;
  };
} // namespace owds

#endif // OWDS_COMMON_WORLD_H
