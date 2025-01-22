#include "overworld/Engine/Common/World.h"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <glm/detail/type_quat.hpp>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/matrix.hpp>
#include <iostream>
#include <string>
#include <unistd.h>
#include <unordered_set>
#include <vector>

#include "overworld/Engine/Common/Camera/CameraView.h"
#include "overworld/Engine/Common/Debug/DebugLine.h"
#include "overworld/Engine/Common/Debug/DebugText.h"
#include "overworld/Engine/Common/Lights/AmbientLight.h"
#include "overworld/Engine/Common/Models/Color.h"
#include "overworld/Engine/Common/Models/Material.h"
#include "overworld/Engine/Common/Models/ModelManager.h"
#include "overworld/Engine/Common/Shapes/Shape.h"
#include "overworld/Engine/Common/Shapes/ShapeBox.h"
#include "overworld/Engine/Common/Shapes/ShapeCapsule.h"
#include "overworld/Engine/Common/Shapes/ShapeCustomMesh.h"
#include "overworld/Engine/Common/Shapes/ShapeCylinder.h"
#include "overworld/Engine/Common/Shapes/ShapeDummy.h"
#include "overworld/Engine/Common/Shapes/ShapeSphere.h"
#include "overworld/Engine/Common/Urdf/Actor.h"
#include "overworld/Engine/Common/Urdf/Urdf.h"
#include "overworld/Engine/Common/Urdf/UrdfLoader.h"
#include "overworld/Engine/Common/Urdf/VisualActor.h"
#include "overworld/Utils/GlmMath.h"
#include "overworld/Utils/RosPackage.h"

namespace owds {

  World::World(const std::filesystem::path& base_assets_path) : base_assets_path_(base_assets_path),
                                                                preloaded_box_model_(owds::ModelManager::get().load(
                                                                  base_assets_path / "models/basic_shapes/cube.obj")),
                                                                preloaded_cylinder_model_(owds::ModelManager::get().load(
                                                                  base_assets_path / "models/basic_shapes/cylinder.obj")),
                                                                preloaded_sphere_model_(owds::ModelManager::get().load(
                                                                  base_assets_path / "models/basic_shapes/sphere.obj"))
  {}

  World::~World()
  {
    for(const auto& actor : actors_)
      delete actor.second;
  }

  /* ACTORS */

  size_t World::createStaticActor(const owds::urdf::Geometry_t& collision_geometry,
                                  const std::vector<owds::urdf::Geometry_t>& visual_geometries,
                                  const std::array<double, 3>& position,
                                  const std::array<double, 4>& rotation)
  {
    owds::Shape collision_shape = convertShape(collision_geometry);
    std::vector<owds::Shape> visual_shapes;
    visual_shapes.reserve(visual_geometries.size());
    for(const auto& geometry : visual_geometries)
      visual_shapes.emplace_back(convertShape(geometry));

    return createStaticActor(collision_shape, visual_shapes,
                             position, rotation);
  }

  size_t World::createActor(const owds::urdf::Geometry_t& collision_geometry,
                            const std::vector<owds::urdf::Geometry_t>& visual_geometries,
                            const std::array<double, 3>& position,
                            const std::array<double, 4>& rotation)
  {
    owds::Shape collision_shape = convertShape(collision_geometry);
    std::vector<owds::Shape> visual_shapes;
    visual_shapes.reserve(visual_geometries.size());
    for(const auto& geometry : visual_geometries)
      visual_shapes.emplace_back(convertShape(geometry));

    return createActor(collision_shape, visual_shapes,
                       position, rotation);
  }

  size_t World::createVisualActor(const std::vector<owds::urdf::Geometry_t>& visual_geometries,
                                  const std::array<double, 3>& position,
                                  const std::array<double, 4>& rotation)
  {
    std::vector<owds::Shape> visual_shapes;
    visual_shapes.reserve(visual_geometries.size());
    for(const auto& geometry : visual_geometries)
      visual_shapes.emplace_back(convertShape(geometry));

    Actor* actor = new VisualActor(visual_shapes);
    actor->setup(position, rotation);

    actors_.emplace(actor->unique_id_, actor);

    return actor->unique_id_;
  }

  size_t World::loadUrdf(const std::string& path,
                         const std::array<double, 3>& position,
                         const std::array<double, 3>& orientation,
                         bool from_base_path)
  {
    auto urdf_model = getUrdf(path, from_base_path);
    return loadUrdf(urdf_model, position, orientation);
  }

  size_t World::loadUrdfRaw(const std::string& content,
                            const std::array<double, 3>& position,
                            const std::array<double, 3>& orientation)
  {
    auto urdf_model = getUrdfRaw(content);
    return loadUrdf(urdf_model, position, orientation);
  }

  size_t World::loadUrdf(urdf::Urdf_t& urdf_model,
                         const std::array<double, 3>& position,
                         const std::array<double, 3>& orientation)
  {
    auto* urdf = loadUrdf(urdf_model);

    loadUrdfLink(urdf, urdf_model, "", urdf_model.root_link, position, orientation);

    for(const auto& joint : urdf_model.joints)
      urdf->addJoint(joint.second);

    insertUrdf(urdf);

    urdf->name_ = urdf_model.name;
    urdfs_[urdf->unique_id_] = urdf;

    return urdf->unique_id_;
  }

  int World::getNumJoints(size_t urdf_id) const
  {
    auto it = urdfs_.find(urdf_id);
    if(it == urdfs_.end())
      return -1;
    else
      return it->second->getNumJoints();
  }

  std::pair<std::array<double, 3>, std::array<double, 4>> World::getBasePositionAndOrientation(int body_id) const
  {
    auto urdf_it = urdfs_.find(body_id);
    if(urdf_it != urdfs_.end())
      return urdf_it->second->getPositionAndOrientation();
    else
    {
      auto actor_it = actors_.find(body_id);
      if(actor_it != actors_.end())
        return actor_it->second->getPositionAndOrientation();
      else
        return std::pair<std::array<double, 3>, std::array<double, 4>>{
          {0., 0., 0.},
          {0., 0., 0., 0.}
        };
    }
  }

  void World::setBasePositionAndOrientation(int body_id, const std::array<double, 3>& position, const std::array<double, 4>& orientation)
  {
    auto urdf_it = urdfs_.find(body_id);
    if(urdf_it != urdfs_.end())
      urdf_it->second->setPositionAndOrientation(position, orientation);
    else
    {
      auto actor_it = actors_.find(body_id);
      if(actor_it != actors_.end())
        actor_it->second->setPositionAndOrientation(position, orientation);
    }
  }

  void World::setBaseVelocity(int body_id, const std::array<double, 3>& linear_velocity, const std::array<double, 3>& angular_velocity)
  {
    auto urdf_it = urdfs_.find(body_id);
    if(urdf_it != urdfs_.end())
      urdf_it->second->setVelocity(linear_velocity, angular_velocity);
    else
    {
      auto actor_it = actors_.find(body_id);
      if(actor_it != actors_.end())
        actor_it->second->setVelocity(linear_velocity, angular_velocity);
    }
  }

  bool World::setJointState(int body_id, const std::string& joint_name, double position, double velocity)
  {
    auto urdf_it = urdfs_.find(body_id);
    if(urdf_it != urdfs_.end())
      return urdf_it->second->setJointState(joint_name, position, velocity);
    else
      return false;
  }

  int World::getLinkId(int body_id, const std::string& link_name)
  {
    auto urdf_it = urdfs_.find(body_id);
    if(urdf_it != urdfs_.end())
      return urdf_it->second->getLinkId(link_name);
    else
      return -1;
  }

  void World::setMass(int body_id, int link_index, double mass_kg)
  {
    auto urdf_it = urdfs_.find(body_id);
    if(urdf_it != urdfs_.end())
      urdf_it->second->setMass(link_index, mass_kg);
    else
    {
      auto actor_it = actors_.find(body_id);
      if(actor_it != actors_.end())
        actor_it->second->setMass((float)mass_kg);
    }
  }

  void World::setStaticFriction(int body_id, int link_index, double friction)
  {
    auto urdf_it = urdfs_.find(body_id);
    if(urdf_it != urdfs_.end())
      urdf_it->second->setStaticFriction(link_index, friction);
    else
    {
      auto actor_it = actors_.find(body_id);
      if(actor_it != actors_.end())
        actor_it->second->setStaticFriction((float)friction);
    }
  }

  void World::setDynamicFriction(int body_id, int link_index, double friction)
  {
    auto urdf_it = urdfs_.find(body_id);
    if(urdf_it != urdfs_.end())
      urdf_it->second->setDynamicFriction(link_index, friction);
    else
    {
      auto actor_it = actors_.find(body_id);
      if(actor_it != actors_.end())
        actor_it->second->setDynamicFriction((float)friction);
    }
  }

  void World::setRestitution(int body_id, int link_index, double restitution)
  {
    auto urdf_it = urdfs_.find(body_id);
    if(urdf_it != urdfs_.end())
      urdf_it->second->setRestitution(link_index, restitution);
    else
    {
      auto actor_it = actors_.find(body_id);
      if(actor_it != actors_.end())
        actor_it->second->setRestitution((float)restitution);
    }
  }

  void World::setSimulation(int body_id, bool activate)
  {
    auto urdf_it = urdfs_.find(body_id);
    if(urdf_it != urdfs_.end())
      return;
    else
    {
      auto actor_it = actors_.find(body_id);
      if(actor_it != actors_.end())
        actor_it->second->setSimulationEnabled(activate);
    }
  }

  void World::setPhysics(int body_id, bool activate)
  {
    auto urdf_it = urdfs_.find(body_id);
    if(urdf_it != urdfs_.end())
      return;
    else
    {
      auto actor_it = actors_.find(body_id);
      if(actor_it != actors_.end())
        actor_it->second->setPhysicsEnabled(activate);
    }
  }

  /* COLISIONS */

  std::vector<RaycastHitResult_t> World::raycasts(const std::vector<std::array<double, 3>>& origins,
                                                  const std::vector<std::array<double, 3>>& destinations,
                                                  float max_distance)
  {
    std::vector<RaycastHitResult_t> results;

    performRaycastsInParallel(origins, destinations, max_distance, results);

    return results;
  }

  AABB_t World::getAABB(int body_id, int link_index)
  {
    auto urdf_it = urdfs_.find(body_id);
    if(urdf_it != urdfs_.end())
      return urdf_it->second->getAABB(link_index);
    else
    {
      auto actor_it = actors_.find(body_id);
      if(actor_it != actors_.end())
        return actor_it->second->getAABB();
      else
        return AABB_t();
    }
  }

  AABB_t World::getLocalAABB(int body_id, int link_index)
  {
    auto urdf_it = urdfs_.find(body_id);
    if(urdf_it != urdfs_.end())
      return urdf_it->second->getLocalAABB(link_index);
    else
    {
      auto actor_it = actors_.find(body_id);
      if(actor_it != actors_.end())
        return actor_it->second->getLocalAABB();
      else
        return AABB_t();
    }
  }

  std::unordered_set<int> World::getOverlappingObjects(int body_id, int link_index)
  {
    Actor* actor = getActor(body_id, link_index);

    if(actor == nullptr)
      return {};

    return getOverlappingActors(actor);
  }

  /* LIGHTS */

  void World::setAmbientLight(const std::array<float, 3>& lat_long_alt,
                              const std::array<float, 3>& color,
                              float ambient_strength,
                              float diffuse_strength,
                              float specular_strength)
  {
    ambient_light_ = AmbientLight(lat_long_alt,
                                  glm::vec3(color[0], color[1], color[2]),
                                  ambient_strength, diffuse_strength, specular_strength);
  }

  void World::setAmbientLightDirection(const std::array<float, 3>& direction)
  {
    ambient_light_.setDirection(glm::vec3(direction[0], direction[1], direction[2]));
  }

  void World::setAmbientLightColor(const std::array<float, 3>& color)
  {
    ambient_light_.setColor(glm::vec3(color[0], color[1], color[2]));
  }

  void World::setAmbientLightAmbientStrength(float ambient_strength)
  {
    ambient_light_.setAmbientStrength(ambient_strength);
  }

  std::size_t World::addPointLight(const std::array<float, 3>& position,
                                   const std::array<float, 3>& color,
                                   float ambient_strength,
                                   float diffuse_strength,
                                   float specular_strength,
                                   float attenuation_radius)
  {
    return point_lights_.addLight(glm::vec3(position[0], position[1], position[2]),
                                  glm::vec3(color[0], color[1], color[2]),
                                  ambient_strength,
                                  diffuse_strength,
                                  specular_strength,
                                  attenuation_radius);
  }

  void World::removePointLight(std::size_t id)
  {
    point_lights_.removeLight(id);
  }

  void World::setPointLightColor(std::size_t id, const glm::vec3& color)
  {
    point_lights_.setColor(id, glm::vec3(color[0], color[1], color[2]));
  }

  void World::setPointLightPosition(std::size_t id, const glm::vec3& position)
  {
    point_lights_.setPosition(id, glm::vec3(position[0], position[1], position[2]));
  }

  void World::setPointLightAmbientStrength(std::size_t id, float strength)
  {
    point_lights_.setAmbientStrength(id, strength);
  }

  /* DEBUG */

  int World::addDebugText(const std::string& text,
                          const std::array<float, 3>& position,
                          float height,
                          const std::array<float, 3>& color,
                          bool centered,
                          double life_time,
                          int replace_id,
                          int body_id,
                          int link_index)
  {
    DebugText_t debug{
      text,
      centered,
      ToV3(position),
      ToV3(color),
      height,
      life_time,
      getActor(body_id, link_index)};

    if(replace_id >= 0)
    {
      if(replace_id < (int)debug_texts_.size())
      {
        debug_texts_[replace_id] = debug;
        return replace_id;
      }
      else
        return -1;
    }
    else
    {
      int id = -1;
      for(size_t i = 0; i < debug_texts_.size(); i++)
      {
        if(debug_texts_[i].text.empty())
        {
          id = i;
          break;
        }
      }

      if(id >= 0)
        debug_texts_[id] = debug;
      else
      {
        id = (int)debug_texts_.size();
        debug_texts_.emplace_back(debug);
      }
      return id;
    }
  }

  void World::removeDebugText(int id)
  {
    if(id >= 0 && id < (int)debug_texts_.size())
      debug_texts_[id].text = "";
  }

  int World::addDebugLine(const std::array<float, 3>& position_from,
                          const std::array<float, 3>& position_to,
                          const std::array<float, 3>& color,
                          double life_time,
                          int replace_id,
                          int body_id,
                          int link_index)
  {
    return addDebugLine({position_from, position_to},
                        {0, 1},
                        color, life_time, replace_id,
                        body_id, link_index);
  }

  int World::addDebugLine(const std::vector<std::array<float, 3>>& vertices,
                          const std::vector<unsigned int>& indices,
                          const std::array<float, 3>& color,
                          double life_time,
                          int replace_id,
                          int body_id,
                          int link_index)
  {
    std::vector<glm::vec3> glm_vertices;
    glm_vertices.reserve(vertices.size());
    for(const auto& vertex : vertices)
      glm_vertices.emplace_back(ToV3(vertex));

    DebugLine debug{
      glm_vertices,
      indices,
      ToV3(color),
      life_time,
      getActor(body_id, link_index)};

    if(replace_id >= 0)
    {
      if(replace_id < (int)debug_lines_.size())
      {
        debug_lines_.at(replace_id) = std::move(debug);
        return replace_id;
      }
      else
        return -1;
    }
    else
    {
      int id = -1;
      for(size_t i = 0; i < debug_lines_.size(); i++)
      {
        if(debug_lines_[i].indices_.empty())
        {
          id = i;
          break;
        }
      }

      if(id >= 0)
        debug_lines_.at(id) = std::move(debug);
      else
      {
        id = debug_lines_.size();
        debug_lines_.emplace_back(debug);
      }
      return id;
    }
  }

  void World::removeDebugLine(int id)
  {
    if(id >= 0 && id < (int)debug_lines_.size())
      debug_lines_[id].indices_.clear();
  }

  void World::processDebugLifeTime(double delta)
  {
    std::vector<int> lines;
    for(size_t i = 0; i < debug_lines_.size(); i++)
    {
      if(debug_lines_[i].remaining_time_ > 0)
      {
        debug_lines_[i].remaining_time_ -= delta;
        if(debug_lines_[i].remaining_time_ <= 0)
          lines.push_back(i);
      }
    }

    for(auto id : lines)
      removeDebugLine(id);

    std::vector<int> texts;
    for(size_t i = 0; i < debug_texts_.size(); i++)
    {
      if(debug_texts_[i].remaining_time > 0)
      {
        debug_texts_[i].remaining_time -= delta;
        if(debug_texts_[i].remaining_time <= 0)
          texts.push_back(i);
      }
    }

    for(auto id : texts)
      removeDebugText(id);
  }

  /* CAMERAS */

  int World::addCamera(unsigned int width, unsigned int height, float fov, owds::CameraView_e view_type, float near_plane, float far_plane)
  {
    cameras_.emplace_back(width, height, fov, view_type, near_plane, far_plane);
    return (int)cameras_.size() - 1;
  }

  bool World::setCameraPositionAndLookAt(int id, const std::array<double, 3>& eye_position, const std::array<double, 3>& dst_position)
  {
    if(id < (int)cameras_.size())
    {
      cameras_[id].setPositionAndLookAt(eye_position, dst_position);
      return true;
    }
    else
      return false;
  }

  bool World::setCameraPositionAndDirection(int id, const std::array<double, 3>& eye_position, const std::array<double, 3>& eye_direction)
  {
    if(id < (int)cameras_.size())
    {
      cameras_[id].setPositionAndDirection(eye_position, eye_direction);
      return true;
    }
    else
      return false;
  }

  bool World::setCameraPositionAndOrientation(int id, const std::array<double, 3>& eye_position, const std::array<double, 4>& orientation)
  {
    if(id < (int)cameras_.size())
    {
      cameras_[id].setPositionAndOrientation(eye_position, orientation);
      return true;
    }
    else
      return false;
  }

  void World::requestCameraRender(const std::vector<int>& ids)
  {
    for(auto id : ids)
    {
      if(id < (int)cameras_.size())
        cameras_[id].requestUpdate();
    }
    has_render_request_ = true;

    while(has_render_request_ == true)
      usleep(1000);
  }

  void World::getCameraImage(int id, uint32_t** image, unsigned int& width, unsigned int& height)
  {
    if(id < (int)cameras_.size())
    {
      *image = *cameras_[id].getImageData();
      width = cameras_[id].getWidth();
      height = cameras_[id].getHeight();
    }
    else
      image = nullptr;
  }

  std::unordered_set<uint32_t> World::getCameraSementation(int id)
  {
    if(id < (int)cameras_.size())
      return cameras_[id].getSegmentedIds();
    else
      return {};
  }

  /* PROTECTED */

  Actor* World::getActor(int body_id, int link_index)
  {
    Actor* actor = nullptr;
    auto urdf_it = urdfs_.find(body_id);
    if(urdf_it != urdfs_.end())
    {
      auto link_it = urdf_it->second->id_links_.find(link_index);
      if(link_it != urdf_it->second->id_links_.end())
        actor = static_cast<Actor*>(link_it->second);
    }
    else
    {
      auto actor_it = actors_.find(body_id);
      if(actor_it != actors_.end())
        actor = static_cast<Actor*>(actor_it->second);
    }

    return actor;
  }

  /* PRIVATE */

  urdf::Urdf_t World::getUrdf(const std::string& path, bool from_base_path)
  {
    UrdfLoader loader;
    urdf::Urdf_t urdf_model;

    if(from_base_path)
      urdf_model = loader.read((base_assets_path_ / path).string());
    else
      urdf_model = loader.read(path);

    return urdf_model;
  }

  urdf::Urdf_t World::getUrdfRaw(const std::string& content)
  {
    UrdfLoader loader;
    return loader.readRaw(content);;
  }

  void World::loadUrdfLink(owds::Urdf* urdf, const urdf::Urdf_t& model,
                           const std::string& parent,
                           const std::string& link_name,
                           const std::array<double, 3>& position,
                           const std::array<double, 3>& orientation)
  {
    const auto& link = model.links.at(link_name);
    owds::Shape visual_shape = convertShape(link.visual);
    std::vector<owds::Shape> collision_shapes;
    collision_shapes.reserve(link.collisions.size());
    for(const auto& geometry : link.collisions)
      collision_shapes.emplace_back(convertShape(geometry));
    if(collision_shapes.empty())
      collision_shapes.emplace_back(ShapeDummy());

    if(parent.empty())
      urdf->addLink(parent, link_name, ToV3(position), ToV3(orientation), collision_shapes.front(), {visual_shape});
    else
    {
      std::string joint_name = model.link_to_parent_joint.at(link_name);
      urdf::Joint_t joint = model.joints.at(joint_name);
      urdf->addLink(parent, link_name, joint.origin_translation, joint.origin_rotation, collision_shapes.front(), {visual_shape});
    }

    auto childs_it = model.tree.find(link_name);
    if(childs_it != model.tree.end())
    {
      for(const auto& child : childs_it->second)
      {
        const auto& child_joint = model.joints.at(child);
        loadUrdfLink(urdf, model, link_name, child_joint.child_link, position, orientation);
      }
    }
  }

  owds::Shape World::createShapeBox(const owds::Color& color, const std::array<float, 3>& half_extents, glm::mat4& transform)
  {
    return owds::ShapeBox{
      half_extents,
      color,
      preloaded_box_model_,
      transform};
  }

  owds::Shape World::createShapeCapsule(const owds::Color& color, const float radius, const float height, glm::mat4& transform)
  {
    return owds::ShapeCapsule{
      radius,
      height,
      color,
      preloaded_cylinder_model_,
      preloaded_sphere_model_,
      transform};
  }

  owds::Shape World::createShapeCylinder(const owds::Color& color, const float radius, const float height, glm::mat4& transform)
  {
    return owds::ShapeCylinder{
      radius,
      height,
      color,
      preloaded_cylinder_model_,
      transform};
  }

  owds::Shape World::createShapeSphere(const owds::Color& color, const float radius, glm::mat4& transform)
  {
    return owds::ShapeSphere{
      radius,
      color,
      preloaded_sphere_model_,
      transform};
  }

  owds::Shape World::createShapeFromModel(const owds::Material& material, const std::string& path, const std::array<float, 3>& scale, glm::mat4& transform)
  {
    return owds::ShapeCustomMesh{
      glm::vec3(scale[0], scale[1], scale[2]),
      material,
      owds::ModelManager::get().load(path),
      transform};
  }

  owds::Shape World::convertShape(const urdf::Geometry_t& geometry)
  {
    glm::mat4 translate = glm::translate(glm::mat4(1), geometry.origin_translation);
    glm::mat4 rotate = glm::mat4_cast(glm::quat(geometry.origin_rotation));
    glm::mat4 transform = translate * rotate;
    return convertShape(geometry, transform);
  }

  owds::Shape World::convertShape(const urdf::Geometry_t& urdf_shape, glm::mat4& transform)
  {
    switch(urdf_shape.type)
    {
    case urdf::GeometryType_e::geometry_sphere:
    {
      return createShapeSphere(urdf_shape.material.diffuse_color_,
                               static_cast<float>(urdf_shape.scale.x),
                               transform);
    }
    case urdf::GeometryType_e::geometry_box:
    {
      return createShapeBox(urdf_shape.material.diffuse_color_,
                            {static_cast<float>(urdf_shape.scale.x / 2.),
                             static_cast<float>(urdf_shape.scale.y / 2.),
                             static_cast<float>(urdf_shape.scale.z / 2.)},
                            transform);
    }
    case urdf::GeometryType_e::geometry_cylinder:
    {
      return createShapeCylinder(urdf_shape.material.diffuse_color_,
                                 static_cast<float>(urdf_shape.scale.x),
                                 static_cast<float>(urdf_shape.scale.y),
                                 transform);
    }
    case urdf::GeometryType_e::geometry_mesh:
    {
      return createShapeFromModel(urdf_shape.material,
                                  owds::getFullPath(urdf_shape.file_name),
                                  {static_cast<float>(urdf_shape.scale.x),
                                   static_cast<float>(urdf_shape.scale.y),
                                   static_cast<float>(urdf_shape.scale.z)},
                                  transform);
    }
    default:
      return ShapeDummy();
    }
  }

} // namespace owds