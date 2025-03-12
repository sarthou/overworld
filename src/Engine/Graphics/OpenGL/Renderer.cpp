#include "overworld/Engine/Graphics/OpenGL/Renderer.h"

#include <cstddef>
#include <cstdint>
#include <glm/ext/matrix_float3x3.hpp>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/vector_float3.hpp>
#include <iostream>
#include <set>
#include <string>
#include <unistd.h>
#include <variant>
#include <vector>

#include "glad/glad.h"
#include "overworld/Engine/Common/Camera/Camera.h"
#include "overworld/Engine/Common/Camera/CameraView.h"
#include "overworld/Engine/Common/Camera/ViewAntiAliasing.h"
#include "overworld/Engine/Common/Debug/DebugLine.h"
#include "overworld/Engine/Common/Lights/AmbientLight.h"
#include "overworld/Engine/Common/Lights/PointLights.h"
#include "overworld/Engine/Common/Models/Mesh.h"
#include "overworld/Engine/Common/Models/Model.h"
#include "overworld/Engine/Common/Shapes/Shape.h"
#include "overworld/Engine/Common/Urdf/Actor.h"
#include "overworld/Engine/Common/World.h"
#include "overworld/Engine/Graphics/Common/InstanceData.h"
#include "overworld/Engine/Graphics/GLFW/Window.h"
#include "overworld/Engine/Graphics/OpenGL/Cubemap.h"
#include "overworld/Engine/Graphics/OpenGL/MeshHandle.h"
#include "overworld/Engine/Graphics/OpenGL/Texture2D.h"
#include "overworld/Utils/RosPackage.h"
#include "overworld/Utils/GlmMath.h"

// should be after glad
#include <GLFW/glfw3.h>

void GLAPIENTRY
MessageCallback(GLenum source,
                GLenum type,
                GLuint id,
                GLenum severity,
                GLsizei length,
                const GLchar* message,
                const void* userParam)
{
  (void)source;
  (void)id;
  (void)length;
  (void)userParam;
  if(severity == GL_DEBUG_SEVERITY_HIGH /* || (severity == GL_DEBUG_SEVERITY_MEDIUM)*/)
    fprintf(stderr, "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s\n",
            (type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""),
            type, severity, message);
}

namespace owds {

  Renderer::~Renderer()
  {}

  void Renderer::init()
  {
    owds::Window::init();
    owds::Window tmp;
    tmp.makeCurrentContext();

    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
      std::cout << "Failed to initialize GLAD" << std::endl;
    }

    glfwMakeContextCurrent(nullptr);
  }

  void Renderer::release()
  {
    owds::Window::release();
  }

  bool Renderer::initialize(Window& window)
  {
    window.makeCurrentContext();

    setRenderCamera(window.getUpdatedCamera());

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_STENCIL_TEST);
    // glEnable(GL_BLEND);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    // glFrontFace(GL_CW);

    glEnable(GL_DEBUG_OUTPUT);
    glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    glDebugMessageCallback(MessageCallback, nullptr);
    glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);

    glEnable(GL_MULTISAMPLE);
    screen_.init();
    screen_.setSize((unsigned int)render_camera_.getWidth(), (unsigned int)render_camera_.getHeight());
    if(render_camera_.getAASetting() != ViewAntiAliasing_e::off)
      setAntiAliasing(render_camera_.getAASetting());

    std::string owds_path = findPackage("overworld");
    Shader::shaders_directory = owds_path + "/shaders/";
    shaders_.insert({
      "default", {"light_shader.vs", "light_shader.fs"}
    });
    shaders_.insert({
      "sky", {"sky_shader.vs", "sky_shader.fs"}
    });
    shaders_.insert({
      "screen", {"screen_shader.vs", "screen_shader.fs"}
    });
    shaders_.insert({
      "depth", {"depth_shader.vs", "depth_shader.fs", "depth_shader.gs"}
    });
    shaders_.insert({
      "depthcube", {"depthcube_shader.vs", "depthcube_shader.fs", "depthcube_shader.gs"}
    });
    shaders_.insert({
      "text", {"text_shader.vs", "text_shader.fs"}
    });
    shaders_.insert({
      "color", {"color_shader.vs", "color_shader.fs"}
    });

    sky_.init(owds_path + "/models/textures/skybox/Footballfield/");

    shadow_.init(render_camera_.getNearPlane(), render_camera_.getFarPlane());
    point_shadows_.init();
    text_renderer_.init();
    text_renderer_.load("/usr/share/fonts/truetype/open-sans/OpenSans-Regular.ttf", 48);

    debug_axis_.emplace(0,
                        DebugLine({glm::vec3(0., 0., 0.),
                                   glm::vec3(1., 0., 0.)},
                                  {0, 1},
                                  glm::vec3(1., 0., 0.)));
    debug_axis_.emplace(1,
                        DebugLine({glm::vec3(0., 0., 0.),
                                   glm::vec3(0., 1., 0.)},
                                  {0, 1},
                                  glm::vec3(0., 1., 0.)));
    debug_axis_.emplace(2,
                        DebugLine({glm::vec3(0., 0., 0.),
                                   glm::vec3(0., 0., 1.)},
                                  {0, 1},
                                  glm::vec3(0., 0., 1.)));

    shaders_.at("screen").use();
    shaders_.at("screen").setInt("screenTexture", 0);

    render_steps_.emplace_back([this](){ loadWorld(); });
    render_steps_.emplace_back([this](){ renderPointShadowDepth(); });
    render_steps_.emplace_back([this](){ loadDebugLines(); });
    render_steps_.emplace_back([this](){ renderAmbientShadowDepth(); });
    render_steps_.emplace_back([this](){ renderMainScreen(); });

    return true;
  }

  void Renderer::commit()
  {
    if(world_ == nullptr)
      return;

    float current_frame = glfwGetTime();
    if(last_frame_ <= 0)
      last_frame_ = current_frame;
    delta_time_ = current_frame - last_frame_;
    last_frame_ = current_frame;

    world_->processDebugLifeTime((double)delta_time_);

    float sleep_time = 1.0f / max_fps_ - delta_time_;
    if(sleep_time > 0.0)
      sleep(sleep_time);

    for(auto& model : current_mesh_batches_)
      for(auto& mesh : model.second)
        mesh.second.clear();

    render_collision_models_ = render_camera_.shouldRendercollisionModels();
    render_shadows_ = render_camera_.shouldShadows();

    for(const auto& step : render_steps_)
    {
      step();
      renderOffScreens();
    }
    glFinish();
  }

  void Renderer::setRenderCamera(Camera* camera)
  {
    render_camera_ = *camera;
    if(render_camera_.sizeHasChanged())
    {
      screen_.setSize(render_camera_.getWidth(), render_camera_.getHeight());
      glViewport(0, 0, render_camera_.getWidth(), render_camera_.getHeight());
      screen_.reinitBuffers();
    }
  }

  void Renderer::loadWorld()
  {
    for(const auto& actor : world_->getActors())
    {
      if(render_collision_models_)
      {
        std::visit([this, actor](const auto& shape_resolv) { loadActor(actor.second, shape_resolv, true); }, actor.second->collision_shape_);
      }
      else
      {
        for(const auto& shape : actor.second->visual_shapes_)
        {
          std::visit([this, actor](const auto& shape_resolv) { loadActor(actor.second, shape_resolv); }, shape);
        }
      }
    }

    for(size_t i = 0; i < PointLights::MAX_POINT_LIGHTS; i++)
    {
      if(world_->point_lights_.isUsed(i))
      {
        double dist = world_->point_lights_.getAttenuationDistance(i);
        auto pose = world_->point_lights_.getPosition(i);
        auto color = world_->point_lights_.getDiffuse(i);
        if(debug_lights_.find(i) == debug_lights_.end())
        {
          debug_lights_.emplace(i,
                                DebugLine({glm::vec3(pose.x, pose.y, pose.z),
                                           glm::vec3(pose.x, pose.y, pose.z - dist)},
                                           {0, 1},
                                           glm::vec3(color.x, color.y, color.z)));
        }
      }
      else
        debug_lights_.erase(i);
    }
  }

  void Renderer::loadActor(Actor* actor, const ShapeBox& shape, bool default_material)
  {
    const auto size_mat = glm::scale(glm::mat4(1.f), ToV3(shape.half_extents_));
    const auto model_mat = shape.shape_transform_ * ToM4(actor->getModelMatrix()) * size_mat;

    if(default_material == false)
      loadInstance(shape.box_model_, {"", shape.diffuse_color_, shape.diffuse_color_, 0., "", "", ""}, model_mat, actor->unique_id_);
    else
      loadInstance(shape.box_model_, createColisionMaterial(actor->unique_id_), model_mat, actor->unique_id_);
  }

  void Renderer::loadActor(Actor* actor, const ShapeCapsule& shape, bool default_material)
  {
    (void)actor; // todo
    (void)shape; // todo
    (void)default_material;
  }

  void Renderer::loadActor(Actor* actor, const ShapeCustomMesh& shape, bool default_material)
  {
    const auto size_mat = glm::scale(glm::mat4(1.f), shape.scale_);
    const auto model_mat = ToM4(actor->getModelMatrix()) * shape.shape_transform_ * size_mat;

    if(default_material == false)
      loadInstance(shape.custom_model_, shape.material_, model_mat, actor->unique_id_);
    else
      loadInstance(shape.custom_model_, createColisionMaterial(actor->unique_id_), model_mat, actor->unique_id_);
  }

  void Renderer::loadActor(Actor* actor, const ShapeCylinder& shape, bool default_material)
  {
    const auto size_mat = glm::scale(glm::mat4(1.f), glm::vec3(shape.radius_, shape.height_, shape.radius_));
    const auto model_mat = ToM4(actor->getModelMatrix()) * shape.shape_transform_ * size_mat;

    if(default_material == false)
      loadInstance(shape.cylinder_model_, {"", shape.diffuse_color_, shape.diffuse_color_, 0., "", "", ""}, model_mat, actor->unique_id_);
    else
      loadInstance(shape.cylinder_model_, createColisionMaterial(actor->unique_id_), model_mat, actor->unique_id_);
  }

  void Renderer::loadActor(Actor* actor, const ShapeDummy& shape, bool default_material)
  {
    (void)actor;
    (void)shape;
    (void)default_material;
  }

  void Renderer::loadActor(Actor* actor, const ShapeSphere& shape, bool default_material)
  {
    (void)actor; // todo
    (void)shape; // todo
    (void)default_material;
  }

  void Renderer::loadInstance(const Model& model, const Material& material, const glm::mat4& model_mat, uint32_t object_id)
  {
    loadModel(model, material);

    for(const auto& mesh : model.meshes_)
      current_mesh_batches_[model.id_][mesh.id_].emplace_back(InstanceData{model_mat, {}, object_id}); // TODO add instance data ?
  }

  Material Renderer::combineMaterials(const Material& shape_material, const Material& model_material)
  {
    Material material;

    if(model_material.diffuse_texture_.empty())
      material.diffuse_texture_ = shape_material.diffuse_texture_;
    else
      material.diffuse_texture_ = model_material.diffuse_texture_;

    if(model_material.specular_texture_.empty())
      material.specular_texture_ = shape_material.specular_texture_;
    else
      material.specular_texture_ = model_material.specular_texture_;

    if(model_material.normal_texture_.empty())
      material.normal_texture_ = shape_material.normal_texture_;
    else
      material.normal_texture_ = model_material.normal_texture_;

    if(model_material.diffuse_color_.a_ == 0.f)
      material.diffuse_color_ = shape_material.diffuse_color_;
    else
      material.diffuse_color_ = model_material.diffuse_color_;

    if(model_material.specular_color_.a_ <= 0.001f)
      material.specular_color_ = shape_material.specular_color_;
    else
      material.specular_color_ = model_material.specular_color_;

    if(model_material.shininess_ == -1.f)
      material.shininess_ = shape_material.shininess_;
    else
      material.shininess_ = shape_material.shininess_;

    return material;
  }

  std::vector<Texture2D> Renderer::loadTextures(Material& material)
  {
    std::vector<Texture2D> textures;

    if(material.diffuse_texture_.empty() == false)
    {
      auto text_it = loaded_textures_.find(material.diffuse_texture_);
      if(text_it == loaded_textures_.end())
      {
        text_it = loaded_textures_.insert({
                                            material.diffuse_texture_, {material.diffuse_texture_, texture_diffuse, true, true}
        })
                    .first;
      }
      textures.emplace_back(text_it->second);
    }

    if(material.specular_texture_.empty() == false)
    {
      auto text_it = loaded_textures_.find(material.specular_texture_);
      if(text_it == loaded_textures_.end())
      {
        text_it = loaded_textures_.insert({
                                            material.specular_texture_, {material.specular_texture_, texture_specular, false, true}
        })
                    .first;
      }
      textures.emplace_back(text_it->second);
    }

    if(material.normal_texture_.empty() == false)
    {
      auto text_it = loaded_textures_.find(material.normal_texture_);
      if(text_it == loaded_textures_.end())
      {
        text_it = loaded_textures_.insert({
                                            material.normal_texture_, {material.normal_texture_, texture_normal, false, true}
        })
                    .first;
      }
      textures.emplace_back(text_it->second);
    }

    return textures;
  }

  void Renderer::loadModel(const Model& model, const Material& material)
  {
    if(model.meshes_.empty())
      return;

    auto model_it = cached_models_.find(model.id_);
    if(model_it != cached_models_.end())
      return;

    model_it = cached_models_.insert({model.id_, {}}).first;

    for(const auto& mesh : model.meshes_)
    {
      auto mesh_material = combineMaterials(mesh.material_, material);
      auto textures = loadTextures(mesh_material);
      auto mesh_it = model_it->second.insert({
                                               mesh.id_, {mesh, textures}
      })
                       .first;

      mesh_it->second.color = mesh_material.diffuse_color_;
      mesh_it->second.color.a_ = 1.0;
      mesh_it->second.shininess = mesh_material.shininess_ <= 0 ? 64.f : mesh_material.shininess_;
      mesh_it->second.specular = mesh_material.specular_color_.a_ == 0. ? 0.1f : mesh_material.specular_color_.r_;
    }
  }

  void Renderer::loadDebugLines()
  {
    std::set<unsigned int> ids;
    for(auto& cached : cached_lines_)
      ids.insert(cached.first);

    for(size_t i = 0; i < world_->debug_lines_.size(); i++)
    {
      if(world_->debug_lines_[i].indices_.empty())
      {
        cached_lines_.erase(world_->debug_lines_[i].id_);
        continue;
      }

      auto cache_it = cached_lines_.find(world_->debug_lines_[i].id_);
      if(cache_it == cached_lines_.end())
        cached_lines_.emplace(world_->debug_lines_[i].id_, world_->debug_lines_[i]);

      ids.erase(world_->debug_lines_[i].id_);
    }

    for(auto id : ids)
      cached_lines_.erase(id);
  }

  void Renderer::renderMainScreen()
  {
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glDisable(GL_BLEND);

    auto& light_shader = shaders_.at("default");
    auto& sky_shader = shaders_.at("sky");

    // 1. draw scene as normal in multisampled buffers

    screen_.bindFrameBuffer();
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    // glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glEnable(GL_FRAMEBUFFER_SRGB);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    // glStencilMask(0x00);

    light_shader.use();
    setLightsUniforms(light_shader, render_shadows_, render_shadows_);
    light_shader.setVec3("view_pose", render_camera_.getPosition());
    light_shader.setMat4("view", render_camera_.getViewMatrix());
    light_shader.setMat4("projection", render_camera_.getProjectionMatrix());

    shadow_.setUniforms(light_shader, 1);
    shadow_.setLightMatrices();

    for(size_t i = 0; i < PointLights::MAX_POINT_LIGHTS; i++)
      if(world_->point_lights_.isUsed(i))
        point_shadows_.setUniforms(i, light_shader, 6);

    renderModels(light_shader, 2);

    // 1.2 draw background

    sky_shader.use();
    glm::mat4 view = glm::mat4(glm::mat3(render_camera_.getViewMatrix()));
    sky_shader.setMat4("view", view);
    sky_shader.setMat4("projection", render_camera_.getProjectionMatrix());
    sky_shader.setVec4("color", world_->ambient_light_.getColor());

    sky_.draw(sky_shader);

    if(render_camera_.shouldRenderDebug())
      renderDebug();

    // 2. now blit multisampled buffer(s) to normal colorbuffer of intermediate FBO. Image is stored in screenTexture
    screen_.generateColorTexture();

    // 3. now render quad with scene's visuals as its texture image
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_FRAMEBUFFER_SRGB);

    // draw Screen quad
    shaders_.at("screen").use();
    screen_.draw();
  }

  void Renderer::renderOffScreens()
  {
    if(world_->has_render_request_)
    {
      for(size_t i = 0; i < world_->cameras_.size(); i++)
      {
        auto& virtual_camera = world_->cameras_[i];
        if(i >= off_screens_.size())
        {
          off_screens_.emplace_back();
          off_screens_.back().init(virtual_camera.getWidth(), virtual_camera.getHeight());
        }

        off_screens_.at(i).bindFrameBuffer();

        Camera* camera = virtual_camera.getCamera();
        if(camera->getViewType() == CameraView_e::regular_view)
          renderOffscreenRgb(camera);
        else
          renderOffscreenSegmented(camera);

        //glFlush();
        off_screens_[i].getImage(virtual_camera.getImageData());
      }
      world_->has_render_request_ = false;
    }
  }

  void Renderer::renderOffscreenRgb(Camera* camera)
  {
    auto& light_shader = shaders_.at("default");
    auto& sky_shader = shaders_.at("sky");

    // 1. draw scene as normal in multisampled buffers
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_FRAMEBUFFER_SRGB);
    glDisable(GL_BLEND);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    light_shader.use();
    setLightsUniforms(light_shader, false, render_shadows_); // We never use the ambient shadows for offscreen
    light_shader.setVec3("view_pose", camera->getPosition());
    light_shader.setMat4("view", camera->getViewMatrix());
    light_shader.setMat4("projection", camera->getProjectionMatrix());

    shadow_.setUniforms(light_shader, 1);
    shadow_.setLightMatrices();

    for(size_t i = 0; i < PointLights::MAX_POINT_LIGHTS; i++)
      if(world_->point_lights_.isUsed(i))
        point_shadows_.setUniforms(i, light_shader, 6);

    renderModels(light_shader, 2);

    // 1.2 draw background

    sky_shader.use();
    glm::mat4 view = glm::mat4(glm::mat3(camera->getViewMatrix()));
    sky_shader.setMat4("view", view);
    sky_shader.setMat4("projection", camera->getProjectionMatrix());
    sky_shader.setVec4("color", world_->ambient_light_.getColor());

    sky_.draw(sky_shader);
  }

  void Renderer::renderOffscreenSegmented(Camera* camera)
  {
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    auto& shader = shaders_.at("color");

    shader.use();
    shader.setMat4("view", camera->getViewMatrix());
    shader.setMat4("projection", camera->getProjectionMatrix());

    renderModelsSegmented(shader);
  }

  void Renderer::renderAmbientShadowDepth()
  {
    if(render_shadows_ == false)
      return;

    auto& shadow_shader = shaders_.at("depth");

    auto ambient_dir = -glm::normalize(world_->ambient_light_.getDirection());
    shadow_.computeLightSpaceMatrices(render_camera_, ambient_dir);

    shadow_shader.use();
    shadow_.setLightMatrices();

    shadow_.bindFrameBuffer();
    glEnable(GL_DEPTH_TEST);

    renderModels(shadow_shader, 2);
  }

  void Renderer::renderPointShadowDepth()
  {
    if(render_shadows_ == false)
      return;

    auto& shadow_point_shader = shaders_.at("depthcube");
    glEnable(GL_DEPTH_TEST);

    shadow_point_shader.use();
    for(size_t i = 0; i < PointLights::MAX_POINT_LIGHTS; i++)
    {
      point_shadows_.bindFrameBuffer(i);
      if(world_->point_lights_.isUsed(i))
      {
        if(point_shadows_.isInit(i) == false)
          point_shadows_.init(i, world_->point_lights_.getAttenuationDistance(i));

        point_shadows_.computeLightTransforms(i, glm::vec3(world_->point_lights_.getPosition(i)));
        
        point_shadows_.setUniforms(i, shadow_point_shader);

        renderModels(shadow_point_shader, 2);
      }
    }
  }

  void Renderer::renderDebug()
  {
    auto& text_shader = shaders_.at("text");
    auto& lines_shader = shaders_.at("color");

    // Draw lines

    lines_shader.use();
    lines_shader.setMat4("view", render_camera_.getViewMatrix());
    lines_shader.setMat4("projection", render_camera_.getProjectionMatrix());
    lines_shader.setMat4("model", glm::mat4(1.));

    for(auto& line_handle : cached_lines_)
    {
      if(line_handle.second.actor == nullptr)
        lines_shader.setMat4("model", glm::mat4(1.));
      else
      {
        auto pose = line_handle.second.actor->getPositionAndOrientation().first;
        lines_shader.setMat4("model", glm::translate(glm::mat4(1), glm::vec3(pose[0], pose[1], pose[2])));
      }
      line_handle.second.draw(lines_shader);
    }

    if(render_camera_.shouldRenderAllDebug())
    {
      glLineWidth(6);
      for(auto& debug_light : debug_lights_)
        debug_light.second.draw(lines_shader);
      glLineWidth(1);
    }

    // Draw debug axis

    glDisable(GL_DEPTH_TEST);
    glLineWidth(3);
    glm::mat4 model = glm::scale(glm::mat4(1), glm::vec3(0.05, 0.05, 0.05));
    glm::mat4 view = glm::mat4(glm::mat3(render_camera_.getViewMatrix()));
    double aspect = (double)screen_.width_ / (double)screen_.height_;
    double trans = tan(render_camera_.getFov() / 2.);
    double trans_x = trans - 0.05;
    double trans_y = trans - 0.05 * aspect;
    glm::mat4 view_translation = glm::translate(glm::mat4(1), glm::vec3(trans_x * aspect, -trans_y, -1.));
    view = view_translation * view;

    lines_shader.setMat4("projection", render_camera_.getProjectionMatrix());
    lines_shader.setMat4("view", view);
    lines_shader.setMat4("model", model);

    debug_axis_.at(0).draw(lines_shader);
    debug_axis_.at(1).draw(lines_shader);
    debug_axis_.at(2).draw(lines_shader);

    glEnable(GL_DEPTH_TEST);
    glLineWidth(1);

    // Draw text

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    text_shader.use();
    text_shader.setMat4("projection", render_camera_.getProjectionMatrix());
    for(auto debug : world_->debug_texts_)
    {
      glm::vec4 actor_pose(0., 0., 0., 0.);
      if(debug.linked_actor != nullptr)
      {
        auto pose = debug.linked_actor->getPositionAndOrientation().first;
        actor_pose = glm::vec4(pose[0], pose[1], pose[2], 0.);
      }

      if(debug.text.empty() == false)
        text_renderer_.renderText(text_shader, render_camera_.getViewMatrix(), actor_pose, debug);
    }

    glDisable(GL_BLEND);
  }

  void Renderer::renderModels(const Shader& shader, unsigned int texture_offset)
  {
    for(auto& [model_id, batch] : current_mesh_batches_)
    {
      auto& meshes = cached_models_.at(model_id);

      for(auto& [mesh_id, transforms] : batch)
      {
        const auto& mesh = meshes.at(mesh_id);

        for(const auto& transform : transforms)
        {
          shader.setMat4("model", transform.mvp_);
          mesh.draw(shader, texture_offset);
        }
      }
    }
  }

  void Renderer::renderModelsSegmented(const Shader& shader)
  {
    for(auto& [model_id, batch] : current_mesh_batches_)
    {
      auto& meshes = cached_models_.at(model_id);

      for(auto& [mesh_id, transforms] : batch)
      {
        const auto& mesh = meshes.at(mesh_id);

        for(const auto& transform : transforms)
        {
          shader.setMat4("model", transform.mvp_);
          mesh.drawId(shader, (uint32_t)transform.object_id_);
        }
      }
    }
  }

  void Renderer::setLightsUniforms(const Shader& shader, bool use_ambient_shadows, bool use_points_shadows)
  {
    AmbientLight& ambient = world_->ambient_light_;

    shader.setVec4("dir_light.ambient", ambient.getAmbient());
    shader.setVec4("dir_light.diffuse", ambient.getDiffuse());
    shader.setVec4("dir_light.specular", ambient.getSpecular());
    shader.setVec4("dir_light.direction", ambient.getDirection());

    shader.setFloat("use_point_shadows", use_points_shadows ? 1.0 : 0.0);
    shader.setFloat("use_ambient_shadows", use_ambient_shadows ? 1.0 : 0.0);

    glActiveTexture(GL_TEXTURE2);
    for(size_t i = 0; i < PointLights::MAX_POINT_LIGHTS; i++)
    {
      shader.setInt("point_lights[" + std::to_string(i) + "].depth_map", 6);
    }
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
    glActiveTexture(GL_TEXTURE0);

    PointLights& points = world_->point_lights_;
    for(size_t i = 0; i < points.getNbLightsFloat(); i++)
    {
      std::string name = "point_lights[" + std::to_string(i) + "]";
      shader.setVec4(name + ".ambient", points.getAmbient(i));
      shader.setVec4(name + ".diffuse", points.getDiffuse(i));
      shader.setVec4(name + ".specular", points.getSpecular(i));
      shader.setVec4(name + ".position", points.getPosition(i));
      shader.setVec4(name + ".attenuation", points.getAttenuation(i));
    }
    shader.setFloat("nb_point_lights", points.getNbLightsFloat());
  }

  void Renderer::setAntiAliasing(ViewAntiAliasing_e setting)
  {
    int samples = 1;
    if(setting == ViewAntiAliasing_e::msaa_x1)
      samples = 1;
    else if(setting == ViewAntiAliasing_e::msaa_x2)
      samples = 2;
    else if(setting == ViewAntiAliasing_e::msaa_x4)
      samples = 4;
    else if(setting == ViewAntiAliasing_e::msaa_x8)
      samples = 8;
    else if(setting == ViewAntiAliasing_e::msaa_x16)
      samples = 16;

    screen_.initBuffers(samples);
  }

  Material Renderer::createColisionMaterial(size_t uid)
  {
    Material material;
    float id = (float)(uid % 60) / 100.f + 0.2;
    material.diffuse_color_ = Color({id, id, id, 1.f});
    material.specular_color_ = Color({id, id, id, 1.f});
    material.shininess_ = 0;

    return material;
  }

} // namespace owds