#include "overworld/Engine/Graphics/OpenGL/Renderer.h"

#include <cstdint>
#include <iostream>
#include <string>
#include <unistd.h>
#include <variant>

#include "glad/glad.h"
#include "overworld/Engine/Common/Lights/AmbientLight.h"
#include "overworld/Engine/Common/Lights/PointLights.h"
#include "overworld/Engine/Common/Models/Mesh.h"
#include "overworld/Engine/Common/Models/Model.h"
#include "overworld/Engine/Common/Shapes/Shape.h"
#include "overworld/Engine/Common/Urdf/Actor.h"
#include "overworld/Engine/Common/World.h"
#include "overworld/Engine/Graphics/Common/InstanceData.h"
#include "overworld/Engine/Graphics/Common/ViewAntiAliasing.h"
#include "overworld/Engine/Graphics/OpenGL/Cubemap.h"
#include "overworld/Engine/Graphics/OpenGL/MeshHandle.h"
#include "overworld/Helper/GlmMath.h"

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
  {
  }

  bool Renderer::initialize(const Window& window)
  {
    (void)window;

    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
      std::cout << "Failed to initialize GLAD" << std::endl;
      return false;
    }

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
    screen_.setSize(render_camera_.view_dimensions_[0], render_camera_.view_dimensions_[1]);
    if(render_camera_.aa_setting_ != ViewAntiAliasing_e::off)
      setAntiAliasing(render_camera_.aa_setting_);

    Shader::shaders_directory = "/home/gsarthou/Robots/Dacobot2/ros2_ws/src/overworld/shaders/";
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

    sky_.init("/home/gsarthou/Robots/Dacobot2/ros2_ws/src/overworld/models/textures/skybox/Footballfield/");

    shadow_.init(render_camera_.getNearPlane(), render_camera_.getFarPlane());

    shaders_.at("screen").use();
    shaders_.at("screen").setInt("screenTexture", 0);

    return true;
  }

  void Renderer::cleanup()
  {}

  void Renderer::notifyResize(std::uint32_t new_width, std::uint32_t new_height)
  {
    screen_.setSize(new_width, new_height);
    glViewport(0, 0, new_width, new_height);
    render_camera_.setOutputResolution(std::array<float, 2>{(float)new_width, (float)new_height});
    render_camera_.finalize();
    screen_.reinitBuffers();
  }

  void Renderer::commit()
  {
    if(world_ == nullptr)
      return;

    float current_frame = glfwGetTime();
    delta_time_ = current_frame - last_frame_;
    last_frame_ = current_frame;
    float sleep_time = 1.0f / max_fps_ - delta_time_;
    if(sleep_time > 0.0)
      sleep(sleep_time);

    for(auto& model : current_mesh_batches_)
      for(auto& mesh : model.second)
        mesh.second.clear();

    loadWorld();
    render();
  }

  void Renderer::loadWorld()
  {
    for(const auto& actor : world_->getActors())
    {
      if(render_collision_models_)
      {
        std::visit([this, &actor](const auto& shape_resolv) { loadActor(actor, shape_resolv); }, actor.get().collision_shape_);
      }
      else
      {
        for(const auto& shape : actor.get().visual_shapes_)
        {
          std::visit([this, &actor](const auto& shape_resolv) { loadActor(actor, shape_resolv); }, shape);
        }
      }
    }
  }

  void Renderer::loadActor(const Actor& actor, const ShapeBox& shape)
  {
    const auto size_mat = glm::scale(glm::mat4(1.f), ToV3(shape.half_extents_));
    const auto model_mat = shape.shape_transform_ * ToM4(actor.getModelMatrix()) * size_mat;

    loadInstance(shape.box_model_, {shape.diffuse_color_, shape.diffuse_color_, 0., "", "", ""}, model_mat);
  }

  void Renderer::loadActor(const Actor& actor, const ShapeCapsule& shape)
  {
    (void)actor; // todo
    (void)shape; // todo
  }

  void Renderer::loadActor(const Actor& actor, const ShapeCustomMesh& shape)
  {
    const auto size_mat = glm::scale(glm::mat4(1.f), shape.scale_);
    const auto model_mat = shape.shape_transform_ * ToM4(actor.getModelMatrix()) * size_mat;

    loadInstance(shape.custom_model_, shape.material_, model_mat);
  }

  void Renderer::loadActor(const Actor& actor, const ShapeCylinder& shape)
  {
    const auto size_mat = glm::scale(glm::mat4(1.f), glm::vec3(shape.radius_, shape.height_, shape.radius_));
    const auto model_mat = shape.shape_transform_ * ToM4(actor.getModelMatrix()) * size_mat;

    loadInstance(shape.cylinder_model_, {shape.diffuse_color_, shape.diffuse_color_, 0., "", "", ""}, model_mat);
  }

  void Renderer::loadActor(const Actor& actor, const ShapeDummy& shape)
  {
    (void)actor;
    (void)shape;
  }

  void Renderer::loadActor(const Actor& actor, const ShapeSphere& shape)
  {
    (void)actor; // todo
    (void)shape; // todo
  }

  void Renderer::loadInstance(const Model& model, const Material& material, const glm::mat4& model_mat)
  {
    loadModel(model, material);

    for(auto& mesh : model.meshes_)
      current_mesh_batches_[model.id_][mesh.id_].emplace_back(InstanceData{model_mat, {}}); // TODO add instance data ?
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

    /*if(material.specular_texture_.empty() == false)
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
    }*/

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

  void Renderer::render()
  {
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    // glStencilFunc(GL_ALWAYS, 1, 0xFF); // all fragments should pass the stencil test
    // glStencilMask(0xFF);               // enable writing to the stencil buffer

    render_collision_models_ = render_camera_.render_collision_models_;

    auto& light_shader = shaders_.at("default");
    auto& sky_shader = shaders_.at("sky");
    auto& shadow_shader = shaders_.at("depth");
    auto& shadow_point_shader = shaders_.at("depthcube");

    // 0. draw scene as normal in depth buffers

    // 0.1. ambient shadows
    auto ambient_dir = -glm::normalize(world_->ambient_light_.getDirection());
    shadow_.computeLightSpaceMatrices(render_camera_, ambient_dir);

    shadow_shader.use();
    shadow_.setLightMatrices();

    shadow_.bindFrameBuffer();
    glEnable(GL_DEPTH_TEST);

    renderModels(shadow_shader, 2);

    // 0.2 points shadows
    for(size_t i = 0; i < PointLights::MAX_POINT_LIGHTS; i++)
    {
      if(world_->point_lights_.isUsed(i))
      {
        if(point_shadows_.isInit(i) == false)
          point_shadows_.init(i, world_->point_lights_.getAttenuationDistance(i));

        point_shadows_.computeLightTransforms(i, glm::vec3(world_->point_lights_.getPosition(i)));

        shadow_point_shader.use();
        point_shadows_.bindFrameBuffer(i);
        point_shadows_.setUniforms(i, shadow_point_shader);

        renderModels(shadow_point_shader, 2);
      }
    }

    // 1. draw scene as normal in multisampled buffers

    screen_.bindFrameBuffer();
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    // glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glEnable(GL_FRAMEBUFFER_SRGB);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    // glStencilMask(0x00);

    light_shader.use();
    setLightsUniforms(light_shader);
    render_camera_.updateMatrices();
    light_shader.setVec3("view_pose", render_camera_.getPosition());
    light_shader.setMat4("view", render_camera_.getViewMatrix());
    light_shader.setMat4("projection", render_camera_.getProjectionMatrix());

    shadow_.setUniforms(light_shader, 1);
    shadow_.setLightMatrices();

    for(size_t i = 0; i < PointLights::MAX_POINT_LIGHTS; i++)
      if(world_->point_lights_.isUsed(i))
        point_shadows_.setUniforms(i, light_shader, 6);

    renderModels(light_shader, 2);

    sky_shader.use();
    glm::mat4 view = glm::mat4(glm::mat3(render_camera_.getViewMatrix()));
    sky_shader.setMat4("view", view);
    sky_shader.setMat4("projection", render_camera_.getProjectionMatrix());

    sky_.draw(sky_shader);

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

  void Renderer::setLightsUniforms(const Shader& shader)
  {
    AmbientLight& ambient = world_->ambient_light_;

    shader.setVec4("dir_light.ambient", ambient.getAmbient());
    shader.setVec4("dir_light.diffuse", ambient.getDiffuse());
    shader.setVec4("dir_light.specular", ambient.getSpecular());
    shader.setVec4("dir_light.direction", ambient.getDirection());

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

} // namespace owds