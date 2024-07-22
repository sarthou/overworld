#include "overworld/Graphics/BGFX/Renderer.h"

#include <cstdint>
#include <fstream>
#include <glm/gtc/packing.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include <string>

#include "overworld/Engine/Common/Models/Model.h"
#include "overworld/Engine/Common/Urdf/Actor.h"
#include "overworld/Engine/Common/World.h"
#include "overworld/Graphics/BGFX/Camera.h"
#include "overworld/Graphics/BGFX/Vertex.h"
#include "overworld/Graphics/Base/Window.h"
#include "overworld/Helper/GlmMath.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define RENDER_SHADOW_PASS_ID 1
#define RENDER_SCENE_PASS_ID 0

namespace owds::bgfx {

  Renderer::Renderer() : shadows_renderer_(RENDER_SHADOW_PASS_ID)
  {}

  Renderer::~Renderer()
  {
    delete ctx_.render_camera_;
  }

  ::bgfx::VertexBufferHandle test_vbh;
  ::bgfx::IndexBufferHandle test_ibh;

  bool Renderer::initialize(const owds::Window& window)
  {
    const auto data = window.getPlatformData();

    ::bgfx::PlatformData pd;
    pd.ndt = data.native_display_type_;
    pd.nwh = data.native_window_handle_;

    ::bgfx::Init init;
    init.type = ::bgfx::RendererType::OpenGL;
    init.resolution.width = ctx_.width_;
    init.resolution.height = ctx_.height_;
    init.resolution.reset = ctx_.flags_;
    init.vendorId = BGFX_PCI_ID_NONE;
    init.platformData = pd;

    if(!::bgfx::init(init))
    {
      return false;
    }

    // const auto caps = ::bgfx::getCaps();
    // ctx_.instanced_rendering_supported = caps->supported & BGFX_CAPS_INSTANCING;

    ::bgfx::setViewClear(RENDER_SCENE_PASS_ID, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, 0x443355FF, 1.0f, 0);

    ctx_.shaders_.emplace(std::piecewise_construct, std::make_tuple("default"), std::make_tuple("vs_default", "fs_default"));
    ctx_.shaders_.emplace(std::piecewise_construct, std::make_tuple("skybox"), std::make_tuple("vs_skybox", "fs_skybox"));
    ctx_.shaders_.emplace(std::piecewise_construct, std::make_tuple("simple_shadows"), std::make_tuple("vs_simple_shadows", "fs_simple_shadows"));
    ctx_.is_initialized_ = true;
    ctx_.render_camera_ = new owds::bgfx::Camera();
    ctx_.render_camera_->setViewId(RENDER_SCENE_PASS_ID);

    shadows_renderer_.initialize();

    initMaterialUniforms();
    initLightUniforms();

    ctx_.loaded_uniforms_["view_position"] = ::bgfx::createUniform("view_position", ::bgfx::UniformType::Vec4);

    static owds::Color white_pixel{255, 255, 255, 255};

    ctx_.white_tex_ = ::bgfx::createTexture2D(
      1,
      1,
      false,
      1,
      ::bgfx::TextureFormat::RGBA8,
      0,
      ::bgfx::makeRef(&white_pixel, sizeof white_pixel));

    return true;
  }

  void Renderer::cleanup()
  {
    ::bgfx::shutdown();
  }

  void Renderer::notifyPreReset()
  {
    // todo
  }

  void Renderer::notifyPostReset()
  {
    // todo
  }

  void Renderer::notifyResize(const std::uint32_t new_width, const std::uint32_t new_height)
  {
    ctx_.width_ = new_width;
    ctx_.height_ = new_height;
    ctx_.has_size_changed_ = true;
  }

  void Renderer::runSanityChecks()
  {
    // todo: add some checks
  }

  void Renderer::commit()
  {
    if(ctx_.has_size_changed_)
    {
      ctx_.has_size_changed_ = false;
      ::bgfx::reset(ctx_.width_, ctx_.height_, ctx_.flags_);
    }

    for(auto& [id, batch] : ctx_.current_mesh_batches_)
    {
      batch.clear();
    }

    commitWorld(*(ctx_.world_));

    ctx_.render_camera_->setOutputResolution({static_cast<float>(ctx_.width_), static_cast<float>(ctx_.height_)});
    ctx_.render_camera_->updateMatrices();
    commitCamera(*ctx_.render_camera_);
    // commitShadows();

    for(const auto& camera : ctx_.cameras_)
    {
      commitCamera(*camera);
    }

    ::bgfx::frame();
  }

  void Renderer::attachWorld(World& world)
  {
    ctx_.world_ = std::addressof(world);
  }

  owds::Camera& Renderer::createCamera(const std::string& alias_name)
  {
    assert(ctx_.is_initialized_ && "The renderer must be initialized before creating a camera");

    const auto& camera = ctx_.cameras_.emplace_back(std::make_unique<owds::bgfx::Camera>());

    ctx_.camera_refs_.emplace_back(*camera);

    if(!alias_name.empty())
    {
      ctx_.named_cameras_.emplace(alias_name, std::ref(*camera));
    }

    return *camera;
  }

  std::vector<std::reference_wrapper<owds::Camera>> Renderer::getCameras()
  {
    return ctx_.camera_refs_;
  }

  owds::Camera* Renderer::getRenderCamera()
  {
    return ctx_.render_camera_;
  }

  void Renderer::commitCamera(const owds::bgfx::Camera& camera)
  {
    constexpr auto state = 0 |
                           BGFX_STATE_WRITE_RGB |
                           BGFX_STATE_WRITE_A |
                           BGFX_STATE_WRITE_Z |
                           BGFX_STATE_DEPTH_TEST_LESS |
                           BGFX_STATE_CULL_CCW; // |
                                                // BGFX_STATE_MSAA;

    ::bgfx::setDebug(camera.show_debug_stats_ ? BGFX_DEBUG_STATS : 0);

    ctx_.render_collision_models_ = camera.render_collision_models_;

    ::bgfx::touch(0); // Submit an empty primitive for rendering. Uniforms and draw state will be applied but no geometry will be submitted.
    if(ctx_.instanced_rendering_supported)
    {
      renderInstanced(state);
    }
    else
    {
      render(state, camera);
    }
  }

  void Renderer::commitShadows()
  {
    auto state = shadows_renderer_.state_;

    ::bgfx::setDebug(shadows_renderer_.camera_.show_debug_stats_ ? BGFX_DEBUG_STATS : 0);

    ctx_.render_collision_models_ = shadows_renderer_.camera_.render_collision_models_;

    ::bgfx::touch(0); // Submit an empty primitive for rendering. Uniforms and draw state will be applied but no geometry will be submitted.
    if(ctx_.instanced_rendering_supported)
    {
      renderInstanced(state);
    }
    else
    {
      render(state, shadows_renderer_.camera_, "simple_shadows");
    }
  }

  void Renderer::commitWorld(const owds::World& world)
  {
    for(const auto& actor : world.getActors())
    {
      if(ctx_.render_collision_models_)
      {
        std::visit([this, &actor](const auto& shape_resolv) { queueActorBatch(actor, shape_resolv); }, actor.get().collision_shape_);
      }
      else
      {
        for(const auto& shape : actor.get().visual_shapes_)
        {
          std::visit([this, &actor](const auto& shape_resolv) { queueActorBatch(actor, shape_resolv); }, shape);
        }
      }
    }
  }

  void Renderer::queueActorBatch(const owds::Actor& actor, const owds::ShapeBox& shape)
  {
    const auto size_mat = glm::scale(glm::mat4(1.f), ToV3(shape.half_extents_));
    const auto model_mat = ToM4(actor.getModelMatrix()) * size_mat;

    queueModelBatch(shape.box_model_, {shape.color_rgba_, ""}, FromM4(model_mat));
  }

  void Renderer::queueActorBatch(const owds::Actor& actor, const owds::ShapeCapsule& shape)
  {
    (void)actor; // todo
    (void)shape; // todo
  }

  void Renderer::queueActorBatch(const owds::Actor& actor, const owds::ShapeCustomMesh& shape)
  {
    const auto size_mat = glm::scale(glm::mat4(1.f), shape.scale_);
    const auto model_mat = ToM4(actor.getModelMatrix()) * size_mat;

    queueModelBatch(shape.custom_model_, shape.material_, FromM4(model_mat));
  }

  void Renderer::queueActorBatch(const owds::Actor& actor, const owds::ShapeCylinder& shape)
  {
    const auto size_mat = glm::scale(glm::mat4(1.f), glm::vec3(shape.radius_, shape.height_, shape.radius_));
    const auto model_mat = ToM4(actor.getModelMatrix()) * size_mat;

    queueModelBatch(shape.cylinder_model_, {shape.color_rgba_, ""}, FromM4(model_mat));
  }

  void Renderer::queueActorBatch(const owds::Actor& actor, const owds::ShapeDummy& shape)
  {
    (void)actor;
    (void)shape;
  }

  void Renderer::queueActorBatch(const owds::Actor& actor, const owds::ShapeSphere& shape)
  {
    (void)actor; // todo
    (void)shape; // todo
  }

  void Renderer::loadModel(const owds::Model& model, const owds::Material& material)
  {
    if(model.meshes_.empty())
      return;

    auto model_it = ctx_.cached_models_.find(model.id_);
    if(model_it != ctx_.cached_models_.end())
      return;

    model_it = ctx_.cached_models_.insert({model.id_, {}}).first;

    // Colors
    model_it->second.color_rgba_ = material.color_rgba_;
    model_it->second.shininess_ = 64.f; // TODO take from material
    model_it->second.specular_ = 0.1f;  // TODO take from material

    // Material
    if(material.texture_path_.empty() == false)
    {
      auto text_it = ctx_.loaded_textures_.find(material.texture_path_);
      if(text_it == ctx_.loaded_textures_.end())
      {
        model_it->second.tex_diffuse_ = loadTexture(material.texture_path_, BGFX_TEXTURE_SRGB);
        ctx_.loaded_textures_[material.texture_path_] = model_it->second.tex_diffuse_;
      }
      else
        model_it->second.tex_diffuse_ = text_it->second;
    }
    else
      model_it->second.tex_diffuse_ = ctx_.white_tex_;

    // Meshes
    for(const auto& mesh : model.meshes_)
    {
      auto mesh_it = ctx_.loaded_meshes_.find(mesh.id_);
      if(mesh_it == ctx_.loaded_meshes_.end())
      {
        mesh_it = ctx_.loaded_meshes_.emplace(mesh.id_, owds::bgfx::MeshHandle{
                                                          ::bgfx::createVertexBuffer(::bgfx::makeRef(mesh.vertices_.data(),
                                                                                                     mesh.vertices_.size() * sizeof(mesh.vertices_[0])),
                                                                                     Vertex::getMSLayout()),
                                                          ::bgfx::createIndexBuffer(::bgfx::makeRef(mesh.indices_.data(),
                                                                                                    mesh.indices_.size() * sizeof(mesh.indices_[0])),
                                                                                    BGFX_BUFFER_INDEX32)})
                    .first;
      }

      model_it->second.meshes_.emplace(mesh.id_, mesh_it->second);
    }
  }

  void Renderer::queueModelBatch(const owds::Model& model, const owds::Material& material, const std::array<float, 16>& model_mat)
  {
    loadModel(model, material);

    for(auto& mesh : model.meshes_)
      ctx_.current_mesh_batches_[model.id_][mesh.id_].emplace_back(owds::InstanceData{model_mat, {}}); // TODO add instance data ?
  }

  void Renderer::render(const std::uint64_t state, const owds::bgfx::Camera& camera, const std::string& shader)
  {
    srand(0);

    setLightUniforms(ctx_.world_);

    auto view_position = glm::vec4(camera.getPosition(), 0.0f);
    ::bgfx::setUniform(ctx_.loaded_uniforms_["view_position"], glm::value_ptr(view_position));

    for(auto& [model_id, batch] : ctx_.current_mesh_batches_)
    {
      auto& model = ctx_.cached_models_[model_id];
      setMaterialUniforms(model);

      for(auto& [mesh_id, transforms] : batch)
      {
        const auto& mesh = model.meshes_[mesh_id];

        for(const auto& transform : transforms)
        {
          ::bgfx::setTransform(transform.mvp_.data(), 1);

          ::bgfx::setVertexBuffer(0, mesh.vbh_);
          ::bgfx::setIndexBuffer(mesh.ibh_);

          ::bgfx::setState(state);
          ctx_.shaders_.at(shader).submit(0);
        }
      }
    }
  }

  void Renderer::renderInstanced(const std::uint64_t state)
  {
    for(auto& [model_id, batch] : ctx_.current_mesh_batches_)
    {
      auto& model = ctx_.cached_models_[model_id];
      setMaterialUniforms(model);

      for(auto& [mesh_id, transforms] : batch)
      {
        const auto& mesh = model.meshes_[mesh_id];

        ::bgfx::InstanceDataBuffer idb{};
        ::bgfx::allocInstanceDataBuffer(&idb, transforms.size(), sizeof(transforms[0]));

        std::memcpy(idb.data, transforms.data(), transforms.size() * sizeof(transforms[0]));

        ::bgfx::setVertexBuffer(0, mesh.vbh_);
        ::bgfx::setIndexBuffer(mesh.ibh_);

        ::bgfx::setInstanceDataBuffer(&idb);

        ::bgfx::setState(state);
        ctx_.shaders_.at("default").submit(0);
      }
    }
  }

  void Renderer::initLightUniforms()
  {
    ctx_.loaded_uniforms_["dir_light_direction"] = ::bgfx::createUniform("dir_light_direction", ::bgfx::UniformType::Vec4);
    ctx_.loaded_uniforms_["dir_light_ambient"] = ::bgfx::createUniform("dir_light_ambient", ::bgfx::UniformType::Vec4);
    ctx_.loaded_uniforms_["dir_light_diffuse"] = ::bgfx::createUniform("dir_light_diffuse", ::bgfx::UniformType::Vec4);
    ctx_.loaded_uniforms_["dir_light_specular"] = ::bgfx::createUniform("dir_light_specular", ::bgfx::UniformType::Vec4);

    ctx_.loaded_uniforms_["point_light_position"] = ::bgfx::createUniform("point_light_position", ::bgfx::UniformType::Vec4, PointLights::MAX_POINT_LIGHTS);
    ctx_.loaded_uniforms_["point_light_ambient"] = ::bgfx::createUniform("point_light_ambient", ::bgfx::UniformType::Vec4, PointLights::MAX_POINT_LIGHTS);
    ctx_.loaded_uniforms_["point_light_diffuse"] = ::bgfx::createUniform("point_light_diffuse", ::bgfx::UniformType::Vec4, PointLights::MAX_POINT_LIGHTS);
    ctx_.loaded_uniforms_["point_light_specular"] = ::bgfx::createUniform("point_light_specular", ::bgfx::UniformType::Vec4, PointLights::MAX_POINT_LIGHTS);
    ctx_.loaded_uniforms_["point_light_attenuation"] = ::bgfx::createUniform("point_light_attenuation", ::bgfx::UniformType::Vec4, PointLights::MAX_POINT_LIGHTS);
    ctx_.loaded_uniforms_["nb_point_light"] = ::bgfx::createUniform("nb_point_light", ::bgfx::UniformType::Vec4);
  }

  void Renderer::setLightUniforms(World* world)
  {
    if(world == nullptr)
      return;

    ::bgfx::setUniform(ctx_.loaded_uniforms_["dir_light_ambient"], glm::value_ptr(getAmbientLight(world).getAmbient()));
    ::bgfx::setUniform(ctx_.loaded_uniforms_["dir_light_diffuse"], glm::value_ptr(getAmbientLight(world).getDiffuse()));
    ::bgfx::setUniform(ctx_.loaded_uniforms_["dir_light_specular"], glm::value_ptr(getAmbientLight(world).getSpecular()));
    ::bgfx::setUniform(ctx_.loaded_uniforms_["dir_light_direction"], glm::value_ptr(getAmbientLight(world).getDirection()));

    ::bgfx::setUniform(ctx_.loaded_uniforms_["point_light_ambient"], glm::value_ptr(getPointLights(world).getAmbients().at(0)), PointLights::MAX_POINT_LIGHTS);
    ::bgfx::setUniform(ctx_.loaded_uniforms_["point_light_diffuse"], glm::value_ptr(getPointLights(world).getDiffuses().at(0)), PointLights::MAX_POINT_LIGHTS);
    ::bgfx::setUniform(ctx_.loaded_uniforms_["point_light_specular"], glm::value_ptr(getPointLights(world).getSpeculars().at(0)), PointLights::MAX_POINT_LIGHTS);
    ::bgfx::setUniform(ctx_.loaded_uniforms_["point_light_position"], glm::value_ptr(getPointLights(world).getPositions().at(0)), PointLights::MAX_POINT_LIGHTS);
    ::bgfx::setUniform(ctx_.loaded_uniforms_["point_light_attenuation"], glm::value_ptr(getPointLights(world).getAttenuations().at(0)), PointLights::MAX_POINT_LIGHTS);
    ::bgfx::setUniform(ctx_.loaded_uniforms_["nb_point_light"], glm::value_ptr(getPointLights(world).getNbLights()));
  }

  void Renderer::initMaterialUniforms()
  {
    ctx_.loaded_uniforms_["material_texture_diffuse"] = ::bgfx::createUniform("material_texture_diffuse", ::bgfx::UniformType::Sampler);
    ctx_.loaded_uniforms_["material_texture_specular"] = ::bgfx::createUniform("material_texture_specular", ::bgfx::UniformType::Sampler);
    ctx_.loaded_uniforms_["material_color"] = ::bgfx::createUniform("material_color", ::bgfx::UniformType::Vec4);
    ctx_.loaded_uniforms_["material_shininess"] = ::bgfx::createUniform("material_shininess", ::bgfx::UniformType::Vec4);
    ctx_.loaded_uniforms_["material_specular"] = ::bgfx::createUniform("material_specular", ::bgfx::UniformType::Vec4);
  }

  void Renderer::setMaterialUniforms(const owds::bgfx::ModelHandle& model)
  {
    ::bgfx::setTexture(0, ctx_.loaded_uniforms_["material_texture_diffuse"], model.tex_diffuse_);
    auto material_shininess = glm::vec4(model.shininess_, 0.f, 0.f, 0.f);
    ::bgfx::setUniform(ctx_.loaded_uniforms_["material_shininess"], glm::value_ptr(material_shininess));
    auto material_specular = glm::vec4(model.specular_, 0.f, 0.f, 0.f);
    ::bgfx::setUniform(ctx_.loaded_uniforms_["material_specular"], glm::value_ptr(material_specular));
    auto material_color = glm::vec4(static_cast<float>(model.color_rgba_.r_) / 255.f,
                                    static_cast<float>(model.color_rgba_.g_) / 255.f,
                                    static_cast<float>(model.color_rgba_.b_) / 255.f,
                                    static_cast<float>(model.color_rgba_.a_) / 255.f);
    ::bgfx::setUniform(ctx_.loaded_uniforms_["material_color"], glm::value_ptr(material_color));
  }

  ::bgfx::TextureHandle Renderer::loadTexture(const std::string& file_name,
                                              uint64_t flags,
                                              ::bgfx::TextureInfo* texture_info,
                                              bool flip,
                                              bool cube_map)
  {
    // flag BGFX_TEXTURE_SRGB
    ::bgfx::TextureHandle handle;

    int width = 0, height = 0, channels = 0;
    stbi_set_flip_vertically_on_load(flip);
    unsigned char* data = stbi_load(file_name.c_str(), &width, &height, &channels, 0);

    if(data != nullptr)
    {
      const ::bgfx::Memory* mem = ::bgfx::makeRef(data, width * height * channels, [](void* ptr, void* user_data) {
        (void)user_data;
        stbi_image_free(ptr);
      });

      ::bgfx::TextureFormat::Enum format;
      if(channels == 1)
        format = ::bgfx::TextureFormat::R8;
      else if(channels == 3)
        format = ::bgfx::TextureFormat::RGB8;
      else if(channels == 4)
        format = ::bgfx::TextureFormat::RGBA8;

      if(texture_info != nullptr)
      {
        ::bgfx::calcTextureSize(*texture_info,
                                uint16_t(width),
                                uint16_t(height),
                                uint16_t(0), // depth
                                cube_map,
                                false, // umMips > 1
                                1,     // num layers
                                format);
      }

      if(cube_map)
      {
        handle = ::bgfx::createTextureCube(uint16_t(width),
                                           false, // umMips > 1
                                           channels, format,
                                           flags, mem);
      }
      else if(::bgfx::isTextureValid(0, false, channels, format, flags))
      {
        handle = ::bgfx::createTexture2D(uint16_t(width),
                                         uint16_t(height),
                                         false, // umMips > 1
                                         1,     // num layers
                                         format,
                                         flags, mem);
      }

      if(::bgfx::isValid(handle))
        ::bgfx::setName(handle, file_name.c_str(), file_name.size());
    }

    return handle;
  }

} // namespace owds::bgfx