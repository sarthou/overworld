#include "overworld/Graphics/BGFX/Renderer.h"

#include <bx/bx.h>
#include <bx/file.h>
#include <bx/readerwriter.h>
#include <fstream>
#include <glm/gtc/packing.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

#include "overworld/Graphics/BGFX/Camera.h"
#include "overworld/Graphics/BGFX/Context.h"
#include "overworld/Graphics/BGFX/EmbeddedAssets.h"
#include "overworld/Graphics/BGFX/Vertex.h"
#include "overworld/Graphics/Base/Model.h"
#include "overworld/Graphics/Base/Window.h"
#include "overworld/Helper/GlmMath.h"
#include "overworld/Physics/Base/Actor.h"
#include "overworld/Physics/Base/World.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

namespace owds::bgfx {
  namespace implementation {
    std::unique_ptr<bx::FileReader> file_reader_ = std::make_unique<bx::FileReader>();

    static const ::bgfx::Memory* loadMem(bx::FileReaderI* _reader, const bx::FilePath& _filePath)
    {
      if(bx::open(_reader, _filePath))
      {
        uint32_t size = (uint32_t)bx::getSize(_reader);
        const ::bgfx::Memory* mem = ::bgfx::alloc(size + 1);
        bx::read(_reader, mem->data, size, bx::ErrorAssert{});
        bx::close(_reader);
        mem->data[mem->size - 1] = '\0';
        return mem;
      }

      BX_ASSERT(false, "Failed to load file.");

      return nullptr;
    }

    static ::bgfx::ShaderHandle loadShader(bx::FileReaderI* _reader, const bx::StringView& _name)
    {
      bx::FilePath filePath("../resources/shaders/compiled/");

      switch(::bgfx::getRendererType())
      {
      case ::bgfx::RendererType::OpenGL:
        filePath.join("glsl");
        break;
      case ::bgfx::RendererType::OpenGLES:
        filePath.join("essl");
        break;
      case ::bgfx::RendererType::Vulkan:
        filePath.join("spirv");
        break;
      default:
        BX_ASSERT(false, "You should not be here!");
        break;
      }

      char fileName[512];
      bx::strCopy(fileName, BX_COUNTOF(fileName), _name);
      bx::strCat(fileName, BX_COUNTOF(fileName), ".bin");

      filePath.join(fileName);

      ::bgfx::ShaderHandle handle = ::bgfx::createShader(loadMem(_reader, filePath.getCPtr()));
      ::bgfx::setName(handle, _name.getPtr(), _name.getLength());

      return handle;
    }

    ::bgfx::ShaderHandle loadShader(const bx::StringView& _name)
    {
      return loadShader(file_reader_.get(), _name);
    }
  } // namespace implementation

  Renderer::Renderer() : ctx_(std::make_unique<owds::bgfx::Context>()) {}
  Renderer::~Renderer() = default;

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
    init.resolution.width = ctx_->width_;
    init.resolution.height = ctx_->height_;
    init.resolution.reset = ctx_->flags_;
    init.vendorId = BGFX_PCI_ID_NONE;
    init.platformData = pd;

    if(!::bgfx::init(init))
    {
      return false;
    }

    // const auto caps = ::bgfx::getCaps();
    // ctx_->instanced_rendering_supported = caps->supported & BGFX_CAPS_INSTANCING;

    ::bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, 0x443355FF, 1.0f, 0);

    const auto type = ::bgfx::getRendererType();
    const auto vsh = ::bgfx::createEmbeddedShader(s_owds_embedded_shaders, type, "vs_default");
    const auto fsh = ::bgfx::createEmbeddedShader(s_owds_embedded_shaders, type, "fs_default");
    ctx_->loaded_programs_["default"] = ::bgfx::createProgram(vsh, fsh, true);
    ctx_->is_initialized_ = true;

    ctx_->loaded_uniforms_["material_texture_diffuse"] = ::bgfx::createUniform("material_texture_diffuse", ::bgfx::UniformType::Sampler);
    ctx_->loaded_uniforms_["material_texture_specular"] = ::bgfx::createUniform("material_texture_specular", ::bgfx::UniformType::Sampler);
    ctx_->loaded_uniforms_["material_color"] = ::bgfx::createUniform("material_color", ::bgfx::UniformType::Vec4);
    ctx_->loaded_uniforms_["material_shininess"] = ::bgfx::createUniform("material_shininess", ::bgfx::UniformType::Vec4);
    ctx_->loaded_uniforms_["material_specular"] = ::bgfx::createUniform("material_specular", ::bgfx::UniformType::Vec4);

    ambient_light_.registerUniforms(ctx_->loaded_uniforms_);
    point_lights_.registerUniforms(ctx_->loaded_uniforms_);

    ambient_light_ = AmbientLight(glm::vec4(-0.2f, -1.0f, -0.3f, 0.0f),
                                  glm::vec4(1.0f, 0.976f, 0.898f, 1.0f),
                                  0.4, 0.5, 1.0);

    point_lights_.addLight(glm::vec3(2.0f, -2.0f, 1.0f),
                           glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                           0.5, 0.5, 1.0,
                           glm::vec3(1.0f, 0.35f, 0.44f));

    point_lights_.addLight(glm::vec3(10.0f, -2.0f, 1.0f),
                           glm::vec4(0.0f, 1.0f, 1.0f, 1.0f),
                           0.5, 0.5, 1.0,
                           glm::vec3(1.0f, 0.35f, 0.44f));

    ctx_->loaded_uniforms_["view_position"] = ::bgfx::createUniform("view_position", ::bgfx::UniformType::Vec4);

    static owds::Color white_pixel{255, 255, 255, 255};

    ctx_->white_tex_ = ::bgfx::createTexture2D(
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
    ctx_->width_ = new_width;
    ctx_->height_ = new_height;
    ctx_->has_size_changed_ = true;
  }

  void Renderer::runSanityChecks()
  {
    // todo: add some checks
  }

  void Renderer::commit()
  {
    if(ctx_->has_size_changed_)
    {
      ctx_->has_size_changed_ = false;
      ::bgfx::reset(ctx_->width_, ctx_->height_, ctx_->flags_);
    }

    ctx_->cached_world_list_.clear();

    for(const auto& camera : ctx_->cameras_)
    {
      ctx_->cached_world_list_.emplace(std::addressof(camera->currently_viewed_world_.get()));
    }

    for(auto& [id, batch] : ctx_->current_mesh_batches_)
    {
      batch.clear();
    }

    for(const auto& world : ctx_->cached_world_list_)
    {
      commitWorld(*world);
    }

    for(const auto& camera : ctx_->cameras_)
    {
      camera->setOutputResolution({static_cast<float>(ctx_->width_), static_cast<float>(ctx_->height_)});
      camera->updateMatrices();
      commitCamera(*camera);
    }

    ::bgfx::frame();
  }

  owds::Camera& Renderer::createCamera(const std::string& alias_name, owds::World& world)
  {
    assert(ctx_->is_initialized_ && "The renderer must be initialized before creating a camera");

    const auto& camera = ctx_->cameras_.emplace_back(std::make_unique<owds::bgfx::Camera>(world));

    ctx_->camera_refs_.emplace_back(*camera);

    if(!alias_name.empty())
    {
      ctx_->named_cameras_.emplace(alias_name, std::ref(*camera));
    }

    return *camera;
  }

  std::vector<std::reference_wrapper<owds::Camera>> Renderer::getCameras()
  {
    return ctx_->camera_refs_;
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

    ctx_->render_collision_models_ = camera.render_collision_models_;

    ::bgfx::touch(0);
    if(ctx_->instanced_rendering_supported)
    {
      renderInstanced(state);
    }
    else
    {
      render(state, camera);
    }
  }

  void Renderer::commitWorld(const owds::World& world)
  {
    for(const auto& actor : world.getActors())
    {
      if(ctx_->render_collision_models_)
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
    const auto size_mat = glm::scale(glm::mat4(1.f), ToV3(shape.scale_));
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

  void Renderer::tryCacheModel(const owds::Model& model, const owds::Material& material)
  {
    if(model.meshes_.empty())
    {
      return;
    }

    if(ctx_->cached_meshes_.count(model.meshes_.front().id_))
    {
      return;
    }

    auto tex = ctx_->white_tex_;

    if(!material.texture_path_.empty())
    {
      if(!ctx_->loaded_textures_.count(material.texture_path_))
      {
        std::ifstream t(material.texture_path_, std::ios::binary);
        assert(t.good());

        std::string data((std::istreambuf_iterator(t)), std::istreambuf_iterator<char>());
        assert(!data.empty());

        int width, height, channels;
        const auto pixels = reinterpret_cast<owds::Color*>(stbi_load_from_memory(
          reinterpret_cast<stbi_uc*>(data.data()),
          static_cast<int>(data.size()),
          &width,
          &height,
          &channels,
          4));

        assert(pixels);

        tex = ::bgfx::createTexture2D(
          width,
          height,
          false,
          1,
          ::bgfx::TextureFormat::RGBA8,
          0,
          ::bgfx::makeRef(pixels, width * height * sizeof(owds::Color), [](void* _ptr, void* _userData) {
            (void)_userData;
            stbi_image_free(_ptr);
          }));

        ctx_->loaded_textures_[material.texture_path_] = tex;
      }
    }

    for(const auto& mesh : model.meshes_)
    {
      ctx_->cached_meshes_.emplace(
        mesh.id_, owds::bgfx::MeshHandle{
                    material.color_rgba_,
                    tex,
                    ::bgfx::createVertexBuffer(
                      ::bgfx::makeRef(mesh.vertices_.data(), mesh.vertices_.size() * sizeof(mesh.vertices_[0])),
                      Vertex::getMSLayout()),
                    ::bgfx::createIndexBuffer(
                      ::bgfx::makeRef(mesh.indices_.data(), mesh.indices_.size() * sizeof(mesh.indices_[0])),
                      BGFX_BUFFER_INDEX32)});
    }
  }

  void Renderer::queueModelBatch(const owds::Model& model, const owds::Material& material, const std::array<float, 16>& model_mat)
  {
    tryCacheModel(model, material);

    for(auto& mesh : model.meshes_)
    {
      ctx_->current_mesh_batches_[model.id_][mesh.id_].emplace_back(owds::InstanceData{
        model_mat,
        {}});
    }
  }

  void Renderer::render(const std::uint64_t state, const owds::bgfx::Camera& camera)
  {
    srand(0);

    ambient_light_.setUniforms(ctx_->loaded_uniforms_);
    point_lights_.setUniforms(ctx_->loaded_uniforms_);

    auto view_position = glm::vec4(camera.world_eye_position_, 0.0f);
    ::bgfx::setUniform(ctx_->loaded_uniforms_["view_position"], glm::value_ptr(view_position));

    for(auto& [model_id, batch] : ctx_->current_mesh_batches_)
    {
      for(auto& [mesh_id, transforms] : batch)
      {
        const auto& mesh = ctx_->cached_meshes_[mesh_id];

        ::bgfx::setTexture(0, ctx_->loaded_uniforms_["material_texture_diffuse"], mesh.tex_);
        auto material_shininess = glm::vec4(64.f, 0.f, 0.f, 0.f);
        ::bgfx::setUniform(ctx_->loaded_uniforms_["material_shininess"], glm::value_ptr(material_shininess));
        auto material_specular = glm::vec4(0.1f, 0.f, 0.f, 0.f);
        ::bgfx::setUniform(ctx_->loaded_uniforms_["material_specular"], glm::value_ptr(material_specular));

        for(const auto& transform : transforms)
        {
          ::bgfx::setTransform(transform.mvp_.data(), 1);

          auto material_color = glm::vec4(
            static_cast<float>(mesh.color_rgba_.r_) / 255.f,
            static_cast<float>(mesh.color_rgba_.g_) / 255.f,
            static_cast<float>(mesh.color_rgba_.b_) / 255.f,
            static_cast<float>(mesh.color_rgba_.a_) / 255.f) /*glm::vec4(
                                                             0.1f + rand() % 200 / 128.f,
                                                             0.1f + rand() % 200 / 128.f,
                                                             0.1f + rand() % 200 / 128.f,
                                                             1)*/
            ;

          ::bgfx::setUniform(ctx_->loaded_uniforms_["material_color"], glm::value_ptr(material_color));

          ::bgfx::setVertexBuffer(0, mesh.vbh_);
          ::bgfx::setIndexBuffer(mesh.ibh_);

          ::bgfx::setState(state);
          ::bgfx::submit(0, ctx_->loaded_programs_["default"]);
        }
      }
    }
  }

  void Renderer::renderInstanced(const std::uint64_t state)
  {
    for(auto& [model_id, batch] : ctx_->current_mesh_batches_)
    {
      for(auto& [mesh_id, transforms] : batch)
      {
        auto& mesh = ctx_->cached_meshes_[mesh_id];

        ::bgfx::setTexture(0, ctx_->loaded_uniforms_["material_texture_diffuse"], mesh.tex_);

        ::bgfx::InstanceDataBuffer idb{};
        ::bgfx::allocInstanceDataBuffer(&idb, transforms.size(), sizeof(transforms[0]));

        std::memcpy(idb.data, transforms.data(), transforms.size() * sizeof(transforms[0]));

        ::bgfx::setVertexBuffer(0, mesh.vbh_);
        ::bgfx::setIndexBuffer(mesh.ibh_);

        ::bgfx::setInstanceDataBuffer(&idb);

        ::bgfx::setState(state);
        ::bgfx::submit(0, ctx_->loaded_programs_["default"]);
      }
    }
  }
} // namespace owds::bgfx