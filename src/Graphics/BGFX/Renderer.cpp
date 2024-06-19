#include "overworld/Graphics/BGFX/Renderer.h"

#include <bx/bx.h>
#include <bx/file.h>
#include <bx/readerwriter.h>
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

    ctx_->loaded_uniforms_["u_color"] = ::bgfx::createUniform("u_color", ::bgfx::UniformType::Vec4);

    ::bgfx::setDebug(BGFX_DEBUG_STATS);

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

    for(auto& [mesh_id, batch] : ctx_->current_mesh_batches_)
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

    if(!alias_name.empty())
    {
      ctx_->named_cameras_.emplace(alias_name, std::ref(*camera));
    }

    return *camera;
  }

  void Renderer::commitCamera(const owds::bgfx::Camera& camera)
  {
    (void)camera;
    ::bgfx::touch(0);
    if(ctx_->instanced_rendering_supported)
    {
      renderInstanced(BGFX_STATE_DEFAULT);
    }
    else
    {
      render(BGFX_STATE_DEFAULT);
    }
  }

  void Renderer::commitWorld(const owds::World& world)
  {
    for(const auto& actor : world.getActors())
    {
      for(const auto& shape : actor.get().visual_shapes_)
      {
        std::visit([this, &actor](const auto& shape_resolv) { queueActorBatch(actor, shape_resolv); }, shape);
      }
    }
  }

  void Renderer::queueActorBatch(const owds::Actor& actor, const owds::ShapeBox& shape)
  {
    const auto size_mat = glm::scale(glm::mat4(1.f), ToV3(shape.half_extents_));
    const auto model_mat = ToM4(actor.getModelMatrix()) * size_mat;

    queueModelBatch(shape.box_model_, FromM4(model_mat));
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

    queueModelBatch(shape.custom_model_, FromM4(model_mat));
  }

  void Renderer::queueActorBatch(const owds::Actor& actor, const owds::ShapeCylinder& shape)
  {
    const auto size_mat = glm::scale(glm::mat4(1.f), glm::vec3(shape.radius_, shape.height_, shape.radius_));
    const auto model_mat = ToM4(actor.getModelMatrix()) * size_mat;

    queueModelBatch(shape.cylinder_model_, FromM4(model_mat));
  }

  void Renderer::queueActorBatch(const owds::Actor& actor, const owds::ShapeDummy& shape)
  {
    (void)actor; // todo
    (void)shape; // todo
    assert(false && "Visual shape cannot be dummy!");
  }

  void Renderer::queueActorBatch(const owds::Actor& actor, const owds::ShapeSphere& shape)
  {
    (void)actor; // todo
    (void)shape; // todo
  }

  void Renderer::tryCacheModel(const owds::Model& model)
  {
    for(const auto& mesh : model.meshes_)
    {
      if(ctx_->cached_meshes_.count(mesh.id_))
      {
        return;
      }

      printf("Uploaded mesh to GPU from '%s'\n", model.source_path_.c_str());

      ctx_->cached_meshes_.emplace(
        mesh.id_, owds::bgfx::MeshHandle{
                    ::bgfx::createVertexBuffer(
                      ::bgfx::makeRef(mesh.vertices_.data(), mesh.vertices_.size() * sizeof(mesh.vertices_[0])),
                      Vertex::getMSLayout()),
                    ::bgfx::createIndexBuffer(
                      ::bgfx::makeRef(mesh.indices_.data(), mesh.indices_.size() * sizeof(mesh.indices_[0])),
                      BGFX_BUFFER_INDEX32)});
    }
  }

  void Renderer::queueModelBatch(const owds::Model& model, const std::array<float, 16>& model_mat)
  {
    tryCacheModel(model);

    for(auto& mesh : model.meshes_)
    {
      auto& mesh_batch = ctx_->current_mesh_batches_[mesh.id_];

      mesh_batch.emplace_back(owds::InstanceData{
        model_mat,
        {}});
    }
  }

  void Renderer::render(const std::uint64_t state)
  {
    srand(0);

    for(auto& [mesh_id, transforms] : ctx_->current_mesh_batches_)
    {
      auto& mesh = ctx_->cached_meshes_[mesh_id];

      for(auto& transform : transforms)
      {
        ::bgfx::setTransform(transform.mvp_.data(), 1);

        auto u_color = glm::vec4(
          0.1f + rand() % 200 / 128.f,
          0.1f + rand() % 200 / 128.f,
          0.1f + rand() % 200 / 128.f,
          1);

        ::bgfx::setUniform(ctx_->loaded_uniforms_["u_color"], glm::value_ptr(u_color));

        ::bgfx::setVertexBuffer(0, mesh.vbh_);
        ::bgfx::setIndexBuffer(mesh.ibh_);

        ::bgfx::setState(state);
        ::bgfx::submit(0, ctx_->loaded_programs_["default"]);
      }
    }
  }

  void Renderer::renderInstanced(const std::uint64_t state)
  {
    // todo
    for(auto& [mesh_id, transforms] : ctx_->current_mesh_batches_)
    {
      auto& mesh = ctx_->cached_meshes_[mesh_id];

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
} // namespace owds::bgfx