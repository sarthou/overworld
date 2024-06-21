#ifndef OWDS_GRAPHICS_BGFX_CONTEXT_H
#define OWDS_GRAPHICS_BGFX_CONTEXT_H

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "overworld/Graphics/BGFX/API.h"
#include "overworld/Graphics/BGFX/MeshHandle.h"
#include "overworld/Graphics/Base/InstanceData.h"
#include "overworld/Graphics/Base/Model.h"

namespace owds {
  class Window;
  class World;
  class Camera;
} // namespace owds

namespace owds::bgfx {
  class Camera;

  class Context
  {
  public:
    bool is_initialized_ = false;
    std::uint32_t width_ = 640;
    std::uint32_t height_ = 480;
    std::uint32_t flags_ = BGFX_RESET_VSYNC;
    bool has_size_changed_ = false;
    bool instanced_rendering_supported = false;
    std::vector<std::unique_ptr<owds::bgfx::Camera>> cameras_;
    std::vector<std::reference_wrapper<owds::Camera>> camera_refs_;
    std::set<owds::World*> cached_world_list_;

    std::unordered_map<std::string, std::reference_wrapper<owds::bgfx::Camera>> named_cameras_;
    std::unordered_map<std::string, ::bgfx::ProgramHandle> loaded_programs_;
    std::unordered_map<std::string, ::bgfx::UniformHandle> loaded_uniforms_;
    std::unordered_map<std::string, ::bgfx::TextureHandle> loaded_textures_;

    ::bgfx::TextureHandle white_tex_{};

    std::unordered_map<owds::Mesh::Id, owds::bgfx::MeshHandle> cached_meshes_;
    std::unordered_map<owds::Model::Id, std::unordered_map<owds::Mesh::Id, std::vector<owds::InstanceData>>> current_mesh_batches_;

    Context();
    ~Context();

    Context(const Context& other) = delete;
    Context& operator=(const Context& other) = delete;

    Context(Context&& other) = delete;
    Context& operator=(Context&& other) = delete;
  };
} // namespace owds::bgfx

#endif // OWDS_GRAPHICS_BGFX_CONTEXT_H