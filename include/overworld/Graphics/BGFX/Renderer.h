#ifndef OWDS_GRAPHICS_BGFX_RENDERER_H
#define OWDS_GRAPHICS_BGFX_RENDERER_H

#include <memory>

#include "overworld/Graphics/BGFX/Context.h"
#include "overworld/Graphics/Base/Renderer.h"

namespace owds {
  class Actor;
  class ShapeBox;
  class ShapeCapsule;
  class ShapeCustomMesh;
  class ShapeCylinder;
  class ShapeDummy;
  class ShapeSphere;
  class Model;
  class Mesh;
  class Material;
} // namespace owds

namespace owds::bgfx {
  class Camera;

  class Renderer final : public owds::Renderer
  {
  public:
    Renderer();
    ~Renderer() override;

    bool initialize(const owds::Window& window) override;
    void cleanup() override;
    void notifyPreReset() override;
    void notifyPostReset() override;
    void notifyResize(std::uint32_t new_width, std::uint32_t new_height) override;

    void runSanityChecks() override;
    void commit() override;

    void attachWorld(World& world);
    owds::Camera& createCamera(const std::string& alias_name) override;

    std::vector<std::reference_wrapper<owds::Camera>> getCameras() override;
    owds::Camera* getRenderCamera() override;

    using owds::Renderer::createCamera;

  protected:
    void commitCamera(const owds::bgfx::Camera& camera);
    void commitWorld(const owds::World& world);
    void queueActorBatch(const owds::Actor& actor, const owds::ShapeBox& shape);
    void queueActorBatch(const owds::Actor& actor, const owds::ShapeCapsule& shape);
    void queueActorBatch(const owds::Actor& actor, const owds::ShapeCustomMesh& shape);
    void queueActorBatch(const owds::Actor& actor, const owds::ShapeCylinder& shape);
    void queueActorBatch(const owds::Actor& actor, const owds::ShapeDummy& shape);
    void queueActorBatch(const owds::Actor& actor, const owds::ShapeSphere& shape);

    void tryCacheModel(const owds::Model& model, const owds::Material& material);
    void queueModelBatch(const owds::Model& model, const owds::Material& material, const std::array<float, 16>& model_mat);

    void render(std::uint64_t state, const owds::bgfx::Camera& camera);
    void renderInstanced(std::uint64_t state);

    void initLightUniforms();
    void setLightUniforms(World* world);

    ::bgfx::TextureHandle loadTexture(const std::string& file_name,
                                      uint64_t flags = 0,
                                      ::bgfx::TextureInfo* texture_info = nullptr,
                                      bool flip = false,
                                      bool cube_map = false);

    owds::bgfx::Context ctx_;
  };
} // namespace owds::bgfx

#endif // OWDS_GRAPHICS_BGFX_RENDERER_H