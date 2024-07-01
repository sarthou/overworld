#ifndef OWDS_GRAPHICS_BASE_RENDERER_H
#define OWDS_GRAPHICS_BASE_RENDERER_H

#include <functional>
#include <string>

namespace owds {
  class Window;
  class World;
  class Actor;
  class Camera;

  class Renderer
  {
  public:
    virtual ~Renderer() = default;
    virtual bool initialize(const owds::Window& window) = 0;
    virtual void cleanup() = 0;
    virtual void notifyPreReset() = 0;
    virtual void notifyPostReset() = 0;
    virtual void notifyResize(std::uint32_t new_width, std::uint32_t new_height) = 0;

    // Call to `runSanityChecks` is optional, but recommended.
    virtual void runSanityChecks() = 0;

    virtual void commit() = 0;

    /**
     * @param alias_name This parameter is optional, leave empty if undesired.
     * @param world
     */
    virtual owds::Camera& createCamera(const std::string& alias_name, owds::World& world) = 0;

    virtual std::vector<std::reference_wrapper<owds::Camera>> getCameras() = 0;

    /**
     * Helper function
     */
    void createCamera(const std::string& alias_name, owds::World& world, const std::function<void(Camera&)>& creator_func);
  };
} // namespace owds

#endif // OWDS_GRAPHICS_BASE_RENDERER_H
