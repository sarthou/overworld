#include "overworld/Graphics/Base/Renderer.h"
#include "overworld/Graphics/Base/Camera.h"

namespace owds {
  void Renderer::createCamera(const std::string& alias_name, owds::World& world, const std::function<void(Camera&)>& creator_func) {
    auto& camera = createCamera(alias_name, world);
    creator_func(camera);
    camera.finalize();
  }
}