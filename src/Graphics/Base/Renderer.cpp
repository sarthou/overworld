#include "overworld/Graphics/Base/Renderer.h"

#include "overworld/Engine/Common/Lights/AmbientLight.h"
#include "overworld/Engine/Common/World.h"

namespace owds {

  const AmbientLight& Renderer::getAmbientLight(World* world)
  {
    return world->ambient_light_;
  }

  const PointLights& Renderer::getPointLights(World* world)
  {
    return world->point_lights_;
  }

} // namespace owds