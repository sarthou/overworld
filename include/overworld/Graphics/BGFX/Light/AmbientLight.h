#ifndef OWDS_GRAPHICS_BGFX_AMBIENT_LIGHT_H
#define OWDS_GRAPHICS_BGFX_AMBIENT_LIGHT_H

#include <glm/gtc/type_ptr.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <string>
#include <unordered_map>

#include "overworld/Graphics/BGFX/API.h"
#include "overworld/Graphics/Base/Light/AmbientLight.h"

namespace owds::bgfx {

  class AmbientLight final : public owds::AmbientLight
  {
  public:
    AmbientLight(const glm::vec3& direction = glm::vec3(0.0),
                 const glm::vec4& color = glm::vec4(1.0),
                 float ambient_strength = 1.0f,
                 float diffuse_strength = 1.0f,
                 float specular_strength = 1.0f) : owds::AmbientLight(direction,
                                                                      color,
                                                                      ambient_strength,
                                                                      diffuse_strength,
                                                                      specular_strength)
    {}

    void registerUniforms(std::unordered_map<std::string, ::bgfx::UniformHandle>& loaded_uniforms)
    {
      loaded_uniforms["dir_light_direction"] = ::bgfx::createUniform("dir_light_direction", ::bgfx::UniformType::Vec4);
      loaded_uniforms["dir_light_ambient"] = ::bgfx::createUniform("dir_light_ambient", ::bgfx::UniformType::Vec4);
      loaded_uniforms["dir_light_diffuse"] = ::bgfx::createUniform("dir_light_diffuse", ::bgfx::UniformType::Vec4);
      loaded_uniforms["dir_light_specular"] = ::bgfx::createUniform("dir_light_specular", ::bgfx::UniformType::Vec4);
    }

    void setUniforms(std::unordered_map<std::string, ::bgfx::UniformHandle>& loaded_uniforms) const
    {
      ::bgfx::setUniform(loaded_uniforms["dir_light_ambient"], glm::value_ptr(getAmbient()));
      ::bgfx::setUniform(loaded_uniforms["dir_light_diffuse"], glm::value_ptr(getDiffuse()));
      ::bgfx::setUniform(loaded_uniforms["dir_light_specular"], glm::value_ptr(getSpecular()));
      ::bgfx::setUniform(loaded_uniforms["dir_light_direction"], glm::value_ptr(getDirection()));
    }
  };

} // namespace owds::bgfx

#endif // OWDS_GRAPHICS_BGFX_AMBIENT_LIGHT_H