#ifndef OWDS_GRAPHICS_BGFX_POINT_LIGHTS_H
#define OWDS_GRAPHICS_BGFX_POINT_LIGHTS_H

#include <glm/gtc/type_ptr.hpp>
#include <glm/vec4.hpp>
#include <string>
#include <unordered_map>

#include "overworld/Graphics/BGFX/API.h"
#include "overworld/Graphics/Base/Light/PointLights.h"

namespace owds::bgfx {

  class PointLights final : public owds::PointLights
  {
  public:
    PointLights() = default;

    static void registerUniforms(std::unordered_map<std::string, ::bgfx::UniformHandle>& loaded_uniforms)
    {
      loaded_uniforms["point_light_position"] = ::bgfx::createUniform("point_light_position", ::bgfx::UniformType::Vec4, MAX_POINT_LIGHTS);
      loaded_uniforms["point_light_ambient"] = ::bgfx::createUniform("point_light_ambient", ::bgfx::UniformType::Vec4, MAX_POINT_LIGHTS);
      loaded_uniforms["point_light_diffuse"] = ::bgfx::createUniform("point_light_diffuse", ::bgfx::UniformType::Vec4, MAX_POINT_LIGHTS);
      loaded_uniforms["point_light_specular"] = ::bgfx::createUniform("point_light_specular", ::bgfx::UniformType::Vec4, MAX_POINT_LIGHTS);
      loaded_uniforms["point_light_attenuation"] = ::bgfx::createUniform("point_light_attenuation", ::bgfx::UniformType::Vec4, MAX_POINT_LIGHTS);
      loaded_uniforms["nb_point_light"] = ::bgfx::createUniform("nb_point_light", ::bgfx::UniformType::Vec4);
    }

    void setUniforms(std::unordered_map<std::string, ::bgfx::UniformHandle>& loaded_uniforms) const
    {
      ::bgfx::setUniform(loaded_uniforms["point_light_ambient"], glm::value_ptr(getAmbients().at(0)), MAX_POINT_LIGHTS);
      ::bgfx::setUniform(loaded_uniforms["point_light_diffuse"], glm::value_ptr(getDiffuses().at(0)), MAX_POINT_LIGHTS);
      ::bgfx::setUniform(loaded_uniforms["point_light_specular"], glm::value_ptr(getSpeculars().at(0)), MAX_POINT_LIGHTS);
      ::bgfx::setUniform(loaded_uniforms["point_light_position"], glm::value_ptr(getPositions().at(0)), MAX_POINT_LIGHTS);
      ::bgfx::setUniform(loaded_uniforms["point_light_attenuation"], glm::value_ptr(getAttenuations().at(0)), MAX_POINT_LIGHTS);
      ::bgfx::setUniform(loaded_uniforms["nb_point_light"], glm::value_ptr(getNbLights()));
    }
  };

} // namespace owds::bgfx

#endif // OWDS_GRAPHICS_BGFX_POINT_LIGHTS_H