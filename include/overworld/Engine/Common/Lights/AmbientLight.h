#ifndef OWDS_COMMON_AMBIENT_LIGHT_H
#define OWDS_COMMON_AMBIENT_LIGHT_H

#include <array>
#include <cmath>
#include <ctime>
#include <glm/gtc/type_ptr.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <iostream>
#include <time.h>

#include "overworld/Utils/SolarAzEl.h"

namespace owds {

  class AmbientLight
  {
  public:
    AmbientLight(const glm::vec3& direction = glm::vec3(0.0f, 0.0f, -1.0f),
                 const glm::vec3& color = glm::vec3(1.0),
                 float ambient_strength = 1.0f,
                 float diffuse_strength = 1.0f,
                 float specular_strength = 1.0f) : direction_(direction, 1.0f),
                                                   color_(glm::vec4(color, 1.0f)),
                                                   ambient_strength_(ambient_strength),
                                                   diffuse_strength_(diffuse_strength),
                                                   specular_strength_(specular_strength)
    {
      computeAmbient();
      computeDiffuse();
      computeSpecular();
    }

    AmbientLight(const std::array<float, 3>& lat_long_alt,
                 const glm::vec3& color = glm::vec3(1.0),
                 float ambient_strength = 1.0f,
                 float diffuse_strength = 1.0f,
                 float specular_strength = 1.0f) : color_(glm::vec4(color, 1.0f)),
                                                   ambient_strength_(ambient_strength),
                                                   diffuse_strength_(diffuse_strength),
                                                   specular_strength_(specular_strength),
                                                   latitude_(lat_long_alt[0]),
                                                   longitude_(lat_long_alt[1]),
                                                   altitude_(lat_long_alt[2])

    {
      double el = 0., az = 0.;
      solarAzEl(time(0), latitude_, longitude_, altitude_, &az, &el);
      setElevationAndAzimuth(glm::radians(el), glm::radians(az));
      computeAmbient();
      computeDiffuse();
      computeSpecular();
    }

    void setDirection(const glm::vec3& direction)
    {
      direction_ = glm::vec4(direction, 1.0f);
    }

    void setElevationAndAzimuth(float elevation, float azimuth)
    {
      float x = std::cos(elevation) * std::sin(azimuth);
      float y = std::cos(elevation) * std::cos(azimuth);
      float z = std::sin(elevation);
      setDirection(glm::vec3(y, -x, -z));

      if(z < 0)
      {
        diffuse_strength_ = 0.;
        specular_strength_ = 0.;
        color_ = glm::vec4(0.31, 0.435f, 0.78f, 1.f);
      }
      else if(z < 1.)
      {
        auto night_color = glm::vec4(0.31, 0.435f, 0.78f, 1.f);
        color_ = z * color_ + night_color * (1.f - z);
        diffuse_strength_ = z;
        specular_strength_ = z;
      }
    }

    void setColor(const glm::vec3& color)
    {
      color_ = glm::vec4(color, 1.0f);
      computeAmbient();
      computeDiffuse();
      computeSpecular();
    }

    void setAmbientStrength(float strength)
    {
      ambient_strength_ = strength;
      computeAmbient();
    }

    void setDiffuseStrength(float strength)
    {
      diffuse_strength_ = strength;
      computeDiffuse();
    }

    void setSpecularStrength(float strength)
    {
      specular_strength_ = strength;
      computeSpecular();
    }

    const glm::vec4& getDirection() const { return direction_; }
    const glm::vec4& getAmbient() const { return ambient_; }
    const glm::vec4& getDiffuse() const { return diffuse_; }
    const glm::vec4& getSpecular() const { return specular_; }

  private:
    glm::vec4 direction_;
    glm::vec4 color_;

    float ambient_strength_;
    float diffuse_strength_;
    float specular_strength_;

    float latitude_;
    float longitude_;
    float altitude_;

    glm::vec4 ambient_;
    glm::vec4 diffuse_;
    glm::vec4 specular_;

    void computeAmbient()
    {
      ambient_ = glm::vec4(color_.x * ambient_strength_,
                           color_.y * ambient_strength_,
                           color_.z * ambient_strength_,
                           color_.w * ambient_strength_);
    }

    void computeDiffuse()
    {
      diffuse_ = glm::vec4(color_.x * diffuse_strength_,
                           color_.y * diffuse_strength_,
                           color_.z * diffuse_strength_,
                           color_.w * diffuse_strength_);
    }

    void computeSpecular()
    {
      specular_ = glm::vec4(color_.x * specular_strength_,
                            color_.y * specular_strength_,
                            color_.z * specular_strength_,
                            color_.w * specular_strength_);
    }
  };

} // namespace owds

#endif // OWDS_COMMON_AMBIENT_LIGHT_H