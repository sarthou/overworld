#ifndef OWDS_COMMON_POINT_LIGHTS_H
#define OWDS_COMMON_POINT_LIGHTS_H

#include <array>
#include <cstddef>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <string>
#include <unordered_map>

namespace owds {

  class PointLights
  {
  public:
    const static std::size_t MAX_POINT_LIGHTS = 20; // MAX_POINTLIGHT

    PointLights() : nb_lights_(0)
    {
      available_.fill(true);
    }

    std::size_t addLight(const glm::vec3& position,
                         const glm::vec3& color = glm::vec3(1.0),
                         float ambient_strength = 1.0f,
                         float diffuse_strength = 1.0f,
                         float specular_strength = 1.0f,
                         const glm::vec3& attenuation = glm::vec3(1.0f, 0.35f, 0.44f))
    {
      size_t id = findAvailableId();
      setPosition(id, position);
      colors_[id] = glm::vec4(color, 1.0);
      setAmbientStrength(id, ambient_strength);
      setDiffuseStrength(id, diffuse_strength);
      setSpecularStrength(id, specular_strength);
      setAttenuation(id, attenuation);
      attenuation_distances_[id] = 20;

      return id;
    }

    std::size_t addLight(const glm::vec3& position,
                         const glm::vec3& color = glm::vec3(1.0),
                         float ambient_strength = 1.0f,
                         float diffuse_strength = 1.0f,
                         float specular_strength = 1.0f,
                         float attenuation_radius = 10.f)
    {
      size_t id = findAvailableId();
      setPosition(id, position);
      colors_[id] = glm::vec4(color, 1.0);
      setAmbientStrength(id, ambient_strength);
      setDiffuseStrength(id, diffuse_strength);
      setSpecularStrength(id, specular_strength);
      setAttenuation(id, attenuation_radius);
      attenuation_distances_[id] = attenuation_radius;

      return id;
    }

    void removeLight(std::size_t id)
    {
      removeId(id);
      ambient_strengths_[id] = 0.f;
      diffuse_strengths_[id] = 0.f;
      specular_strengths_[id] = 0.f;
      computeAmbient(id);
      computeDiffuse(id);
      computeSpecular(id);
    }

    void setColor(std::size_t id, const glm::vec3& color)
    {
      colors_[id] = glm::vec4(color, 1.0f);
      computeAmbient(id);
      computeDiffuse(id);
      computeSpecular(id);
    }

    void setPosition(std::size_t id, const glm::vec3& position)
    {
      positions_[id] = glm::vec4(position, 1.0f);
    }

    void setAttenuation(std::size_t id, const glm::vec3& attenuation)
    {
      attenuations_[id] = glm::vec4(attenuation, 0.0f);
    }

    void setAttenuation(std::size_t id, float radius)
    {
      attenuations_[id] = glm::vec4(1.0f, 2.0 / radius, 1.0 / (radius /* * radius*/), 0.0f);
    }

    void setAmbientStrength(std::size_t id, float strength)
    {
      ambient_strengths_[id] = strength;
      computeAmbient(id);
    }

    void setDiffuseStrength(std::size_t id, float strength)
    {
      diffuse_strengths_[id] = strength;
      computeDiffuse(id);
    }

    void setSpecularStrength(std::size_t id, float strength)
    {
      specular_strengths_[id] = strength;
      computeSpecular(id);
    }

    const std::array<glm::vec4, MAX_POINT_LIGHTS>& getAmbients() const { return ambients_; }
    const std::array<glm::vec4, MAX_POINT_LIGHTS>& getDiffuses() const { return diffuses_; }
    const std::array<glm::vec4, MAX_POINT_LIGHTS>& getSpeculars() const { return speculars_; }
    const std::array<glm::vec4, MAX_POINT_LIGHTS>& getPositions() const { return positions_; }
    const std::array<glm::vec4, MAX_POINT_LIGHTS>& getAttenuations() const { return attenuations_; }
    glm::vec4 getNbLights() const { return glm::vec4((float)nb_lights_); }

    bool isUsed(std::size_t id) const { return !available_[id]; }
    const glm::vec4& getAmbient(std::size_t id) const { return ambients_[id]; }
    const glm::vec4& getDiffuse(std::size_t id) const { return diffuses_[id]; }
    const glm::vec4& getSpecular(std::size_t id) const { return speculars_[id]; }
    const glm::vec4& getPosition(std::size_t id) const { return positions_[id]; }
    const glm::vec4& getAttenuation(std::size_t id) const { return attenuations_[id]; }
    float getAttenuationDistance(std::size_t id) const { return attenuation_distances_[id]; }
    float getNbLightsFloat() const { return (float)nb_lights_; }

  private:
    std::array<bool, MAX_POINT_LIGHTS> available_;
    std::size_t nb_lights_;

    std::array<glm::vec4, MAX_POINT_LIGHTS> colors_;
    std::array<glm::vec4, MAX_POINT_LIGHTS> positions_;

    std::array<float, MAX_POINT_LIGHTS> ambient_strengths_;
    std::array<float, MAX_POINT_LIGHTS> diffuse_strengths_;
    std::array<float, MAX_POINT_LIGHTS> specular_strengths_;

    std::array<glm::vec4, MAX_POINT_LIGHTS> ambients_;
    std::array<glm::vec4, MAX_POINT_LIGHTS> diffuses_;
    std::array<glm::vec4, MAX_POINT_LIGHTS> speculars_;
    std::array<glm::vec4, MAX_POINT_LIGHTS> attenuations_;
    std::array<float, MAX_POINT_LIGHTS> attenuation_distances_;

    void removeId(std::size_t id)
    {
      available_[id] = true;
      nb_lights_ = 0;
      for(std::size_t i = 0; i < MAX_POINT_LIGHTS; i++)
      {
        if(available_[i])
          nb_lights_ = i + 1;
      }
    }

    std::size_t findAvailableId()
    {
      std::size_t id = MAX_POINT_LIGHTS;
      for(std::size_t i = 0; i < MAX_POINT_LIGHTS; i++)
      {
        if(available_[i])
        {
          id = i;
          break;
        }
      }

      if(id >= nb_lights_)
        nb_lights_ = id + 1;

      available_[id] = false;
      return id;
    }

    void computeAmbient(std::size_t id)
    {
      ambients_[id] = glm::vec4(colors_[id].x * ambient_strengths_[id],
                                colors_[id].y * ambient_strengths_[id],
                                colors_[id].z * ambient_strengths_[id],
                                colors_[id].w * ambient_strengths_[id]);
    }

    void computeDiffuse(std::size_t id)
    {
      diffuses_[id] = glm::vec4(colors_[id].x * diffuse_strengths_[id],
                                colors_[id].y * diffuse_strengths_[id],
                                colors_[id].z * diffuse_strengths_[id],
                                colors_[id].w * diffuse_strengths_[id]);
    }

    void computeSpecular(std::size_t id)
    {
      speculars_[id] = glm::vec4(colors_[id].x * specular_strengths_[id],
                                 colors_[id].y * specular_strengths_[id],
                                 colors_[id].z * specular_strengths_[id],
                                 colors_[id].w * specular_strengths_[id]);
    }
  };

} // namespace owds

#endif // OWDS_COMMON_POINT_LIGHTS_H