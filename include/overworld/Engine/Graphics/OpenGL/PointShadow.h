#ifndef OWDS_GRAPHICS_OPNEGL_POINTSHADOW_H
#define OWDS_GRAPHICS_OPNEGL_POINTSHADOW_H

#include <array>

#include "overworld/Engine/Common/Camera/Camera.h"

namespace owds {
  class Shader;

  class PointShadow
  {
    const static std::size_t MAX_POINTS = 20; // MAX_POINTLIGHT
  public:
    PointShadow();

    void init();
    void init(size_t id, float far_plane);
    bool isInit(std::size_t id) const { return is_init_[id]; }
    void bindFrameBuffer(size_t id) const;
    void setUniforms(size_t id, const Shader& shader, unsigned int texture_offset) const;
    void setUniforms(size_t id, const Shader& shader) const;
    void computeLightTransforms(size_t id, const glm::vec3& light_pose);

  private:
    std::array<bool, MAX_POINTS> is_init_;
    std::array<unsigned int, MAX_POINTS> depth_framebuffer_;
    std::array<unsigned int, MAX_POINTS> depth_cubemap_;
    unsigned int resolution_ = 1024;

    std::array<float, MAX_POINTS> far_plane_;
    std::array<glm::vec3, MAX_POINTS> positions_;
    std::array<glm::mat4, MAX_POINTS> projection_matrix_;
    std::array<std::vector<glm::mat4>, MAX_POINTS> shadow_transforms_;
  };

} // namespace owds

#endif // OWDS_GRAPHICS_OPNEGL_POINTSHADOW_H