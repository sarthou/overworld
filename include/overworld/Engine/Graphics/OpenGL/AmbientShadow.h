#ifndef OWDS_GRAPHICS_OPNEGL_SHADOW_H
#define OWDS_GRAPHICS_OPNEGL_SHADOW_H

#include <array>

#include "overworld/Engine/Graphics/OpenGL/Camera.h"
#include "overworld/Engine/Graphics/OpenGL/CameraUpdater.h"

namespace owds {
  class Shader;

  class AmbientShadow
  {
  public:
    void init(float near_plane, float far_plane);
    void bindFrameBuffer() const;
    void setUniforms(const Shader& shader, unsigned int texture_offset) const;
    void setLightMatrices();
    void computeLightSpaceMatrices(Camera ref_cam, const glm::vec3& light_dir);

  private:
    std::vector<glm::mat4> lightspace_matrices_;

    unsigned int matrices_uniform_buffer_;
    unsigned int depth_framebuffer_;
    unsigned int depth_maps_;
    unsigned int resolution_ = 4096;

    float near_plane_;
    float far_plane_;
    std::vector<float> shadow_cascade_levels_;

    glm::mat4 getLightSpaceMatrix(CameraUpdater& ref_cam, const glm::vec3& light_dir, float near_plane, float far_plane);
  };

} // namespace owds

#endif // OWDS_GRAPHICS_OPNEGL_SHADOW_H