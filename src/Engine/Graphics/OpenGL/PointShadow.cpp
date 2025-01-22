#include "overworld/Engine/Graphics/OpenGL/PointShadow.h"

#include <array>
#include <cstddef>
#include <glm/ext/vector_float3.hpp>
#include <glm/gtc/packing.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <limits>
#include <string>

#include "glad/glad.h"
#include "overworld/Engine/Graphics/OpenGL/Shader.h"

namespace owds {

  PointShadow::PointShadow()
  {
    is_init_.fill(false);
  }

  void PointShadow::init()
  {
    for(int id = 0; id < MAX_POINTS; id++)
    {
      glGenFramebuffers(1, &depth_framebuffer_[id]);

      glGenTextures(1, &depth_cubemap_[id]);
      glBindTexture(GL_TEXTURE_CUBE_MAP, depth_cubemap_[id]);
      for(unsigned int i = 0; i < 6; ++i)
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_DEPTH_COMPONENT,
                    resolution_, resolution_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);

      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

      constexpr float bordercolor[] = {1.0f, 1.0f, 1.0f, 1.0f};
      glTexParameterfv(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_BORDER_COLOR, bordercolor);

      glBindFramebuffer(GL_FRAMEBUFFER, depth_framebuffer_[id]);
      glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_cubemap_[id], 0);
      glDrawBuffer(GL_NONE);
      glReadBuffer(GL_NONE);

      int status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
      if(status != GL_FRAMEBUFFER_COMPLETE)
      {
        std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!";
        throw 0;
      }

      glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
  }

  void PointShadow::init(size_t id, float far_plane)
  {
    far_plane *= 2.5;
    far_plane_[id] = far_plane;
    projection_matrix_[id] = glm::perspective(glm::radians(90.0f), 1.0f, 0.02f, far_plane);

    /*glGenFramebuffers(1, &depth_framebuffer_[id]);

    glGenTextures(1, &depth_cubemap_[id]);
    glBindTexture(GL_TEXTURE_CUBE_MAP, depth_cubemap_[id]);
    for(unsigned int i = 0; i < 6; ++i)
      glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_DEPTH_COMPONENT,
                   resolution_, resolution_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    constexpr float bordercolor[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glTexParameterfv(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_BORDER_COLOR, bordercolor);

    glBindFramebuffer(GL_FRAMEBUFFER, depth_framebuffer_[id]);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_cubemap_[id], 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    int status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if(status != GL_FRAMEBUFFER_COMPLETE)
    {
      std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!";
      throw 0;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);*/
    is_init_[id] = true;
  }

  void PointShadow::bindFrameBuffer(size_t id) const
  {
    glBindFramebuffer(GL_FRAMEBUFFER, depth_framebuffer_[id]);
    glViewport(0, 0, resolution_, resolution_);
    glClear(GL_DEPTH_BUFFER_BIT);
  }

  void PointShadow::setUniforms(size_t id, const Shader& shader, unsigned int texture_offset) const
  {
    shader.setFloat("point_lights[" + std::to_string(id) + "].far_plane", far_plane_[id]);

    glActiveTexture(GL_TEXTURE0 + texture_offset + id);
    shader.setInt("point_lights[" + std::to_string(id) + "].depth_map", texture_offset + id);
    glBindTexture(GL_TEXTURE_CUBE_MAP, depth_cubemap_[id]);

    // always good practice to set everything back to defaults once configured.
    glActiveTexture(GL_TEXTURE0);
  }

  void PointShadow::setUniforms(size_t id, const Shader& shader) const
  {
    shader.setFloat("far_plane", far_plane_[id]);
    shader.setVec3("lightPos", positions_[id]);
    for(unsigned int i = 0; i < 6; ++i)
      shader.setMat4("shadowMatrices[" + std::to_string(i) + "]", shadow_transforms_[id][i]);
  }

  void PointShadow::computeLightTransforms(size_t id, const glm::vec3& light_pose)
  {
    positions_[id] = light_pose;
    shadow_transforms_[id].clear();
    shadow_transforms_[id].push_back(projection_matrix_[id] * glm::lookAt(light_pose, light_pose + glm::vec3(1.0, 0.0, 0.0), glm::vec3(0.0, -1.0, 0.0)));
    shadow_transforms_[id].push_back(projection_matrix_[id] * glm::lookAt(light_pose, light_pose + glm::vec3(-1.0, 0.0, 0.0), glm::vec3(0.0, -1.0, 0.0)));
    shadow_transforms_[id].push_back(projection_matrix_[id] * glm::lookAt(light_pose, light_pose + glm::vec3(0.0, 1.0, 0.0), glm::vec3(0.0, 0.0, 1.0)));
    shadow_transforms_[id].push_back(projection_matrix_[id] * glm::lookAt(light_pose, light_pose + glm::vec3(0.0, -1.0, 0.0), glm::vec3(0.0, 0.0, -1.0)));
    shadow_transforms_[id].push_back(projection_matrix_[id] * glm::lookAt(light_pose, light_pose + glm::vec3(0.0, 0.0, 1.0), glm::vec3(0.0, -1.0, 0.0)));
    shadow_transforms_[id].push_back(projection_matrix_[id] * glm::lookAt(light_pose, light_pose + glm::vec3(0.0, 0.0, -1.0), glm::vec3(0.0, -1.0, 0.0)));
  }

} // namespace owds