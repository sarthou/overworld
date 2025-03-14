#include "overworld/Engine/Graphics/OpenGL/OffScreen.h"

#include <cstdint>
#include <cstring>
#include <iostream>

#include "glad/glad.h"

namespace owds {

  void OffScreen::init(unsigned int width, unsigned int height)
  {
    width_ = width;
    height_ = height;

    glGenFramebuffers(1, &framebuffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);

    // ajoute un depth buffer bouffon

    glGenTextures(1, &colorbuffer_);
    glBindTexture(GL_TEXTURE_2D, colorbuffer_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, (int)width_, (int)height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorbuffer_, 0);

    glGenTextures(1, &depthbuffer_);
    glBindTexture(GL_TEXTURE_2D, depthbuffer_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, (int)width_, (int)height_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthbuffer_, 0);

    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
      std::cout << "ERROR::FRAMEBUFFER:: OffScreen framebuffer is not complete!" << std::endl;
      exit(-2);
    }
  }

  OffScreen::~OffScreen()
  {
    glDeleteFramebuffers(1, &framebuffer_);
  }

  void OffScreen::bindFrameBuffer() const
  {
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
    glViewport(0, 0, width_, height_);
  }

  void OffScreen::getImage(uint32_t** data)
  {
    glReadPixels(0, 0, width_, height_,
                 GL_RGBA,
                 GL_UNSIGNED_INT_8_8_8_8,
                 *data);
  }

} // namespace owds