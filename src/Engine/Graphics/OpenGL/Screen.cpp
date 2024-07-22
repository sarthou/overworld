#include "overworld/Engine/Graphics/OpenGL/Screen.h"

#include <array>
#include <iostream>

#include "glad/glad.h"

namespace owds {

  void Screen::init()
  {
    glGenVertexArrays(1, &screen_vao_);
    glGenBuffers(1, &screen_vbo_);
    glBindVertexArray(screen_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, screen_vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(screen_vertices), &screen_vertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
  }

  void Screen::setSize(unsigned int width, unsigned int height)
  {
    width_ = width;
    height_ = height;
  }

  void Screen::initBuffers(unsigned int msaa_samples)
  {
    msaa_samples_ = msaa_samples;
    glGenFramebuffers(1, &msaa_framebuffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, msaa_framebuffer_);
    // create a multisampled color attachment texture
    glGenTextures(1, &texture_color_buffer_ms_);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, texture_color_buffer_ms_);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, (int)msaa_samples, GL_RGB, (int)width_, (int)height_, GL_TRUE);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, texture_color_buffer_ms_, 0);
    // create a (also multisampled) renderbuffer object for depth and stencil attachments
    glGenRenderbuffers(1, &msaa_renderbuffer_);
    glBindRenderbuffer(GL_RENDERBUFFER, msaa_renderbuffer_);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, (int)msaa_samples, GL_DEPTH24_STENCIL8, (int)width_, (int)height_);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, msaa_renderbuffer_);

    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
      std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // configure second post-processing framebuffer
    glGenFramebuffers(1, &intermediate_framebuffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, intermediate_framebuffer_);
    // create a color attachment texture
    glGenTextures(1, &screen_texture_);
    glBindTexture(GL_TEXTURE_2D, screen_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, (int)width_, (int)height_, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, screen_texture_, 0); // we only need a color buffer

    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
      std::cout << "ERROR::FRAMEBUFFER:: Intermediate framebuffer is not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    init_ = true;
    std::cout << "init ok" << std::endl;
  }

  void Screen::reinitBuffers()
  {
    if(init_ == false)
      return;

    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, texture_color_buffer_ms_);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, (int)msaa_samples_, GL_RGB, (int)width_, (int)height_, GL_TRUE);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);

    glBindRenderbuffer(GL_RENDERBUFFER, msaa_renderbuffer_);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, (int)msaa_samples_, GL_DEPTH24_STENCIL8, (int)width_, (int)height_);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    glBindTexture(GL_TEXTURE_2D, screen_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, (int)width_, (int)height_, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
  }

  void Screen::bindFrameBuffer() const
  {
    glBindFramebuffer(GL_FRAMEBUFFER, msaa_framebuffer_);
  }

  void Screen::generateColorTexture() const
  {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, msaa_framebuffer_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, intermediate_framebuffer_);
    glBlitFramebuffer(0, 0, (int)width_, (int)height_, 0, 0, (int)width_, (int)height_, GL_COLOR_BUFFER_BIT, GL_NEAREST);
  }

  void Screen::draw() const
  {
    glBindVertexArray(screen_vao_);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, screen_texture_); // use the now resolved color attachment as the quad's texture
    glDrawArrays(GL_TRIANGLES, 0, 6);
  }

  std::array<float, 24> Screen::screen_vertices = {
    // positions   // texCoords
    -1.0f, 1.0f, 0.0f, 1.0f,
    -1.0f, -1.0f, 0.0f, 0.0f,
    1.0f, -1.0f, 1.0f, 0.0f,

    -1.0f, 1.0f, 0.0f, 1.0f,
    1.0f, -1.0f, 1.0f, 0.0f,
    1.0f, 1.0f, 1.0f, 1.0f};

} // namespace owds