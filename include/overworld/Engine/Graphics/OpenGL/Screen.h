#ifndef OWDS_GRAPHICS_OPENGL_SCREEN_H
#define OWDS_GRAPHICS_OPENGL_SCREEN_H

#include <array>

namespace owds {

  class Screen
  {
    static std::array<float, 24> screen_vertices;

  public:
    void init();

    void setSize(unsigned int width, unsigned int height);

    void initBuffers(unsigned int msaa_samples);
    void reinitBuffers();
    void bindFrameBuffer() const;
    void generateColorTexture() const;
    void draw() const;

    unsigned int width_;
    unsigned int height_;

  private:
    unsigned int texture_color_buffer_ms_;
    unsigned int screen_texture_;
    unsigned int msaa_framebuffer_;
    unsigned int msaa_renderbuffer_;
    unsigned int intermediate_framebuffer_;
    unsigned int msaa_samples_;

    unsigned int screen_vao_;
    unsigned int screen_vbo_;

    bool init_ = false;
  };

} // namespace owds

#endif // OWDS_GRAPHICS_OPENGL_SCREEN_H