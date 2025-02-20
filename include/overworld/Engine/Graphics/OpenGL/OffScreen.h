#ifndef OWDS_OFFSCREEN_H
#define OWDS_OFFSCREEN_H

#include <cstdint>

namespace owds {

  class OffScreen
  {
  public:
    OffScreen() = default;
    ~OffScreen();

    void init(unsigned int width, unsigned int height);
    void bindFrameBuffer() const;

    void getImage(uint32_t** data);

  private:
    unsigned int width_;
    unsigned int height_;

    unsigned int framebuffer_;
    unsigned int colorbuffer_;
    unsigned int depthbuffer_;
  };

} // namespace owds

#endif // OWDS_OFFSCREEN_H