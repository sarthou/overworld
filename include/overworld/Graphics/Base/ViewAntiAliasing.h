#ifndef OWDS_GRAPHICS_BASE_VIEWANTIALIASING_H
#define OWDS_GRAPHICS_BASE_VIEWANTIALIASING_H

namespace owds {
  /**
   * off      Disables anti-aliasing, recommended if high performance is desired and you don't care about about rough Graphics.
   * msaa_x2  Will render at x2 the resolution and scale it down
   * msaa_x4  Will render at x4 the resolution and scale it down
   * msaa_x8  Will render at x8 the resolution and scale it down
   * msaa_x16 Will render at x16 the resolution and scale it down
   *
   * There's not that many visual improvements above MSAA x4 so you probably want to stay at that level to avoid wasting system resources.
   */
  enum class ViewAntiAliasing_e
  {
    off,
    msaa_x1 = off,
    msaa_x2,
    msaa_x4,
    msaa_x8,
    msaa_x16
  };
}

#endif // OWDS_GRAPHICS_BASE_VIEWANTIALIASING_H
