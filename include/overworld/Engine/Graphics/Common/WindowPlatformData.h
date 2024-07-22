#ifndef OWDS_GRAPHICS_COMMON_WINDOWPLATFORMDATA_H
#define OWDS_GRAPHICS_COMMON_WINDOWPLATFORMDATA_H

namespace owds {
  class WindowPlatformData
  {
  public:
    void* native_display_type_;
    void* native_window_handle_;
  };
} // namespace owds

#endif // OWDS_GRAPHICS_COMMON_WINDOWPLATFORMDATA_H
