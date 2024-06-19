#ifndef OWDS_GRAPHICS_BASE_CAMERAVIEW_H
#define OWDS_GRAPHICS_BASE_CAMERAVIEW_H

namespace owds {
  /**
   * regular_view Will render the world with all the colors, textures and all the fancy visual details.
   * segmented_view Will randomly assign a color to every model & disable their textures.
   */
  enum class CameraView_e
  {
    regular_view,
    segmented_view
  };
} // namespace owds

#endif // OWDS_GRAPHICS_BASE_CAMERAVIEW_H