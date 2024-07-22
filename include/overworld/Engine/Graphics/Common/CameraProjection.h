#ifndef OWDS_GRAPHICS_COMMON_CAMERAPROJECTION_H
#define OWDS_GRAPHICS_COMMON_CAMERAPROJECTION_H

namespace owds {
  /**
   * Its best for you to look online about the differences between perspective and orthographic.

   * But generally, as a rule of thumb:
   * - Use ePERSPECTIVE if you need 3D
   * - Use eORTHOGRAPHIC if you need 2D
   */
  enum class CameraProjection_e
  {
    perspective,
    orthographic
  };
} // namespace owds

#endif // OWDS_GRAPHICS_COMMON_CAMERAPROJECTION_H