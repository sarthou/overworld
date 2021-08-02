#ifndef OWDS_FIELDOFVIEW_H
#define OWDS_FIELDOFVIEW_H

namespace owds {

class FieldOfView
{
public:
    FieldOfView(double height,
                double width,
                double clip_near,
                double clip_far) : height_(height),
                                   width_(width),
                                   clip_near_(clip_near),
                                   clip_far_(clip_far)
    {}

    double getHeight() { return height_; }
    double getWidth() { return width_; }
    double getClipNear() { return clip_near_; }
    double getClipFar() { return clip_far_; }

    double getRatio() { return width_ / height_; }

private:
    double height_; // degrees 
    double width_; // degrees
    double clip_near_; // meters
    double clip_far_; // meters
};

} // namespace owds

#endif // OWDS_FIELDOFVIEW_H