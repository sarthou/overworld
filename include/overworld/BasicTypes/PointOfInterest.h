#ifndef OWDS_POINTOFINTEREST_H
#define OWDS_POINTOFINTEREST_H

#include "overworld/Geometry/Pose.h"

#include <string>
#include <vector>

namespace owds {

class PointOfInterest
{
public:
    explicit PointOfInterest(const std::string& id) : id_(id), nb_unseen_frames_(0) {}

    const std::string& getId() const { return id_; }

    void addPoint(const Pose& point) { points_.push_back(point); }
    const std::vector<Pose>& getPoints() const { return points_; }

    void setUnseen() { if(nb_unseen_frames_ < 100) nb_unseen_frames_++; }
    void setSeen() { nb_unseen_frames_ = 0; }
    size_t getNbUnseen() const { return nb_unseen_frames_; }

private:
    std::string id_;
    std::vector<Pose> points_;
    size_t nb_unseen_frames_;
};

} // namespace owds

#endif // OWDS_POINTOFINTEREST_H