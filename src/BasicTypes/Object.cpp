#include "overworld/BasicTypes/Object.h"

namespace owds {

Object::Object(const std::string& id, bool is_true_id): Entity(id, is_true_id) {}

void Object::setPointsOfInterest(const std::vector<Pose>& points_of_interest)
{
    points_of_interest_ = points_of_interest;
}

void Object::addPointOfInterest(const Pose& point_of_interest)
{
    points_of_interest_.push_back(point_of_interest);
}

void Object::emptyPointsOfInterest()
{
    points_of_interest_.empty();
}

const std::vector<Pose>& Object::getPointsOfInterest() const
{
    return points_of_interest_;
}

} // namespace owds