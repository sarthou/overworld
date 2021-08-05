#include "overworld/BasicTypes/Object.h"

namespace owds {

Object::Object(const std::string& id, bool is_true_id): Entity(id, is_true_id), is_static_(false), hand_in_(nullptr)
{}

void Object::setPointsOfInterest(const std::vector<PointOfInterest>& points_of_interest)
{
    points_of_interest_ = points_of_interest;
}

void Object::addPointOfInterest(const PointOfInterest& point_of_interest)
{
    points_of_interest_.push_back(point_of_interest);
}

void Object::emptyPointsOfInterest()
{
    points_of_interest_.empty();
}

const std::vector<PointOfInterest>& Object::getPointsOfInterest() const
{
    return points_of_interest_;
}

void Object::setAllPoiUnseen()
{
    for(auto& poi : points_of_interest_)
        poi.setUnseen();
}

} // namespace owds