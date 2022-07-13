#include "overworld/BasicTypes/Object.h"

#include "overworld/BasicTypes/Hand.h"
#include "overworld/Utility/ShellDisplay.h"

namespace owds {

Object::Object(const std::string& id, bool is_true_id): Entity(id, is_true_id),
                                                        is_static_(false),
                                                        hand_in_(nullptr),
                                                        mass_(0)
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
    (void)points_of_interest_.empty();
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

void Object::removeFromHand()
{
    if(hand_in_ != nullptr)
    {
        hand_in_->removeFromHand(id_);
        hand_in_ = nullptr;
    }
    else
        ShellDisplay::warning("[Object] Try to remove " + id_ + " from hand while the object is not in any hand");
}

void Object::merge(Object* other)
{
    Entity::merge(other);

    if((isInHand() == false) && other->isInHand())
    {
        auto hand = other->getHandIn();
        other->removeFromHand();
        hand->putInHand(this);
    }
}

double Object::getBbVolume() const
{
    return (bounding_box_[0] * bounding_box_[1] * bounding_box_[2]);
}

void Object::setDefaultMass(double density)
{
    switch (shape_.type)
    {
    case ShapeType_e::SHAPE_MESH:
        mass_ = getBbVolume() * density;
        break;

    case ShapeType_e::SHAPE_SPEHERE:
        mass_ = 4.0*M_PI* std::pow(shape_.scale[0], 3) / 3.0;
        break;
    case ShapeType_e::SHAPE_CUBE:
        mass_ = (shape_.scale[0] * shape_.scale[1] * shape_.scale[2]) * density;
        break;
    case ShapeType_e::SHAPE_CYLINDER:
        mass_ = M_PI* std::pow((std::min(shape_.scale[0], shape_.scale[1]) / 2.), 2) * shape_.scale[2];
        break;
    default:
        throw std::runtime_error("setDefaultMass has been called on entity '" + id_ + "' + but its ShapeType is not defined.");
        break;
    }
}

} // namespace owds