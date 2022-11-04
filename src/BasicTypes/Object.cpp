#include "overworld/BasicTypes/Object.h"

#include "overworld/BasicTypes/Hand.h"
#include "overworld/Utility/ShellDisplay.h"

#include <limits>

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

void Object::computeCorners()
{
    double min_x = origin_offset_[0] - bounding_box_[0] / 2.;
    double max_x = origin_offset_[0] + bounding_box_[0] / 2.;
    double min_y = origin_offset_[1] - bounding_box_[1] / 2.;
    double max_y = origin_offset_[1] + bounding_box_[1] / 2.;
    double min_z = origin_offset_[2] - bounding_box_[2] / 2.;
    double max_z = origin_offset_[2] + bounding_box_[2] / 2.;
    corners_ = { {{min_x, min_y, min_z}, {0.0, 0.0, 0.0, 1.0}},
                 {{max_x, min_y, min_z}, {0.0, 0.0, 0.0, 1.0}},
                 {{min_x, max_y, min_z}, {0.0, 0.0, 0.0, 1.0}},
                 {{max_x, max_y, min_z}, {0.0, 0.0, 0.0, 1.0}},
                 {{min_x, min_y, max_z}, {0.0, 0.0, 0.0, 1.0}},
                 {{max_x, min_y, max_z}, {0.0, 0.0, 0.0, 1.0}},
                 {{min_x, max_y, max_z}, {0.0, 0.0, 0.0, 1.0}},
                 {{max_x, max_y, max_z}, {0.0, 0.0, 0.0, 1.0}} };
}

double Object::getMinDistanceTo(const Object& other)
{
    std::vector<Pose> other_corners = other.getCorners();
    double min_dist = std::numeric_limits<double>::max();

    for(auto& c_a : other_corners)
    {
        Pose map_to_corner = other.pose() * c_a;
        Pose in_b = map_to_corner.transformIn(pose());
        for(auto& c_b : corners_)
        {
            double dist = in_b.distanceSqTo(c_b);
            if(dist < min_dist)
                min_dist = dist;
        }
    }
    
    return sqrt(min_dist);
}

void Object::setTypes(const std::vector<std::string>& types)
{
    for(auto& type : types)
        types_.insert(type);
}

bool Object::isA(const std::string& type)
{
    return (types_.find(type) != types_.end());
}

} // namespace owds