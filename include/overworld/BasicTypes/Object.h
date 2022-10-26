#ifndef OWDS_OBJECT_H
#define OWDS_OBJECT_H

#include "overworld/BasicTypes/Entity.h"
#include "overworld/BasicTypes/PointOfInterest.h"

namespace owds {

class Hand;

class Object: public Entity
{
public:
    explicit Object(const std::string& id, bool is_true_id = true);

    void setPointsOfInterest(const std::vector<PointOfInterest>& points_of_interest);
    void addPointOfInterest(const PointOfInterest& point_of_interest);
    void emptyPointsOfInterest();
    const std::vector<PointOfInterest>& getPointsOfInterest() const;
    void setAllPoiUnseen();

    void setStatic(bool is_static = true) { is_static_ = is_static; }
    bool isStatic() const { return is_static_; }

    // should be setted in hand through the hand
    void setInHand(Hand* hand) { hand_in_ = hand; }
    void removeFromHand();
    bool isInHand() const { return (hand_in_ != nullptr); }
    Hand* getHandIn() const { return hand_in_; }

    void merge(Object* other);

    void setBoundingBox(const std::array<double, 3>& bb) { bounding_box_ = bb; }
    const std::array<double, 3>& getBoundingBox() const { return bounding_box_; }
    double getBbVolume() const;
    void setOriginOffset(const std::array<double, 3>& origin_offset) { origin_offset_ = origin_offset; }
    const std::array<double, 3>& getOriginOffset() const { return origin_offset_; }
    void computeCorners();
    std::vector<Pose> getCorners() const { return corners_; }

    double getMinDistanceTo(const Object& other);

    void setMass(double mass) { mass_ = mass; }
    void setDefaultMass(double density = 500.0); // shape and aabb should exist to compute the default mass
    double getMass() const { return mass_; }

    void setTypes(const std::vector<std::string>& types);
    bool isA(const std::string& type);

protected:
    std::vector<PointOfInterest> points_of_interest_;
    bool is_static_;
    Hand* hand_in_;

    std::array<double, 3> bounding_box_;
    std::array<double, 3> origin_offset_;
    std::vector<Pose> corners_;

    double mass_;

    std::unordered_set<std::string> types_;
};

} // namespace owds

#endif // OWDS_OBJECT_H
