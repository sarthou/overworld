#ifndef OWDS_OBJECT_H
#define OWDS_OBJECT_H

#include "overworld/BasicTypes/Entity.h"
#include "overworld/BasicTypes/PointOfInterest.h"

namespace owds {

class Hand;

class Object: public Entity
{
public:
    Object(const std::string& id, bool is_true_id = true);

    void setPointsOfInterest(const std::vector<PointOfInterest>& points_of_interest);
    void addPointOfInterest(const PointOfInterest& point_of_interest);
    void emptyPointsOfInterest();
    const std::vector<PointOfInterest>& getPointsOfInterest() const;
    void setAllPoiUnseen();

    void setStatic() { is_static_ = true; }
    bool isStatic() const { return is_static_; }

    // should be setted in hand through the hand
    void setInHand(Hand* hand) { hand_in_ = hand; }
    void removeFromHand();
    bool isInHand() const { return (hand_in_ != nullptr); }
    Hand* getHandIn() const { return hand_in_; }

    void merge(Object* other);

protected:
    std::vector<PointOfInterest> points_of_interest_;
    bool is_static_;
    Hand* hand_in_;
};

} // namespace owds

#endif // OWDS_OBJECT_H
