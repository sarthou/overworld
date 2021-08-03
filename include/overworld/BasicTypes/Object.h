#ifndef OWDS_OBJECT_H
#define OWDS_OBJECT_H

#include "overworld/BasicTypes/Entity.h"
#include "overworld/BasicTypes/PointOfInterest.h"

namespace owds {

class BodyPart;

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

    void setInHand(BodyPart* hand) { hand_in_ = hand; }
    void removeFromHand() { hand_in_ = nullptr; }
    bool isInHand() const { return (hand_in_ != nullptr); }
    BodyPart* getHandIn() const { return hand_in_; }

protected:
    std::vector<PointOfInterest> points_of_interest_;
    bool is_static_;
    BodyPart* hand_in_;
};

} // namespace owds

#endif // OWDS_OBJECT_H
