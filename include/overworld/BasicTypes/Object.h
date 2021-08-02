#ifndef OWDS_OBJECT_H
#define OWDS_OBJECT_H

#include "overworld/BasicTypes/Entity.h"
#include "overworld/BasicTypes/PointOfInterest.h"

namespace owds {

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
    bool isStatic() { return is_static_; }

protected:
    std::vector<PointOfInterest> points_of_interest_;
    bool is_static_;
};

} // namespace owds

#endif // OWDS_OBJECT_H
