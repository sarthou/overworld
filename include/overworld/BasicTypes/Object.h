#ifndef OWDS_OBJECT_H
#define OWDS_OBJECT_H
#include "overworld/BasicTypes/Entity.h"

namespace owds {

class Object: public Entity
{
public:
    Object(const std::string& id, bool is_true_id = true);

    void setPointsOfInterest(const std::vector<Pose>& points_of_interest);
    void addPointOfInterest(const Pose& point_of_interest);
    void emptyPointsOfInterest();
    const std::vector<Pose>& getPointsOfInterest() const;

protected:
    std::vector<Pose> points_of_interest_;
};

} // namespace owds

#endif // OWDS_OBJECT_H
