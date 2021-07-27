#ifndef OBJECT_H
#define OBJECT_H
#include "overworld/BasicTypes/Entity.h"

namespace owds{

class Object: public Entity{
    public:
    Object();
    Object(const std::string& id);

    void setPointsOfInterest(const std::vector<Pose>& pointsOfInterest);
    void addPointOfInterest(const Pose& pointOfInterest);
    void emptyPointsOfInterest();
    const std::vector<Pose>& getPointsOfInterest() const;

    protected:
    std::vector<Pose> pointsOfInterest_;
    
};
}

#endif /* OBJECT_H */
