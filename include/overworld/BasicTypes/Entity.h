#ifndef ENTITY_H
#define ENTITY_H

#include <ros/ros.h>
#include <exception>
#include "overworld/Geometry/Pose.h"

namespace owds{

class Entity{
    public:
    Entity();
    Entity(const std::string& id);

    void updatePose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation);
    void updatePose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation, ros::Time stamp);
    void unsetPose();
    bool isLocated() const;
    const owds::Pose& pose() const;

    void setId(const std::string& id);
    const std::string& id() const;

    protected:
    std::string id_;
    Pose pose_;
    ros::Time lastPose_;
    bool isLocated_;
};

class UnlocatedEntityError: public std::runtime_error{
    public:
    inline UnlocatedEntityError(const std::string& entityName): 
        std::runtime_error("Entity '" + entityName + "' is not located, but its pose has been asked."){

    }
};
}


#endif /* ENTITY_H */
