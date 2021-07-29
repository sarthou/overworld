#ifndef OWDS_ENTITY_H
#define OWDS_ENTITY_H

#include <ros/ros.h>
#include <exception>
#include <string>

#include "overworld/Geometry/Pose.h"

namespace owds {

class Entity
{
public:
    Entity(const std::string& id);

    void updatePose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation);
    void updatePose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation, ros::Time stamp);
    void unsetPose() { is_located_ = false; }
    bool isLocated() const { return is_located_; }
    const Pose& pose() const;

    void setId(const std::string& id) { id_ = id; }
    const std::string& id() const { return id_; }

    void setBulletId(int bullet_id) { bullet_id_ = bullet_id; }
    int bulletId() const { return bullet_id_; }

protected:
    std::string id_;
    Pose pose_;
    ros::Time last_pose_;
    bool is_located_;
    int bullet_id_;
};

class UnlocatedEntityError: public std::runtime_error
{
public:
    inline UnlocatedEntityError(const std::string& entity_name): 
        std::runtime_error("Entity '" + entity_name + "' is not located, but its pose has been asked.")
    {}
};

} // namespace owds

#endif // OWDS_ENTITY_H
