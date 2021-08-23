#ifndef OWDS_ENTITY_H
#define OWDS_ENTITY_H

#include <ros/ros.h>
#include <exception>
#include <string>

#include "overworld/Geometry/Pose.h"
#include "overworld/BasicTypes/Shape.h"
#include "overworld/Utility/CircularBuffer.h"

#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include "overworld/Bullet/BulletClient.h"

namespace owds {

struct PoseStamped_s
{
    Pose pose;
    ros::Time stamp;
};

class Entity
{
public:
    Entity(const std::string& id, bool is_true_id = true);

    void updatePose(const Pose& pose, ros::Time stamp = ros::Time::now());
    void updatePose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation);
    void updatePose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation, ros::Time stamp);
    void updatePose(const geometry_msgs::PoseStamped& pose);
    void unsetPose() { is_located_ = false; }
    bool isLocated() const { return is_located_; }
    const Pose& pose() const;
    ros::Time lastStamp() const { return last_poses_.back().stamp; }
    bool hasMoved() const;

    void setId(const std::string& id, bool is_true_id = true);
    std::string id() const { return id_; }
    bool isTrueId() const { return is_true_id_; }

    void addFalseId(const std::string& false_id) { false_ids_.insert(false_id); }
    std::unordered_set<std::string> getFalseIds() { return false_ids_; }
 
    void setBulletId(int bullet_id) { bullet_id_ = bullet_id; }
    int bulletId() const { return bullet_id_; }

    void setAabb(const struct aabb_t& aabb) { aabb_ = aabb; }
    struct aabb_t getAabb() const { return aabb_; }
    double getAabbVolume() const;
    bool isAabbValid() const { return aabb_.is_valid; }

    void setShape(const Shape_t& shape) { shape_ = shape; }
    const Shape_t& getShape() const { return shape_; }
    bool hasShape() const { return (shape_.type != SHAPE_NONE); }

    void setSeen() { nb_frame_unseen_ = 0; }
    void setUnseen() { if(nb_frame_unseen_ < 100) nb_frame_unseen_++; }
    bool hasBeenSeen() const { return (nb_frame_unseen_ == 0); }
    size_t getNbFrameUnseen() const { return nb_frame_unseen_; }

    void merge(const Entity* other);

    geometry_msgs::TransformStamped toTfTransform() const;
    visualization_msgs::Marker toMarker(int id, double lifetime, const std::string& ns) const;

    void computeFeature();

protected:
    std::string id_;
    bool is_true_id_;
    std::unordered_set<std::string> false_ids_;

    CircularBuffer<PoseStamped_s, 10> last_poses_;
    bool is_located_;
    int bullet_id_;
    Shape_t shape_;
    size_t nb_frame_unseen_;
    struct aabb_t aabb_;

    void* feature_;
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
