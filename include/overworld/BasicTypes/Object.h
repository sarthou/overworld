#ifndef OWDS_OBJECT_H
#define OWDS_OBJECT_H

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include "overworld/BasicTypes/Entity.h"
#include "overworld/BasicTypes/PointOfInterest.h"

namespace owds {

  class Hand;

  struct HandStamped_t
  {
    Hand* hand;
    ros::Time stamp;
  };

  class Object : public Entity
  {
  public:
    explicit Object(const std::string& id, bool is_true_id = true);
    virtual ~Object() = default;

    void setPointsOfInterest(const std::string& module_name, const std::vector<PointOfInterest>& points_of_interest);
    void addPointOfInterest(const std::string& module_name, const PointOfInterest& point_of_interest);
    void emptyPointsOfInterest(const std::string& module_name);
    const std::vector<PointOfInterest>& getPointsOfInterest(const std::string& module_name) const;
    void setAllPoiUnseen(const std::string& module_name);

    // setInHand or removeFromHand should be done before the update pose
    void updatePose(const Pose& pose, ros::Time stamp = ros::Time::now()) override;
    void updatePose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation) override;
    void updatePose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation, ros::Time stamp) override;
    void updatePose(const geometry_msgs::PoseStamped& pose) override;

    Pose poseRaw() const;
    Pose pose() const override;
    Pose pose(unsigned int id) const override;
    Pose pose(const ros::Time& stamp) const override;

    std::array<double, 3> direction() const override;

    void setInHand(Hand* hand) { hand_in_ = hand; }
    void removeFromHand() { hand_in_ = nullptr; }
    bool isInHand() const { return (hand_in_ != nullptr); }
    Hand* getHandIn() const { return hand_in_; }
    Hand* getHandIn(unsigned int id) const;
    Hand* getHandIn(const ros::Time& stamp) const;

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
    std::unordered_map<std::string, std::vector<PointOfInterest>> points_of_interest_;
    Hand* hand_in_;
    CircularBuffer<HandStamped_t, 30> last_hands_;

    std::array<double, 3> bounding_box_;
    std::array<double, 3> origin_offset_;
    std::vector<Pose> corners_;

    double mass_;

    std::unordered_set<std::string> types_;
  };

} // namespace owds

#endif // OWDS_OBJECT_H
