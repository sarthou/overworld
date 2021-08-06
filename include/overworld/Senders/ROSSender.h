#ifndef OWDS_ROSSENDER_H
#define OWDS_ROSSENDER_H

#include "SharedMemory/PhysicsClientC_API.h"
#include "overworld/BasicTypes/Object.h"
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <image_transport/image_transport.h>

#include <unordered_map>

namespace owds {
class ROSSender
{
  public:
    ROSSender(ros::NodeHandle* n);

    void sendEntitiesToTF(const std::vector<Entity>& entities);
    void sendEntitiesToRViz(const std::string& topic_name, const std::vector<Entity>& entities);
    void sendEntitiesToTFAndRViz(const std::string& rviz_topic_name, const std::vector<Entity>& entities);
    void sendEntitiesToTF(const std::map<std::string, Object>& entities);
    void sendEntitiesToRViz(const std::string& topic_name, const std::map<std::string, Object>& entities);
    void sendEntitiesToTFAndRViz(const std::string& rviz_topic_name, const std::map<std::string, Object>& entities);
    void sendImage(const std::string& topic_name, const b3CameraImageData& image);

  protected:
    ros::NodeHandle* n_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    image_transport::ImageTransport it_;

    std::unordered_map<std::string, ros::Publisher> rviz_publishers_;
    std::unordered_map<std::string, image_transport::Publisher> image_publishers_;
};
} // namespace owds

#endif /* OWDS_ROSSENDER_H */
