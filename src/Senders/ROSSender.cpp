#include "overworld/Senders/ROSSender.h"

#include <sensor_msgs/image_encodings.h>

namespace owds {

ROSSender::ROSSender(ros::NodeHandle* n) : n_(n), rviz_publishers_({}), image_publishers_({}), it_(*n) {}

void ROSSender::sendEntitiesToTF(const std::vector<Entity>& entities)
{
    std::vector<geometry_msgs::TransformStamped> transforms;
    for (const auto& entity : entities)
    {
        if (entity.isLocated())
        {
            transforms.push_back(entity.toTfTransform());
        }
    }
    tf_broadcaster_.sendTransform(transforms);
}

void ROSSender::sendEntitiesToRViz(const std::string& topic_name, const std::vector<Entity>& entities)
{
    if (rviz_publishers_.count(topic_name) == 0)
    {
        rviz_publishers_.insert(std::make_pair(topic_name, n_->advertise<visualization_msgs::MarkerArray>(topic_name, 1, false)));
    }
    ros::Publisher& pub = rviz_publishers_.at(topic_name);
    visualization_msgs::MarkerArray markers;
    size_t id = 0;
    for (const auto& entity : entities)
    {
        if (entity.isLocated())
        {
            markers.markers.push_back(entity.toMarker(id++, 1.0, "marker"));
        }
    }
    pub.publish(markers);
}

void ROSSender::sendEntitiesToTFAndRViz(const std::string& rviz_topic_name, const std::vector<Entity>& entities)
{
    if (rviz_publishers_.count(rviz_topic_name) == 0)
    {
        rviz_publishers_.insert(std::make_pair(rviz_topic_name, n_->advertise<visualization_msgs::MarkerArray>(rviz_topic_name, 1, false)));
    }
    ros::Publisher& pub = rviz_publishers_.at(rviz_topic_name);
    std::vector<geometry_msgs::TransformStamped> transforms;
    visualization_msgs::MarkerArray markers;
    size_t id = 0;
    for (const auto& entity : entities)
    {
        if (entity.isLocated())
        {
            markers.markers.push_back(entity.toMarker(id++, 1.0, "marker"));
            transforms.push_back(entity.toTfTransform());
        }
    }
    pub.publish(markers);
    tf_broadcaster_.sendTransform(transforms);
}

void ROSSender::sendEntitiesToTF(const std::map<std::string, Object>& entities)
{
    std::vector<geometry_msgs::TransformStamped> transforms;
    for (const auto& entity : entities)
    {
        if (entity.second.isLocated())
        {
            transforms.push_back(entity.second.toTfTransform());
        }
    }
    tf_broadcaster_.sendTransform(transforms);
}

void ROSSender::sendEntitiesToRViz(const std::string& topic_name, const std::map<std::string, Object>& entities)
{
    if (rviz_publishers_.count(topic_name) == 0)
    {
        rviz_publishers_.insert(std::make_pair(topic_name, n_->advertise<visualization_msgs::MarkerArray>(topic_name, 1, false)));
    }
    ros::Publisher& pub = rviz_publishers_.at(topic_name);
    visualization_msgs::MarkerArray markers;
    size_t id = 0;
    for (const auto& entity : entities)
    {
        if (entity.second.isLocated())
        {
            markers.markers.push_back(entity.second.toMarker(id++, 1.0, "marker"));
        }
    }
    pub.publish(markers);
}

void ROSSender::sendEntitiesToTFAndRViz(const std::string& rviz_topic_name, const std::map<std::string, Object>& entities)
{
    if (rviz_publishers_.count(rviz_topic_name) == 0)
    {
        rviz_publishers_.insert(std::make_pair(rviz_topic_name, n_->advertise<visualization_msgs::MarkerArray>(rviz_topic_name, 1, false)));
    }
    ros::Publisher& pub = rviz_publishers_.at(rviz_topic_name);
    std::vector<geometry_msgs::TransformStamped> transforms;
    visualization_msgs::MarkerArray markers;
    size_t id = 0;
    for (const auto& entity : entities)
    {
        if (entity.second.isLocated())
        {
            markers.markers.push_back(entity.second.toMarker(id++, 1.0, "marker"));
            transforms.push_back(entity.second.toTfTransform());
        }
    }
    pub.publish(markers);
    tf_broadcaster_.sendTransform(transforms);
}

void ROSSender::sendImage(const std::string& topic_name, const b3CameraImageData& image) {
    if (image_publishers_.count(topic_name) == 0){
        image_publishers_.insert(std::make_pair(topic_name, it_.advertise(topic_name, 1, false)));
    }
    image_transport::Publisher& pub = image_publishers_.at(topic_name);
    sensor_msgs::Image im;
    im.encoding = sensor_msgs::image_encodings::RGBA8;
    im.header.stamp = ros::Time::now();
    im.height = image.m_pixelHeight;
    im.width = image.m_pixelWidth;
    im.step = im.width * 4;
    im.data.resize(im.height * im.width * 4);
    memcpy(im.data.data(), image.m_rgbColorData, im.height * im.width * 4);
    pub.publish(im);
}

} // namespace owds
