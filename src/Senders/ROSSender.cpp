#include "overworld/Senders/ROSSender.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace owds {

  ROSSender::ROSSender(ros::NodeHandle* n) : n_(n), rviz_publishers_({}), image_publishers_({}) {}

  void ROSSender::sendImage(const std::string& topic_name, uint32_t* image, unsigned int width, unsigned int height)
  {
    if(image_publishers_.count(topic_name) == 0)
    {
      image_publishers_.insert(std::make_pair(topic_name, n_->advertise<sensor_msgs::Image>(topic_name, 1, false)));
    }
    ros::Publisher& pub = image_publishers_.at(topic_name);
    sensor_msgs::Image im;
    im.encoding = sensor_msgs::image_encodings::RGBA8;
    im.header.stamp = ros::Time::now();
    im.height = height;
    im.width = width;
    im.step = im.width * 4;
    im.data.resize(im.height * im.width * 4);
    memcpy(im.data.data(), image, im.height * im.width * 4);
    pub.publish(im);
  }

} // namespace owds
