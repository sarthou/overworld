#include "overworld/Senders/ROSSender.h"

#include <string>

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

    uint8_t* dst = im.data.data();
    uint8_t* src = reinterpret_cast<uint8_t*>(image) + (height - 1) * im.step;

    for (unsigned int row = 0; row < height; ++row, dst += im.step, src -= im.step)
    {
        for (unsigned int col = 0; col < width * 4; col += 4)
        {
            std::swap(src[col + 0], src[col + 3]);  // Swap R and B
            std::swap(src[col + 1], src[col + 2]);  // Swap R and B
            std::memcpy(dst + col, src + col, 4);   // Copy pixel
        }
    }

    pub.publish(im);
  }

} // namespace owds
