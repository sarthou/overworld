#ifndef OWDS_BERNIE_H
#define OWDS_BERNIE_H
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
namespace owds {

class BernieSenders
{
  public:
    explicit BernieSenders(ros::NodeHandle* nh);
    void sendBernie();

  private:
    cv::Mat curlImg();
    cv::Mat img_;
    ros::NodeHandle* nh_;
    ros::Publisher image_pub_;
};

inline size_t curl_write_data(char* ptr, size_t size, size_t nmemb, void* userdata)
{
    std::vector<uchar>* stream = (std::vector<uchar>*)userdata;
    size_t count = size * nmemb;
    stream->insert(stream->end(), ptr, ptr + count);
    return count;
}

} // namespace owds

#endif /* OWDS_BERNIE_H */
