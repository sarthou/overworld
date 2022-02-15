#include "curl/curl.h"

#include <overworld/Senders/Bernie.h>
#include <sensor_msgs/Image.h>

namespace owds {
BernieSenders::BernieSenders(ros::NodeHandle* nh) : nh_(nh)
{
    img_ = curlImg();
    image_pub_ = nh->advertise<sensor_msgs::Image>("/overworld/bernie", 1);
}

void BernieSenders::sendBernie()
{
    cv_bridge::CvImage cv_image;
    cv_image.image = img_;
    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    image_pub_.publish(ros_image);
}

cv::Mat BernieSenders::curlImg()
{
    std::vector<uchar> stream;
    CURL* curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, "https://upload.wikimedia.org/wikipedia/en/0/0f/Bernie_Sanders_mittens.jpg"); // the img url
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_data);                                                   // pass the writefunction
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &stream); // pass the stream ptr to the writefunction
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10);        // timeout if curl_easy hangs,
    CURLcode res = curl_easy_perform(curl);             // start curl
    curl_easy_cleanup(curl);                            // cleanup
    return cv::imdecode(stream, -1);                    // 'keep-as-is'
}

} // namespace owds
