#ifndef ROSBAG_TO_VIDEO_H_
#define ROSBAG_TO_VIDEO_H_

#include <boost/filesystem.hpp>
#include "opencv2/core/core.hpp"
#include <cv_bridge/cv_bridge.h>

namespace bag_to_video {

    bool images_to_video(const boost::filesystem::path & output_path,
                         const float & fps);
    cv::Mat debayer(const cv_bridge::CvImageConstPtr & cv_ptr);
    bool extract_bag_to_images(const boost::filesystem::path & bag_path,
                               const std::string & topic_name,
                               boost::filesystem::path & output_path,
                               float & fps)
}

#endif  // ROSBAG_TO_VIDEO_H_