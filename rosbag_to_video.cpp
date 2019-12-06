// ROSbag to Video

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// Converts an image topic (of RGGB format) from a ROSbag into an MP4 video

#include <stdio.h>

#include <algorithm>
#include <string>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>

namespace rosbag_to_video {

// Returns a debayered version of the input image.  Assumes RGGB input
cv::Mat debayer(const cv_bridge::CvImageConstPtr & cv_ptr) {
    cv::Mat debayered;
    cv::cvtColor(cv_ptr->image, debayered, CV_BayerBG2BGR);
    return debayered;
}

// Extract and save images from a given ROSbag camera topic to disk
//
// Assumes ROS topic is in Bayer RGGB format. Images saved as .png format
// TODO: function is too long.  Break up
//
// Argument Outputs:
//  output_path: directory of extracted images
//  fps: average number of frames per second for extracted images
bool ExtractBagToImages(const boost::filesystem::path & bag_path,
                           const std::string & topic_name,
                           boost::filesystem::path & output_path,
                           float & fps) {
    // Open ROSbag for message processing
    std::cout << "Opening bag for image extraction" << std::endl;
    if(!boost::filesystem::exists(bag_path)) {
        std::cout << "Error: Bag file not found" << std::endl;
        throw std::invalid_argument("Input bagfile not found");
    }
    rosbag::Bag bag;
    try{
        bag.open(bag_path.string());  // read-only by default
    }
    catch (rosbag::BagException & e) {
        std::cout << "Error: " << e.what()
                  << "\nUnable to open bag file: " << bag_path << std::endl;
        return false;
    }

    // Determine output location for extracted images
    output_path = bag_path.parent_path() / bag_path.stem();
    if(!boost::filesystem::create_directory(output_path)) {
        std::cout << "Unable to create output directory" << std::endl;
        return false;
    }

    // Save image messages to disk
    std::cout << "Extracting images to: " << output_path.string() << std::endl;
    rosbag::View view(bag, rosbag::TopicQuery(topic_name));
    int image_count = 0;
    ros::Time earliest, latest;
    for(rosbag::MessageInstance const message: view) {
        sensor_msgs::ImageConstPtr ros_image = 
                message.instantiate<sensor_msgs::Image>();
        if(ros_image != nullptr) {
            // Convert ROS Image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr;
            cv::Mat debayered_image;
            try {
                // Create copy (vs toCvShare) so we can debayer
                cv_ptr = cv_bridge::toCvCopy(*ros_image,
                    sensor_msgs::image_encodings::BAYER_RGGB8);
                debayered_image = debayer(cv_ptr);
            }
            catch (cv_bridge::Exception& e) {
                std::cout << "Error: " << e.what()
                          << "Unable to convert image message to OpenCV format"
                          << std::endl;
                continue;
            }

            // Save image to disk
            std::string filename = "extracted_" + std::to_string(image_count)
                                   + ".png";
            cv::imwrite(output_path.string() + filename, debayered_image);
            image_count++;
            
            // Track earliest and latest message timestamps for framerate
            earliest = std::min(earliest, ros_image->header.stamp);
            latest = std::max(latest, ros_image->header.stamp);
        }
    }
    bag.close();
    std::cout << "Extraction complete.  Processed " << image_count << " images"
              << std::endl;

    // Calculate average frame rate
    if(image_count > 1) {
        double duration = (latest - earliest).toSec();
        fps = image_count / duration;
        return true;
    }
    else if(image_count == 1) {
        // Arbitrary fps if only a single image is extracted
        fps = 1.0;
        return true;
    }
    return false;
}

// Create an MP4 video from a directory of images
// 
// It is possible to create a video directly from FFmpeg/libx264 C libraries but
// quite involved; more straightforward to run on the commandline.  OpenCV's
// VideoWriter class only supports AVI
bool ImagesToVideo(const boost::filesystem::path & output_path,
                     const float & fps) {
    std::string video_name = output_path.parent_path().string()
                             + output_path.stem().string() + ".mp4";

    // Construct terminal string to execute
    std::cout << "Constructing video from extracted images.  "
              << "This process may take some time..." << std::endl;
    std::string vid_command = "ffmpeg -framerate " + std::to_string(fps)
                              + " -i extracted_%d.png -vcodec libx264 -crf 25 "
                              "-pix_fmt yuv420p " + video_name;

    if(system(vid_command.c_str()) != -1){
        std::cout << "Video saved to: " << video_name << std::endl;
        return true;
    }
    return false;

}
}

int main(int argc, char * argv[]) {
    // Parse command line arguments
    if(argc != 3) {
        std::cout << "Incorrect command line usage. Expected usage:\n\t"
                  << argv[0] << " /path_to_bag/bag_name.bag /image_topic_name"
                  << std::endl;
    }
    boost::filesystem::path bag_path (argv[1]), images_path;
    std::string topic_name (argv[2]);
    float fps = -1.0;
    
    if(!rosbag_to_video::ExtractBagToImages(bag_path, topic_name,
                                               images_path, fps)) {
        std::cout << "Unable to extract and save ROSbag images" << std::endl;
        return EXIT_FAILURE;
    }
    if(!rosbag_to_video::ImagesToVideo(images_path, fps)) {
        std::cout << "Unable to convert saved images into a video" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}