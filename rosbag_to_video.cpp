#include <stdio.h>
#include <algorithm>
#include <string>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
// #include <boost/lexical_cast.hpp>



// bag_to_images(bag_path, topic_name, save_path="output_dir")
//  * Open bag file and save all images of a certain topic
//  * Return average/median fps (TODO median)
//  * check if bag exists
//  * watch for CvBridge copy vs shared ptr and conversion types
bool bag_to_images(const boost::filesystem::path & bag_path,
                   const std::string & topic_name,
                   boost::filesystem::path & output_path,
                   float & fps) {
                //    boost::filesystem::path output_dir=""){

    // Determine output location for extracted images
    output_path = bag_path.parent_path() / bag_path.stem();
    if(!boost::filesystem::create_directory(output_path)){
        std::cout << "Unable to create output directory" << std::endl;
        return false;
    }
    
    // Open ROSbag for message processing
    if(!boost::filesystem::exists(bag_path)){
        std::cout << "Error: Bag file does not exist" << std::endl;
        throw std::invalid_argument("Input bagfile does not exist");
    }
    rosbag::Bag bag;
    try{
        bag.open(bag_path.string());  // read-only by default
    }
    catch (rosbag::BagException & e){
        std::cout << "Error: " << e.what()
                  << "\nUnable to open bag file: " << bag_path << std::endl;
        return false;
    }
    
    // Save image messages to disk
    rosbag::View view(bag, rosbag::TopicQuery(topic_name));
    int image_count = 0;
    ros::Time earliest, latest;
    for(rosbag::MessageInstance const message: view) {
        sensor_msgs::ImageConstPtr ros_image = 
                message.instantiate<sensor_msgs::Image>();
        if(ros_image != nullptr){
            // Convert ROS Image message to OpenCV format
            cv_bridge::CvImageConstPtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvShare(ros_image,
                    sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e){
                std::cout << "Error: " << e.what()
                            << "Unable to convert image message to OpenCV format"
                            << std::endl;
                continue;
            }

            // Save image to disk
            std::string filename = std::to_string(image_count) + ".png";
            // std::string filename = boost::lexical_cast<std::string>(image_count)
            //                        + ".png";
            cv::imwrite(filename, cv_ptr->image);
            image_count++;
            
            // Track earliest and latest message timestamps for framerate
            earliest = std::min(earliest, ros_image->header.stamp);
            latest = std::max(latest, ros_image->header.stamp);
        }
    }
    bag.close();

    // Calculate average frame rate
    if(image_count > 1){
        double duration = (latest - earliest).toSec();
        fps = image_count / duration;
        return true;
    }
    else if(image_count == 1){
        // Arbitrary fps if only a single image is extracted
        fps = 1.0;
        return true;
    }
    return false;

}

// images_to_video(input_dir, fps=15, output_file='vid.mp4')
//  * Check if input folder exists and is non-empty
//  * Use OpenCV to create
bool images_to_video(const boost::filesystem::path & output_path,
                     const float & fps){
    // TODO
    return false;
}

// Main
//  * Parse inputs (bag path, topic name)
//  * Provide 'help' message
//  * Run bag_to_images(), images_to_video()
//  * Debayering?
//  * Delete image folder
int main(int argc, char * argv[])
{
    // Parse command line arguments
    if(argc != 3){
        std::cout << "Incorrect command line usage. Expected usage:\n\t"
                  << argv[0] << " /path_to_bag/bag_name.bag /image_topic_name"
                  << std::endl;
    }

    boost::filesystem::path bag_path (argv[1]), images_path;
    std::string topic_name (argv[2]);
    float fps;
    
    if(!bag_to_images(bag_path, topic_name, images_path, fps)){
        std::cout << "Unable to extract and save ROSbag images" << std::endl;
        return 1;
    }
    if(!images_to_video(images_path, fps)){
        std::cout << "Unable to convert saved images into a video" << std::endl;
    }

    return 0;
}