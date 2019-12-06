#include "rosbag_to_video.h"

#include <stdio.h>

#include <string>

#include <boost/filesystem.hpp>

// Main
//  * Parse inputs (bag path, topic name)
//  * Provide 'help' message
//  * Run bag_to_images(), images_to_video()
//  * Debayering?
//  * Delete image folder
int main(int argc, char * argv[]) {
    // Parse command line arguments
    if(argc != 3) {
        std::cout << "Incorrect command line usage. Expected usage:\n\t"
                  << argv[0] << " /path_to_bag/bag_name.bag /image_topic_name"
                  << std::endl;
    }
    boost::filesystem::path bag_path (argv[1]), images_path;
    std::string topic_name (argv[2]);
    float fps;
    
    if(!bag_to_video::extract_bag_to_images(bag_path, topic_name, images_path,
                                           fps)) {
        std::cout << "Unable to extract and save ROSbag images" << std::endl;
        return EXIT_FAILURE;
    }
    if(!bag_to_video::images_to_video(images_path, fps)) {
        std::cout << "Unable to convert saved images into a video" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}