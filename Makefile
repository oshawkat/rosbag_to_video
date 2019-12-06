# Initial makefile sourced from: https://github.com/gerkey/ros1_external_use

all: extract_video

ros_libs = $(shell pkg-config --libs roscpp rosbag std_msgs sensor_msgs)
# There are some incompatibility issues with some versions of pkg-config
# and some linkers regarding the -l:/absolute/path syntax. As a quick
# workaround, we'll remove the leading -l: prefix, leaving just the
# absolute path.
ros_libs_nocolon = $(subst -l:,,$(ros_libs))

extract_video: extract_video.o rosbag_to_video.o
	$(CXX) -std=c++11 -Wall -o $@ $^ `pkg-config --cflags roscpp rosbag std_msgs sensor_msgs opencv` $< $(ros_libs_nocolon) `pkg-config --libs opencv`

extract_video.o: extract_video.cpp
	$(CXX) -std=c++11 -Wall -o $@ `pkg-config --cflags roscpp rosbag std_msgs sensor_msgs opencv` $< $(ros_libs_nocolon) `pkg-config --libs opencv`

rosbag_to_video.o: rosbag_to_video.cpp
	$(CXX) -std=c++11 -Wall -c $@ `pkg-config --cflags roscpp rosbag std_msgs sensor_msgs opencv` $< $(ros_libs_nocolon) `pkg-config --libs opencv`
	# $(CXX) -std=gnu++11 -Wall -o $@ `pkg-config --cflags roscpp std_msgs sensor_msgs opencv` $< $(ros_libs_nocolon) `pkg-config --libs opencv`
	# $(CXX) -Wall -o $@ `pkg-config --cflags roscpp std_msgs sensor_msgs opencv` $< $(ros_libs_nocolon) `pkg-config --libs opencv`

clean:
	rm -f rosbag_to_video
