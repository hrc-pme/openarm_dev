/**
 * ROS 2 Wrapper for Koch VAMP Server V149
 */
#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// VAMP headers
#include <vamp/robots/koch.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/xorshift.hh>
#include <vamp/vector.hh>

// ... I will copy the rest of koch_vamp_server.cc and replace capture_thread with a ROS subscriber ...
