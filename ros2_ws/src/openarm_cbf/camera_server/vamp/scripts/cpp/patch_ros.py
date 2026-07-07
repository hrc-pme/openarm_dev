import re

with open("koch_vamp_server.cc", "r") as f:
    code = f.read()

# Replace #include <librealsense2/rs.hpp> with ROS headers
code = code.replace("#include <librealsense2/rs.hpp>", """
// ROS 2 Headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
""")

# Replace capture_thread
capture_thread_pattern = r'void capture_thread\(\) \{.*?(?=// \.\.\. \(以下邏輯與 V147 完全相同\))'
capture_replacement = """
// [ROS 2 Subscriber Node for PointCloud]
class CameraSubscriberNode : public rclcpp::Node {
public:
    CameraSubscriberNode() : Node("koch_vamp_server_node") {
        RCLCPP_INFO(this->get_logger(), "[V149] ROS 2 PointCloud Subscriber Active (Listening to /camera/camera1/depth/color/points)");
        
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera1/depth/color/points", rclcpp::SensorDataQoS(),
            std::bind(&CameraSubscriberNode::pointcloud_callback, this, std::placeholders::_1));
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!g_running) return;
        
        std::vector<PointCloudBuffer::Point3D> candidates;
        candidates.reserve(PREALLOC_SIZE);
        
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        
        size_t count = 0;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++count) {
            if (count % CAPTURE_STEP != 0) continue;
            
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;
            
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z <= 0.15f) continue;
            
            // 轉換邏輯與 RealSense 一致
            float y_f = CAM_X_OFFSET - z;
            float x_f = -x + CAM_Y_OFFSET;
            float z_f = -y + CAM_Z_OFFSET;
            
            if (y_f >= 0.01f && z_f > FLOOR_THRESHOLD && x_f >= -0.20f) {
                candidates.push_back({x_f, y_f, z_f, morton_3d(x_f, y_f, z_f)});
            }
        }
        
        auto filtered = filter_z_order(candidates, FILTER_RADIUS);
        g_pointcloud_buffer.update(filtered);
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
"""

code = re.sub(capture_thread_pattern, capture_replacement, code, flags=re.DOTALL)

# Replace main function
main_pattern = r'int main\(\) \{.*\}'
main_replacement = """
int main(int argc, char** argv) {
    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT, [](int) { g_running = false; rclcpp::shutdown(); });
    
    std::cout << ">>> Koch VAMP Server V149 [ROS 2 PointCloud Active] <<<" << std::endl;
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraSubscriberNode>();
    
    // 開啟 Socket Server Thread
    std::thread socket_t(socket_server_thread);
    
    // 讓 ROS 2 spin
    rclcpp::spin(node);
    
    g_running = false;
    socket_t.join();
    
    return 0;
}
"""

code = re.sub(main_pattern, main_replacement, code, flags=re.DOTALL)

with open("koch_vamp_server.cc", "w") as f:
    f.write(code)

