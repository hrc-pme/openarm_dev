/**
 * PointCloud Preprocessor Node (ROS 2)
 * ------------------------------------------------
 * 功能：
 * 1. 訂閱原始的 RealSense 點雲 Topic。
 * 2. 使用您既有的 C++ 邏輯進行座標轉換與 Z-Order 過濾。
 * 3. 將處理後的點雲發布到新的 Topic，供下游 CBF 節點使用。
 */

#include <iostream>
#include <memory>
#include <atomic>
#include <mutex>
#include <vector>
#include <cmath>
#include <algorithm>
#include <csignal>

// ROS 2 Headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
 
#include <vamp/robots/koch.hh>
#include <vamp/vector.hh>

// ================= ⚙️ 參數設定 =================
// [避障參數]
constexpr float FILTER_RADIUS = 0.02f;         // 點雲過濾半徑

constexpr size_t PREALLOC_SIZE = 50000; 

// ================= 🤖 類型定義 =================
using Robot = vamp::robots::Koch;
using Configuration = Robot::Configuration;

// ================= 🧠 Morton Code (保留用於排序) =================
inline uint32_t expand_bits(uint32_t v) {
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

inline uint32_t morton_3d(float x, float y, float z) {
    float min_val = -1.0f; float max_val = 1.0f;
    float scale = 1023.0f / (max_val - min_val);
    auto to_int = [&](float val) -> uint32_t {
        float norm = (val - min_val) * scale;
        if (norm < 0) return 0; if (norm > 1023) return 1023;
        return static_cast<uint32_t>(norm);
    };
    uint32_t xx = expand_bits(to_int(x));
    uint32_t yy = expand_bits(to_int(y));
    uint32_t zz = expand_bits(to_int(z));
    return xx | (yy << 1) | (zz << 2);
}

// ================= ☁️ 點雲處理 =================
struct Point3D { float x, y, z; uint32_t code; };

std::atomic<bool> g_running{true};

// Z-Order 過濾器
std::vector<Point3D> filter_z_order(std::vector<Point3D>& input, float r_filter) {
    if (input.empty()) return {};
    std::sort(input.begin(), input.end(), [](const auto& a, const auto& b) { return a.code < b.code; });
    std::vector<Point3D> output;
    output.reserve(input.size());
    output.push_back(input[0]);
    float r_sq = r_filter * r_filter;
    for (size_t i = 1; i < input.size(); ++i) {
        const auto& p = input[i];
        const auto& last = output.back();
        float dx = p.x - last.x; float dy = p.y - last.y; float dz = p.z - last.z;
        if (dx*dx + dy*dy + dz*dz > r_sq) output.push_back(p);
    }
    return output;
}

// [相機設定]
constexpr float CAM_X_OFFSET = 0.30f;
constexpr float CAM_Y_OFFSET = 0.00f;
constexpr float CAM_Z_OFFSET = 0.75f; // 攝影機安裝高度
constexpr size_t CAPTURE_STEP = 20;
constexpr float FLOOR_THRESHOLD = 0.01f;  // 0.01f -> 0.05f (提高地板过滤高度)


// [ROS 2 PointCloud Preprocessor Node]
class PointCloudPreprocessorNode : public rclcpp::Node {
public:
    PointCloudPreprocessorNode() : Node("pointcloud_preprocessor_node") {
        // 🔴 關鍵修正：使用 SensorDataQoS，使其與 Python 訂閱端的 QoS 匹配
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cbf/filtered_points", qos_profile);

        // 🔴 關鍵修正：將訂閱的 QoS profile 改為與 RealSense 發布端 (預設為 RELIABLE) 匹配。
        // SensorDataQoS() 是 BEST_EFFORT，會導致無法接收到訊息。
        rclcpp::QoS subscription_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // 預設為 RELIABLE
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera1/depth/color/points", subscription_qos_profile,
            std::bind(&PointCloudPreprocessorNode::pointcloud_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "✅ C++ PointCloud Preprocessor is running.");
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!g_running) return;

        // 🔴 新增日誌：用於第一時間確認是否有收到資料。
        // 如果連這行都沒印出來，代表 QoS 設定不匹配或 Topic 名稱錯誤。
        RCLCPP_INFO_ONCE(
            this->get_logger(),
            "📥 First PointCloud message received! Now processing..."
        );
        
        std::vector<Point3D> candidates;
        candidates.reserve(msg->width * msg->height / CAPTURE_STEP + 1);
        
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
            
            // 🔴 終極修正：根據「頂視」相機的物理架設重寫座標轉換
            // RealSense(x, y, z) -> Robot(x_f, y_f, z_f)
            // 假設：相機由上往下看，影像的「下方」對應機器人的「前方」。
            //       (X_cam -> -Y_robot, Y_cam -> +X_robot, Z_cam -> -Z_robot)
            //
            // 機器人 X (前後) = 相機安裝位置 X + 相機影像 Y
            // 機器人 Y (左右) = 相機安裝位置 Y + 相機影像 X
            // 機器人 Z (上下) = 相機安裝位置 Z - 相機影像 Z
            float x_f = CAM_X_OFFSET + y;
            // 🔴 修正：將 `- x` 改為 `+ x`。
            // 這可以修正座標系從右手系被錯誤地轉換為左手系（鏡像）的問題。
            float y_f = CAM_Y_OFFSET + x;
            float z_f = CAM_Z_OFFSET - z;
            
            // 過濾掉高度太低（地板）的點，並只保留相機前方 50cm 內的點雲
            if (z_f > FLOOR_THRESHOLD && z <= 0.5f) {
                candidates.push_back({x_f, y_f, z_f, morton_3d(x_f, y_f, z_f)});
            }
        }
        
        auto filtered = filter_z_order(candidates, FILTER_RADIUS);

        // 🔴 新增日誌：在終端機印出第一個轉換後的點座標，用於驗證
        if (!filtered.empty()) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000, // 每 2 秒印一次
                "✅ Processed %zu points. First point transformed to: (x=%.3f, y=%.3f, z=%.3f)",
                filtered.size(),
                filtered[0].x,
                filtered[0].y,
                filtered[0].z
            );
        }
        // 將處理後的點雲轉換回 PointCloud2 格式並發布
        sensor_msgs::msg::PointCloud2 output_msg;
        output_msg.header = msg->header; // 保持原始的時間戳和 frame_id
        output_msg.header.frame_id = "base_link"; // 由於我們做了座標轉換，frame_id 應設為基準座標系
        output_msg.height = 1;
        output_msg.width = filtered.size();
        output_msg.is_dense = true;
        
        sensor_msgs::PointCloud2Modifier modifier(output_msg);
        modifier.setPointCloud2Fields(3, 
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(filtered.size());

        sensor_msgs::PointCloud2Iterator<float> iter_out_x(output_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_out_y(output_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_out_z(output_msg, "z");

        for (const auto& pt : filtered) {
            *iter_out_x = pt.x;
            *iter_out_y = pt.y;
            *iter_out_z = pt.z;
            ++iter_out_x; ++iter_out_y; ++iter_out_z;
        }

        publisher_->publish(output_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};


int main(int argc, char** argv) {
    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT, [](int) { g_running = false; rclcpp::shutdown(); });
    
    std::cout << ">>> Koch VAMP Server V149 [ROS 2 PointCloud Active] <<<" << std::endl;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudPreprocessorNode>();
    rclcpp::spin(node);
    
    g_running = false;
    rclcpp::shutdown();
    return 0;
}
