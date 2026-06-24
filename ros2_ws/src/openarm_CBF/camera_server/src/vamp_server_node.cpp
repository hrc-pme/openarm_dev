/**
 * VAMP Server ROS2 Node Wrapper
 * 功能：啟動 koch_vamp_server 並監控其狀態
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <memory>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>

class VampServerNode : public rclcpp::Node
{
public:
    VampServerNode() : Node("vamp_server_node"), server_pid_(-1)
    {
        RCLCPP_INFO(this->get_logger(), "🚀 VAMP Server Node Starting...");
        
        // 參數
        this->declare_parameter<std::string>("socket_path", "/tmp/koch_vamp_server.sock");
        this->declare_parameter<bool>("auto_start_server", true);
        this->declare_parameter<int>("status_check_interval_ms", 5000);
        
        socket_path_ = this->get_parameter("socket_path").as_string();
        auto_start_ = this->get_parameter("auto_start_server").as_bool();
        int interval = this->get_parameter("status_check_interval_ms").as_int();
        
        // 發布器 (未來可發布點雲、碰撞球等)
        status_pub_ = this->create_publisher<std_msgs::msg::String>("vamp_server/status", 10);
        
        // 定時器：檢查 server 狀態
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(interval),
            std::bind(&VampServerNode::check_server_status, this));
        
        // 啟動 server
        if (auto_start_) {
            start_vamp_server();
        }
        
        RCLCPP_INFO(this->get_logger(), "✅ VAMP Server Node Ready");
    }
    
    ~VampServerNode()
    {
        stop_vamp_server();
    }

private:
    void start_vamp_server()
    {
        RCLCPP_INFO(this->get_logger(), "🔧 Starting koch_vamp_server process...");
        
        // 清理舊的 socket
        unlink(socket_path_.c_str());
        
        server_pid_ = fork();
        
        if (server_pid_ == 0) {
            // 子進程：執行 koch_vamp_server
            execlp("ros2", "ros2", "run", "canera_server", "koch_vamp_server", nullptr);
            // 如果 execlp 失敗
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to exec koch_vamp_server");
            exit(1);
        } else if (server_pid_ > 0) {
            RCLCPP_INFO(this->get_logger(), "✅ koch_vamp_server started (PID: %d)", server_pid_);
            // 等待 socket 創建
            for (int i = 0; i < 20; ++i) {
                if (access(socket_path_.c_str(), F_OK) == 0) {
                    RCLCPP_INFO(this->get_logger(), "🔗 Socket ready: %s", socket_path_.c_str());
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            RCLCPP_WARN(this->get_logger(), "⚠️  Socket not detected after 10s");
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to fork() server process");
        }
    }
    
    void stop_vamp_server()
    {
        if (server_pid_ > 0) {
            RCLCPP_INFO(this->get_logger(), "🛑 Stopping koch_vamp_server (PID: %d)", server_pid_);
            kill(server_pid_, SIGINT);
            
            // 等待進程結束
            int status;
            waitpid(server_pid_, &status, 0);
            
            server_pid_ = -1;
            unlink(socket_path_.c_str());
        }
    }
    
    void check_server_status()
    {
        std_msgs::msg::String msg;
        
        if (server_pid_ <= 0) {
            msg.data = "STOPPED";
            status_pub_->publish(msg);
            return;
        }
        
        // 檢查進程是否存活
        int result = kill(server_pid_, 0);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Server process died! Restarting...");
            server_pid_ = -1;
            start_vamp_server();
            msg.data = "RESTARTING";
        } else {
            msg.data = "RUNNING";
        }
        
        status_pub_->publish(msg);
    }
    
    std::string socket_path_;
    bool auto_start_;
    pid_t server_pid_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VampServerNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
