/**
 * Koch VAMP Server V148 (Virtual Obstacle Injection)
 * ------------------------------------------------
 * 功能：
 * 1. [Virtual Wall] 不使用 RealSense，而是生成一道虛擬牆 (X=0.25m)。
 * 2. [Full Pipeline] 保留 Z-Order Filter、SIMD Broadcast、Padding 邏輯。
 * 3. 用於測試實體手臂是否能正確避開 Server 想像出來的障礙物。
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

// 雖然不讀相機，但保留 include 以免編譯錯誤 (如果 CMakeList 沒改)
#include <librealsense2/rs.hpp> 
#include <vamp/robots/koch.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/xorshift.hh>
#include <vamp/vector.hh>

// ================= ⚙️ 參數設定 =================
constexpr const char* SOCKET_PATH = "/tmp/koch_vamp_server.sock";

// [避障參數]
constexpr float FILTER_RADIUS = 0.02f;         // 點雲過濾半徑
constexpr float PHYSICAL_MARGIN = 0.03f;       // 0.015f -> 0.03f (加大物理安全邊距)
constexpr float OBSTACLE_TOTAL_RADIUS = FILTER_RADIUS + PHYSICAL_MARGIN;  // 總計 0.05f
constexpr float SELF_FILTER_PADDING = 0.15f;   // 手臂安全範圍    

constexpr size_t PREALLOC_SIZE = 50000; 

// ================= 🤖 類型定義 =================
using Robot = vamp::robots::Koch;
using Configuration = Robot::Configuration;
constexpr std::size_t rake = vamp::FloatVectorWidth;
using Environment = vamp::collision::Environment<vamp::FloatVector<rake>>;
using Planner = vamp::planning::RRTC<Robot, rake, Robot::resolution>;

struct __attribute__((packed)) PlanRequest { 
    float start[7]; float goal[7]; uint32_t max_iter; float range; 
};

struct __attribute__((packed)) PlanResponse { 
    uint8_t success; uint8_t padding[3]; uint32_t path_size; 
    uint32_t cloud_size; uint32_t sphere_count; uint32_t time_ms; 
};

struct SphereData { float x, y, z, r; };

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
class PointCloudBuffer {
public:
    struct Point3D { float x, y, z; uint32_t code; };
private:
    std::mutex mutex_;
    std::vector<Point3D> points_;
public:
    PointCloudBuffer() { points_.reserve(PREALLOC_SIZE); }
    void update(const std::vector<Point3D>& new_points) { 
        std::lock_guard<std::mutex> lock(mutex_); 
        points_ = new_points; 
    }
    std::vector<Point3D> get_copy() { 
        std::lock_guard<std::mutex> lock(mutex_); 
        return points_; 
    }
};

PointCloudBuffer g_pointcloud_buffer;
std::atomic<bool> g_running{true};

// Z-Order 過濾器
std::vector<PointCloudBuffer::Point3D> filter_z_order(std::vector<PointCloudBuffer::Point3D>& input, float r_filter) {
    if (input.empty()) return {};
    std::sort(input.begin(), input.end(), [](const auto& a, const auto& b) { return a.code < b.code; });
    std::vector<PointCloudBuffer::Point3D> output;
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
constexpr float CAM_X_OFFSET = 0.46f;
constexpr float CAM_Y_OFFSET = 0.00f;
constexpr float CAM_Z_OFFSET = 0.12f;
constexpr size_t CAPTURE_STEP = 20;
constexpr float FLOOR_THRESHOLD = 0.01f;  // 0.01f -> 0.05f (提高地板过滤高度)

void capture_thread() {
    std::cout << "[V149] RealSense Camera Active" << std::endl;
    while (g_running) {
        try {
            rs2::pipeline pipe;
            rs2::config rs_cfg;
            rs_cfg.enable_device("319522065801");
            rs_cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
            pipe.start(rs_cfg);
            rs2::pointcloud pc;
            std::vector<PointCloudBuffer::Point3D> candidates;
            candidates.reserve(PREALLOC_SIZE);
            
            for (int i = 0; i < 30; ++i) pipe.wait_for_frames();
            
            while (g_running) {
                auto frames = pipe.wait_for_frames();
                auto depth = frames.get_depth_frame();
                if (!depth) continue;
                auto points = pc.calculate(depth);
                auto vertices = points.get_vertices();
                size_t count = points.size();
                candidates.clear();
                
                for (size_t i = 0; i < count; i += CAPTURE_STEP) {
                    if (i >= count) break;
                    auto v = vertices[i];
                    if (!std::isfinite(v.x) || v.z <= 0.15f) continue;
                    
                    float y_f = CAM_X_OFFSET - v.z;
                    float x_f = -v.x + CAM_Y_OFFSET;
                    float z_f = -v.y + CAM_Z_OFFSET;
                    
                    // 過濾條件：前方 + 地板 + 左側範圍
                    if (y_f >= 0.01f && z_f > FLOOR_THRESHOLD && x_f >= -0.20f) {
                        candidates.push_back({x_f, y_f, z_f, morton_3d(x_f, y_f, z_f)});
                    }
                }
                auto filtered = filter_z_order(candidates, FILTER_RADIUS);
                g_pointcloud_buffer.update(filtered);
            }
            pipe.stop();
            break;
        } catch (const std::exception& e) {
            std::cerr << "Camera error: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

// ... (以下邏輯與 V147 完全相同) ...

std::vector<PointCloudBuffer::Point3D> filter_robot_body(const std::vector<PointCloudBuffer::Point3D>& raw, const Configuration& cfg, std::vector<SphereData>& sph_out) {
    std::vector<PointCloudBuffer::Point3D> filtered;
    filtered.reserve(raw.size());
    Robot::Spheres<1> rs; Robot::ConfigurationBlock<1> cb;
    auto arr = cfg.to_array(); for(size_t i=0; i<6; ++i) cb[i].data[0][0] = arr[i]; 
    Robot::sphere_fk(cb, rs);
    sph_out.clear();
    sph_out.reserve(Robot::n_spheres);
    for (size_t i = 0; i < Robot::n_spheres; ++i) 
        sph_out.push_back({rs.x[i].data[0][0], rs.y[i].data[0][0], rs.z[i].data[0][0], rs.r[i].data[0][0]});
    for (const auto& pt : raw) {
        bool is_self = false;
        for (size_t i = 0; i < Robot::n_spheres; ++i) {
            float rr = rs.r[i].data[0][0] + SELF_FILTER_PADDING;
            float dx = pt.x - rs.x[i].data[0][0]; 
            float dy = pt.y - rs.y[i].data[0][0]; 
            float dz = pt.z - rs.z[i].data[0][0];
            if (dx*dx + dy*dy + dz*dz < rr*rr) { is_self = true; break; }
        }
        if (!is_self) filtered.push_back(pt);
    }
    return filtered;
}

bool is_strictly_colliding_debug(const Configuration& cfg, const std::vector<PointCloudBuffer::Point3D>& pts, int& hitting_sphere) {
    Robot::Spheres<1> rs; Robot::ConfigurationBlock<1> cb;
    auto arr = cfg.to_array(); for(size_t i=0; i<6; ++i) cb[i].data[0][0] = arr[i]; 
    Robot::sphere_fk(cb, rs);
    for (size_t i = 1; i < Robot::n_spheres; ++i) { 
        for (const auto& pt : pts) {
            float dx = rs.x[i].data[0][0] - pt.x; float dy = rs.y[i].data[0][0] - pt.y; float dz = rs.z[i].data[0][0] - pt.z;
            float safe = rs.r[i].data[0][0] + OBSTACLE_TOTAL_RADIUS; 
            if (dx*dx + dy*dy + dz*dz < safe * safe) { hitting_sphere = (int)i; return true; }
        }
    }
    return false;
}

template <typename V> void broadcast_simd(V& vec, float val) { for (size_t i = 0; i < rake; ++i) vec[i] = val; }

bool plan_path(const float start[7], const float goal[7], const std::vector<PointCloudBuffer::Point3D>& raw, uint32_t iter, float rng_val, std::vector<std::array<float, 6>>& path_out, std::vector<PointCloudBuffer::Point3D>& f_obs, std::vector<SphereData>& sph_out, uint32_t& ms) {
    auto t0 = std::chrono::high_resolution_clock::now();
    Configuration sc(std::array<float, 6>{start[0], start[1], start[2], start[3], start[4], start[5]});
    Configuration gc(std::array<float, 6>{goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]});
    f_obs = filter_robot_body(raw, sc, sph_out);
    
    // 使用真實點雲障礙物
    std::vector<PointCloudBuffer::Point3D> all_obs = f_obs;
    
    Environment env; env.spheres.reserve(all_obs.size());
    for (const auto& pt : all_obs) {
        vamp::FloatVector<rake> vx, vy, vz, vr;
        broadcast_simd(vx, pt.x); broadcast_simd(vy, pt.y); broadcast_simd(vz, pt.z); broadcast_simd(vr, OBSTACLE_TOTAL_RADIUS); 
        env.spheres.push_back(vamp::collision::Sphere<vamp::FloatVector<rake>>{vx, vy, vz, vr});
    }
    
    // 🔍 显示起点和终点的末端执行器位置
    Robot::Spheres<1> ee_spheres_start, ee_spheres_goal;
    Robot::ConfigurationBlock<1> cb_s, cb_g;
    for(size_t i=0; i<6; ++i) { 
        cb_s[i].data[0][0] = sc.to_array()[i];
        cb_g[i].data[0][0] = gc.to_array()[i];
    }
    Robot::sphere_fk(cb_s, ee_spheres_start);
    Robot::sphere_fk(cb_g, ee_spheres_goal);
    
    // 末端执行器是最后一个球（索引17）
    float start_ee_x = ee_spheres_start.x[17].data[0][0];
    float start_ee_y = ee_spheres_start.y[17].data[0][0];
    float start_ee_z = ee_spheres_start.z[17].data[0][0];
    
    float goal_ee_x = ee_spheres_goal.x[17].data[0][0];
    float goal_ee_y = ee_spheres_goal.y[17].data[0][0];
    float goal_ee_z = ee_spheres_goal.z[17].data[0][0];
    
    std::cout << "📍 [Motion] Start EE: [" << std::fixed << std::setprecision(3) 
              << start_ee_x << ", " << start_ee_y << ", " << start_ee_z << "] → Goal EE: ["
              << goal_ee_x << ", " << goal_ee_y << ", " << goal_ee_z << "]" << std::endl;

    int hitting_id = -1;
    // 🧪 [Test] 暫時跳過起始位置檢查，讓規劃器處理
    // if (is_strictly_colliding_debug(sc, all_obs, hitting_id)) { std::cout << "⚠️ [Start Blocked] Sphere #" << hitting_id << std::endl; return false; }
    if (is_strictly_colliding_debug(gc, all_obs, hitting_id)) { std::cout << "🚫 [Goal Blocked]" << std::endl; return false; }

    std::array<float, 6> mid_arr; for(int i=0; i<6; ++i) mid_arr[i] = (sc.to_array()[i] + gc.to_array()[i]) * 0.5f;
    Configuration mid(mid_arr); int hit_s = -1;
    bool is_direct_blocked = is_strictly_colliding_debug(mid, all_obs, hit_s);
    if (is_direct_blocked) std::cout << "🛡️ [Obstacle] Direct Path Blocked. RRT active." << std::endl;

    vamp::planning::RRTCSettings settings;
    settings.max_iterations = std::max(iter, (uint32_t)100000); settings.range = 0.02f; 
    auto result = Planner::solve(sc, gc, env, settings, std::make_shared<vamp::rng::XORShift<Robot>>());
    ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t0).count();
    
    if (result.path.empty()) { 
        std::cout << "❌ [RRT Failed] No path found (Time: " << ms << "ms, Obstacles: " << all_obs.size() << ")" << std::endl; 
        return false; 
    }
    if (is_direct_blocked && result.path.size() <= 2) { 
        std::cout << "❌ [Safety Reject] RRT returned straight path (Nodes: " << result.path.size() << ")" << std::endl; 
        return false; 
    }

    std::cout << "✅ [RRT Success] Nodes: " << result.path.size() << " (Time: " << ms << "ms)" << std::endl;
    for (const auto& c : result.path) { auto a = c.to_array(); path_out.push_back({a[0],a[1],a[2],a[3],a[4],a[5]}); }
    return true;
}

bool send_all(int fd, const void* data, size_t size) {
    const uint8_t* ptr = (const uint8_t*)data;
    size_t remaining = size;
    while (remaining > 0) {
        ssize_t sent = send(fd, ptr, remaining, MSG_NOSIGNAL);
        if (sent < 0) return false; ptr += sent; remaining -= sent;
    }
    return true;
}

void socket_server_thread() {
    int sfd = socket(AF_UNIX, SOCK_STREAM, 0); unlink(SOCKET_PATH);
    struct sockaddr_un addr; memset(&addr, 0, sizeof(addr)); addr.sun_family = AF_UNIX; strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path)-1);
    bind(sfd, (struct sockaddr*)&addr, sizeof(addr)); listen(sfd, 5);
    std::cout << ">>> Server Ready (V149)." << std::endl;
    while (g_running) {
        int cfd = accept(sfd, nullptr, nullptr); if (cfd < 0) continue;
        PlanRequest req; ssize_t bytes_read = recv(cfd, &req, sizeof(req), 0);
        if (bytes_read == sizeof(req)) {
            float start_aligned[7], goal_aligned[7];
            std::memcpy(start_aligned, req.start, sizeof(start_aligned)); std::memcpy(goal_aligned, req.goal, sizeof(goal_aligned));
            auto r = g_pointcloud_buffer.get_copy(); 
            std::vector<std::array<float, 6>> path; std::vector<PointCloudBuffer::Point3D> fo; std::vector<SphereData> rs; uint32_t ms;
            bool ok = plan_path(start_aligned, goal_aligned, r, req.max_iter, req.range, path, fo, rs, ms);
            PlanResponse resp = { (uint8_t)(ok?1:0), {0,0,0}, (uint32_t)path.size(), (uint32_t)fo.size(), (uint32_t)rs.size(), ms };
            if (!send_all(cfd, &resp, sizeof(resp))) { close(cfd); continue; }
            if(ok) if (!send_all(cfd, path.data(), path.size() * 24)) { close(cfd); continue; }
            std::vector<float> buffer; buffer.reserve(fo.size() * 3);
            for(auto& p : fo) { buffer.push_back(p.x); buffer.push_back(p.y); buffer.push_back(p.z); }
            if (!send_all(cfd, buffer.data(), buffer.size() * 4)) { close(cfd); continue; }
            buffer.clear(); buffer.reserve(rs.size() * 4);
            for(auto& s : rs) { buffer.push_back(s.x); buffer.push_back(s.y); buffer.push_back(s.z); buffer.push_back(s.r); }
            if (!send_all(cfd, buffer.data(), buffer.size() * 4)) { close(cfd); continue; }
        } 
        close(cfd);
    }
}

int main() {
    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT, [](int) { g_running = false; });
    std::cout << ">>> Koch VAMP Server V149 [RealSense Active] <<<" << std::endl;
    std::thread rs_t(capture_thread); 
    std::this_thread::sleep_for(std::chrono::seconds(1));
    socket_server_thread();
    rs_t.join();
    return 0;
}