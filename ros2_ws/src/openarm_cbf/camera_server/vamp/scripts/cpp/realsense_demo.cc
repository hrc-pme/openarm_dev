#include <iostream>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <vector>
#include <chrono>

int main() {
    std::cout << "🎥 RealSense C++ Demo" << std::endl;

    try {
        // 1. 建立 RealSense context
        rs2::context ctx;
        auto devices = ctx.query_devices();
        
        if (devices.size() == 0) {
            std::cerr << "❌ No RealSense device detected!" << std::endl;
            return 1;
        }

        std::cout << "✅ Found " << devices.size() << " device(s)" << std::endl;

        // 2. 取得第一個設備資訊
        rs2::device dev = devices[0];
        std::cout << "Device: " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
        std::cout << "Serial: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        std::cout << "FW Ver: " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;

        // 3. 建立 pipeline
        rs2::pipeline pipe;
        rs2::config cfg;
        
        // 設定 640x480 @ 30Hz
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);

        std::cout << "\n🚀 Starting pipeline..." << std::endl;
        auto profile = pipe.start(cfg);

        // 取得深度感測器的尺度 (depth units)
        auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
        float depth_scale = depth_sensor.get_depth_scale();
        std::cout << "Depth scale: " << depth_scale << " meters/unit" << std::endl;

        // 4. 擷取幾幀測試
        std::cout << "\n📸 Capturing frames..." << std::endl;
        
        for (int i = 0; i < 30; ++i) {
            pipe.wait_for_frames();  // 等待自動曝光穩定
        }

        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < 100; ++i) {
            rs2::frameset frames = pipe.wait_for_frames();
            
            rs2::depth_frame depth = frames.get_depth_frame();
            rs2::video_frame color = frames.get_color_frame();
            
            if (i % 20 == 0) {
                int w = depth.get_width();
                int h = depth.get_height();
                
                // 讀取中心點的深度
                float center_depth = depth.get_distance(w/2, h/2);
                
                std::cout << "Frame " << i << ": "
                          << "Size=" << w << "x" << h 
                          << ", Center depth=" << center_depth << "m" << std::endl;
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::cout << "\n⏱️  Captured 100 frames in " << duration.count() << " ms" << std::endl;
        std::cout << "   Average FPS: " << 100000.0 / duration.count() << std::endl;

        // 5. 取得點雲範例 (單幀)
        std::cout << "\n☁️  Converting depth to point cloud..." << std::endl;
        
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();
        
        // 建立點雲濾波器
        rs2::pointcloud pc;
        rs2::points points = pc.calculate(depth);
        
        auto vertices = points.get_vertices();
        size_t point_count = points.size();
        
        std::cout << "Point cloud contains " << point_count << " points" << std::endl;
        
        // 打印前10個有效點
        std::cout << "\nFirst 10 valid points (x, y, z):" << std::endl;
        int valid_count = 0;
        for (size_t i = 0; i < point_count && valid_count < 10; ++i) {
            auto v = vertices[i];
            if (v.z > 0) {  // 過濾無效點
                std::cout << "  [" << valid_count << "]: (" 
                          << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
                valid_count++;
            }
        }

        // 6. 停止 pipeline
        pipe.stop();
        std::cout << "\n✅ Demo complete!" << std::endl;

    } catch (const rs2::error & e) {
        std::cerr << "❌ RealSense error: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception & e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
