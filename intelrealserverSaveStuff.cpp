// intelrealserver.cpp

#include <assert.h>

#include <chrono>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <thread>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "cameraserver.h"

//#define DEBUG(x) std::cout << x << std::endl
#define DEBUG(x)

void cameraThread(bool saveFlag, std::string path) {
    rs2::context ctx;

    std::vector<rs2::pipeline> pipelines;

    for (auto&& dev : ctx.query_devices()) {
        auto serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::cout << "got serial: " << serial << std::endl;

        std::vector<rs2::sensor> sensors = dev.query_sensors();
        int index = 0;
    
        // We can now iterate the sensors and print their names
        for (rs2::sensor sensor : sensors)
        
            if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
                ++index;
                
                std::cout << "  " << index << " : " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
                // get_sensor_option(sensor);
                if (index == 1) {
                //     sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
                //     sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT,50000);
                //     sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1); // emitter on for depth information
                    // sensor.set_option(RS2_OPTION_DEPTH_UNITS, 0.001);
                }
                if (index == 2){
                    // RGB camera
                    sensor.set_option(RS2_OPTION_EXPOSURE,80.f);
                    
                }

                if (index == 3){
                    sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION,0);
                }

            }

        rs2::config cfg;
        cfg.enable_device(serial);
        cfg.enable_stream(RS2_STREAM_DEPTH,640, 480, RS2_FORMAT_Z16);
        cfg.enable_stream(RS2_STREAM_COLOR,1280, 720, RS2_FORMAT_RGB8);

        rs2::pipeline pipe(ctx);
        pipe.start(cfg);
        pipelines.push_back(pipe);

        CameraState::get()->cameras.push_back(0);
    }

    if (pipelines.size() == 0) {
        std::cerr << "no cameras found" << std::endl;
        exit(-1);
    }

    rs2::align alignment(RS2_STREAM_COLOR);
    // rs2::align alignment(RS2_STREAM_DEPTH);

    while (true) {
        auto start = std::chrono::high_resolution_clock::now();

        int num = 0;
        for (auto& p : pipelines) {
            std::shared_ptr<CameraOutput> output(new CameraOutput());
            rs2::frameset frames = p.wait_for_frames();

            // this handles the geometry so that the
            // x/y of the depth and color are the same
            frames = alignment.process(frames);

            // do color frame
            auto vf = frames.get_color_frame();

            assert(vf.get_bytes_per_pixel() == 3);
            assert(vf.get_stride_in_bytes() ==
                   (vf.get_width() * vf.get_bytes_per_pixel()));

            output->width = vf.get_width();
            output->height = vf.get_height();
            output->ppmdata =
                my_write_ppm((const char*)vf.get_data(), vf.get_width(),
                             vf.get_height(), vf.get_bytes_per_pixel());

            // create depth map

            auto depth = frames.get_depth_frame();
            output->add_depth(depth.get_bytes_per_pixel(), depth.get_units(),
                              depth.get_width(), depth.get_height(),
                              (const char*)depth.get_data());

            DEBUG("middle distance: " << depth.get_distance(
                      depth.get_width() / 2, depth.get_height() / 2));

            CameraState::get()->cameras[num++] = output;
            if(saveFlag){
		int width_img=depth.get_width(), height_img=depth.get_height();
            
                cv::Mat im, imDepth;
            im = cv::Mat(cv::Size(width_img,height_img ), CV_8UC3, (void*)(vf.get_data()), cv::Mat::AUTO_STEP);
            imDepth = cv::Mat(cv::Size(width_img, height_img), CV_16U, (void*)(depth.get_data()), cv::Mat::AUTO_STEP);
            char timestampVIAM[100];
            //     std::strftime(timestampVIAM, sizeof(timestampVIAM), "%FT%H_%M_%S", std::gmtime(&t));
            
            // double millisecondTime  = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            // double sec_since_epoch = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            // int currMillis = millisecondTime - sec_since_epoch*1000;
            struct tm* tm_info;
            struct timeval tv;
              gettimeofday(&tv, NULL);

                int millisec = lrint(tv.tv_usec/1000.0); // Round to nearest millisec
                if (millisec>=1000) { // Allow for rounding up to nearest second
                    millisec -=1000;
                    tv.tv_sec++;
                }

                tm_info = localtime(&tv.tv_sec);

                strftime(timestampVIAM, sizeof(timestampVIAM), "%FT%H_%M_%S", tm_info);
                printf("%s.%03d\n", timestampVIAM, millisec);
            // std::replace(timeString.begin(), timeString.end(), ':', '_');
		std::string millisString = std::to_string(millisec);
            std::string filenameRGB = path + "/data/rgb/color_data_" + std::string(timestampVIAM) +"." + millisString  + ".png";
            std::string filenameDEPTH = path + "/data/depth/color_data_" + std::string(timestampVIAM) + "." +  millisString + ".png"; 
            cv::imwrite(filenameRGB,im);
            cv::imwrite(filenameDEPTH,imDepth);
            }
            
        }

        auto finish = std::chrono::high_resolution_clock::now();
        DEBUG(std::chrono::duration_cast<std::chrono::milliseconds>(finish -
                                                                    start)
                  .count()
              << "ms");

        CameraState::get()->ready = 1;

        if (time(0) - CameraState::get()->lastRequest > 30) {
            DEBUG("sleeping");
            sleep(1);
        }
    }
}

int main(int argc, char** argv) {
    int port = 8181;
    std::string path;
    bool saveFiles = false;
    if(argc == 2){
        saveFiles = true;
        path = std::string(argv[1]);
    }
    httpserver::webserver ws = httpserver::create_webserver(port);
    installWebHandlers(&ws);

    std::thread t(cameraThread,saveFiles,path);

    std::cout << "Starting to listen on port: " << port << std::endl;

    ws.start(true);

    return 0;
}
