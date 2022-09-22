// intelrealserver.cpp

#include <assert.h>

#include <chrono>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <thread>

#include "cameraserver.h"

//#define DEBUG(x) std::cout << x << std::endl
#define DEBUG(x)

void cameraThread() {
    rs2::context ctx;

    std::vector<rs2::pipeline> pipelines;

    for (auto&& dev : ctx.query_devices()) {
        auto serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::cout << "got serial: " << serial << std::endl;

        rs2::config cfg;
        cfg.enable_device(serial);
        cfg.enable_stream(RS2_STREAM_DEPTH);
        cfg.enable_stream(RS2_STREAM_COLOR);

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
            std::cout << "----------------------\n";

            std::shared_ptr<CameraOutput> output(new CameraOutput());

            auto start1 = std::chrono::high_resolution_clock::now();
            rs2::frameset frames;
            auto succ = p.try_wait_for_frames(&frames, 1000);
            if (!succ) {
                std::cout << "Failed to get frame, skipping...\n";
                continue;
            }
            auto finish1 = std::chrono::high_resolution_clock::now();
            std::cout << " - wait_for_frame: " << std::chrono::duration_cast<std::chrono::milliseconds>(finish1 - start1).count() << "ms\n";
            
            // this handles the geometry so that the
            // x/y of the depth and color are the same
            auto start2 = std::chrono::high_resolution_clock::now();
            //frames = alignment.process(frames);
            try {
                frames = alignment.process(frames);
            } catch (std::exception& e) {
                // Catch exceptions, since alignment can fail
                std::cout << "Exception while execution of the processing: " << e.what() << std::endl;
                continue;
            }
            auto finish2 = std::chrono::high_resolution_clock::now();
            std::cout << " - aligment: " << std::chrono::duration_cast<std::chrono::milliseconds>(finish2 - start2).count() << "ms\n";

            // do color frame
            auto vf = frames.get_color_frame();

            assert(vf.get_bytes_per_pixel() == 3);
            assert(vf.get_stride_in_bytes() ==
                   (vf.get_width() * vf.get_bytes_per_pixel()));

            auto start4 = std::chrono::high_resolution_clock::now();
            output->width = vf.get_width();
            output->height = vf.get_height();
            output->ppmdata =
                my_write_ppm((const char*)vf.get_data(), vf.get_width(),
                             vf.get_height(), vf.get_bytes_per_pixel());

            auto finish4 = std::chrono::high_resolution_clock::now();
            std::cout << "- ppm: " << std::chrono::duration_cast<std::chrono::milliseconds>(finish4 - start4).count() << "ms\n";
            auto start5 = std::chrono::high_resolution_clock::now();
            try {
                output->pic_cv = cv::Mat(vf.get_height(), vf.get_width(),
                                         CV_8UC3, (void*)(vf.get_data()));
            } catch (std::exception& e) {
                // Catch exceptions, since constructing the matrix can fail
                // when the size is 0.
                std::cout
                    << "Exception while constructing matrix for color frame: "
                    << e.what() << std::endl;
                output->pic_cv = cv::Mat();
            }
            auto finish5 = std::chrono::high_resolution_clock::now();
            std::cout << "- try: " << std::chrono::duration_cast<std::chrono::milliseconds>(finish5 - start5).count() << "ms\n";

            // create depth maps

            auto depth = frames.get_depth_frame();
            output->depth_width = depth.get_width();
            output->depth_height = depth.get_height();
            output->add_depth(depth.get_bytes_per_pixel(), depth.get_units(),
                              depth.get_width(), depth.get_height(),
                              (const char*)depth.get_data());

	    int w = output->depth_width;
	    int h = output->depth_height;
	    cv::Mat cvBuf(h, w, CV_16U);
	    const uint16_t *z_pixels = reinterpret_cast<const uint16_t*>(depth.get_data());
        auto start3 = std::chrono::high_resolution_clock::now();
	    for (int y = 0; y < h; y++) {
	      for (int x = 0; x < w; x++) { 
		cvBuf.at<uint16_t>(y,x) = z_pixels[y*w+x];
	      }
	    }
        auto finish3 = std::chrono::high_resolution_clock::now();
        std::cout << "- loop: " << std::chrono::duration_cast<std::chrono::milliseconds>(finish3 - start3).count() << "ms\n";
	    output->depth_cv = cvBuf;
            DEBUG("middle distance: " << depth.get_distance(
                      depth.get_width() / 2, depth.get_height() / 2));

            CameraState::get()->cameras[num++] = output;
        }

        auto finish = std::chrono::high_resolution_clock::now();
        DEBUG(std::chrono::duration_cast<std::chrono::milliseconds>(finish -
                                                                    start)
                  .count()
              << "ms");
        std::cout << "final time: " << std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() << "ms\n";

        CameraState::get()->ready = 1;

        if (time(0) - CameraState::get()->lastRequest > 30) {
            DEBUG("sleeping");
            sleep(1);
        }
    }
}

int main(int argc, char** argv) {
    int port = 8181;

    httpserver::webserver ws = httpserver::create_webserver(port);
    installWebHandlers(&ws);

    std::thread t(cameraThread);

    std::cout << "Starting to listen on port: " << port << std::endl;

    ws.start(true);

    return 0;
}
