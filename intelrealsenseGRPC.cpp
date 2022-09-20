#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
// #include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>

#include "proto/api/common/v1/common.grpc.pb.h"
#include "proto/api/common/v1/common.pb.h"
#include "proto/api/component/camera/v1/camera.grpc.pb.h"
#include "proto/api/component/camera/v1/camera.pb.h"
#include "proto/api/robot/v1/robot.grpc.pb.h"
#include "proto/api/robot/v1/robot.pb.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <librealsense2/rs.hpp>

#include <assert.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <sstream>
#include <thread>
#include <tuple>
#include <vector>

#define DEBUG(x)
using namespace std;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::ServerReaderWriter;
using grpc::ServerWriter;
using proto::api::common::v1::ResourceName;
using proto::api::component::camera::v1::CameraService;
using proto::api::component::camera::v1::GetImageRequest;
using proto::api::component::camera::v1::GetImageResponse;
using proto::api::component::camera::v1::GetPointCloudRequest;
using proto::api::component::camera::v1::GetPointCloudResponse;
using proto::api::component::camera::v1::GetPropertiesRequest;
using proto::api::component::camera::v1::GetPropertiesResponse;
using proto::api::component::camera::v1::IntrinsicParameters;
using proto::api::robot::v1::ResourceNamesRequest;
using proto::api::robot::v1::ResourceNamesResponse;
using proto::api::robot::v1::RobotService;

class CameraState {
   private:
    CameraState() : ready(0), lastRequest(0) {}

   public:
    static CameraState* get();

    std::vector<std::shared_ptr<CameraOutput>> cameras;
    bool ready;
    volatile time_t lastRequest;
};

CameraState* myCameraState = 0;

CameraState* CameraState::get() {
    if (!myCameraState) {
        myCameraState = new CameraState();
    }
    return myCameraState;
}

class CameraOutput {
   public:
    CameraOutput() {}

    int color_width;
    int color_height;
    int color_pixel_bytes;
    cv::Mat colorframe;
    int depth_width;
    int depth_height;
    int depth_pixel_bytes;
    cv::Mat depthframe;
};

class CameraServiceImpl final : public CameraService::Service {
   private:
       CameraState* cams_;
   public:
       explicit CameraServiceImpl(CameraState* cam) : cams_(cam) {}
       virtual ~CameraServiceImpl() = default;

	   ::grpc::Status GetImage(ServerContext* context, 
               const GetImageRequest* request, 
               GetImageResponse* response) override {
           // check if camera is found 
           if (cams_->cameras.size() < 1) {
               return grpc::Status(grpc::StatusCode::NOT_FOUND, "camera not found");
           }
           // check if camera is ready
           if (!cams_->ready) {
               return grpc::Status(grpc::StatusCode::UNAVAILABLE, "camera is not ready");
           }
           CameraOutput* data = cams_->cameras[0].get();
           auto reqName = request->name();
           auto reqMimeType = request->mime_type();
           if (reqName == "color") {
               if (reqMimeType == "image/png") {
                   response->set_mime_type("image/png");
                   std::vector<uchar> cfbuf;
                   chbuf.resize(5*1024*1024);
                   cv::imencode(".png", data->colorframe, chbuf);
                   std::string s(chbuf.begin(), chbuf.end()); 
                   response->set_image(s);
               } else if (reqMimeType == "image/vnd.viam.rgba") {
                   response->set_mime_type("image/vnd.viam.rgba");
                   cv::Mat cvConverted(data->color_height, data->color_width, CV_16UC4);
                   cv::cvtColor(data->colorframe, cvConverted, cv::COLOR_RGB2RGBA, 4);
                    std::string s(cvConverted.datastart, cvConverted.dataend);
                   response->set_image(s);
               } else { // return jpeg by default
                   response->set_mime_type("image/jpeg");
                   std::vector<uchar> cfbuf;
                   chbuf.resize(5*1024*1024);
                   cv::imencode(".jpg", data->colorframe, chbuf);
                   std::string s(chbuf.begin(), chbuf.end()); 
                   response->set_image(s);
               }
           }
           if (reqName == "depth") {
               if (reqMimeType == "image/jpeg") {
                   response->set_mime_type("image/jpeg");
                   std::vector<uchar> cfbuf;
                   chbuf.resize(5*1024*1024);
                   cv::imencode(".png", data->colorframe, chbuf);
                   std::string s(chbuf.begin(), chbuf.end()); 
                   response->set_image(s);
               } else if (reqMimeType == "image/vnd.viam.rgba") {
                   cv::Mat cvConverted(data->depth_height, data->depth_width, CV_16UC4);
                   cv::cvtColor(data->colorframe, cvConverted, cv::COLOR_GRAY2RGBA, 4);
                    std::string s(cvConverted.datastart, cvConverted.dataend);
                   response->set_image(s);
               } else { // return png by default
                   response->set_mime_type("image/png");
                   std::vector<uchar> cfbuf;
                   chbuf.resize(5*1024*1024);
                   cv::imencode(".png", data->depthframe, chbuf);
                   std::string s(chbuf.begin(), chbuf.end()); 
                   response->set_image(s);
               }
           }
		   return grpc::Status::OK;
       }

       ::grpc::Status GetPointCloud(ServerContext* context, 
               const GetPointCloudRequest* request, 
               GetPointCloudResponse* response) override {
            //rs2::pointcloud pc = rs2::context().create_pointcloud();
            //rs2::points points;
           return grpc::Status::OK;
       }

       ::grpc::Status GetProperties(ServerContext* context,
               const GetPropertiesRequest* request,
               GetPropertiesResponse* response) override {
            return grpc::Status::OK;
        }

    virtual std::string name() const { return std::string("IntelRealSenseServer"); }
};

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
            std::shared_ptr<CameraOutput> output(new CameraOutput());
            rs2::frameset frames = p.wait_for_frames();

            // this handles the geometry so that the
            // x/y of the depth and color are the same
            frames = alignment.process(frames);
            rs2::video_frame color = frames.get_color_frame();
            rs2::depth_frame depth = frames.get_depth_frame();
            points = pc.calculate(depth);
            pc.map_to(color);
            // color info 
            output->color_width = color.get_width();
            output->color_height = color.get_height();
            output->color_pixel_bytes = color.bytes_per_pixel();
            try {
                output->colorframe = cv::Mat(vf.get_height(), vf.get_width(), CV_8UC3, (void*)(color.get_data()));
            } catch (std::exception& e) {
                // Catch exceptions, since constructing the matrix can fail when the size is 0.
                std::cout << "Exception while constructing matrix for color frame: " << e.what() << std::endl;
                output->colorframe = cv::Mat();
            }
            // depth info
            output->depth_width = depth.get_width();
            output->depth_height = depth.get_height();
            output->depth_pixel_bytes = depth.bytes_per_pixel();
            std::cout << "depth bytes per pixel: " << depth.bytes_per_pixel() << std::endl;
            try {
                output->depthframe = cv::Mat(depth.get_height(), depth.get_width(), CV_16U, (void*)(depth.get_data()));
            } catch (std::exception& e) {
                std::cout << "Exception while constructing matrix for depth frame: " << e.what() << std::endl;
                output->depthframe = cv::Mat();
            }
            // save the points in the output
            CameraState::get()->cameras[num++] = output;
        }
        auto finish = std::chrono::high_resolution_clock::now();
        DEBUG(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() << "ms");

        CameraState::get()->ready = 1;

        if (time(0) - CameraState::get()->lastRequest > 30) {
            DEBUG("sleeping");
            sleep(1);
        }
    }
}

class RobotServiceImpl final : public RobotService::Service {
   public:
    grpc::Status ResourceNames(ServerContext* context,
                               const ResourceNamesRequest* request,
                               ResourceNamesResponse* response) override {
        ResourceName* name = response->add_resources();
        name->set_namespace_("rdk");
        name->set_type("component");
        name->set_subtype("camera");
        name->set_name("color");

        ResourceName* name2 = response->add_resources();
        name2->set_namespace_("rdk");
        name2->set_type("component");
        name2->set_subtype("camera");
        name2->set_name("depth");
        return grpc::Status::OK;
    }
};

int main(int argc, char* argv[]) {
    std::thread t(cameraThread); // start running the camera
    RobotServiceImpl robotService;
    CameraServiceImpl cameraService(myCameraState);
    grpc::EnableDefaultHealthCheckService(true);
    // grpc::reflection::InitProtoReflectionServerBuilderPlugin();
    ServerBuilder builder;
    builder.AddListeningPort("0.0.0.0:8085", grpc::InsecureServerCredentials());
    builder.RegisterService(&robotService);
    builder.RegisterService(&cameraService);
    std::unique_ptr<Server> server(builder.BuildAndStart());
    std::cout << "Starting to listen on 0.0.0.0:8085" << std::endl;
    server->Wait();
    return 0;
}
