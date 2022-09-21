#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
// #include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>

#include "common/v1/common.grpc.pb.h"
#include "common/v1/common.pb.h"
#include "component/camera/v1/camera.grpc.pb.h"
#include "component/camera/v1/camera.pb.h"
#include "robot/v1/robot.grpc.pb.h"
#include "robot/v1/robot.pb.h"

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

#include "rs-pcl-color.cpp"

#define DEBUG(x)
using namespace std;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::ServerReaderWriter;
using grpc::ServerWriter;
using viam::common::v1::ResourceName;
using viam::component::camera::v1::CameraService;
using viam::component::camera::v1::GetImageRequest;
using viam::component::camera::v1::GetImageResponse;
using viam::component::camera::v1::GetPointCloudRequest;
using viam::component::camera::v1::GetPointCloudResponse;
using viam::component::camera::v1::GetPropertiesRequest;
using viam::component::camera::v1::GetPropertiesResponse;
using viam::component::camera::v1::IntrinsicParameters;
using viam::robot::v1::ResourceNamesRequest;
using viam::robot::v1::ResourceNamesResponse;
using viam::robot::v1::RobotService;

class CameraOutput {
   public:
    CameraOutput() {}

    int color_width;
    int color_height;
    float color_fx;
    float color_fy;
    float color_ppx;
    float color_ppy;
    int color_pixel_bytes;
    cv::Mat colorframe;
    int depth_width;
    int depth_height;
    float depth_fx;
    float depth_fy;
    float depth_ppx;
    float depth_ppy;
    int depth_pixel_bytes;
    cv::Mat depthframe;
    pcl::PointCloud<pcl::PointXYZRGB> colorcloud;
};

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

class CameraServiceImpl final : public CameraService::Service {
   public:
       CameraServiceImpl() = default;
       virtual ~CameraServiceImpl() = default;

	   ::grpc::Status GetImage(ServerContext* context, 
               const GetImageRequest* request, 
               GetImageResponse* response) override {
           // check if camera is found 
           if (CameraState::get()->cameras.size() < 1) {
               return grpc::Status(grpc::StatusCode::NOT_FOUND, "camera not found");
           }
           // check if camera is ready
           if (!CameraState::get()->ready) {
               return grpc::Status(grpc::StatusCode::UNAVAILABLE, "camera is not ready");
           }
           std::shared_ptr<CameraOutput> mine = CameraState::get()->cameras[0];
           CameraOutput* data = mine.get();
           auto reqName = request->name();
           auto reqMimeType = request->mime_type();
           if (reqName == "color") {
               if (reqMimeType == "image/png") {
                   response->set_mime_type("image/png");
                   std::vector<uchar> chbuf;
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
                   std::vector<uchar> chbuf;
                   chbuf.resize(5*1024*1024);
                   cv::imencode(".jpg", data->colorframe, chbuf);
                   std::string s(chbuf.begin(), chbuf.end()); 
                   response->set_image(s);
               }
           }
           if (reqName == "depth") {
               if (reqMimeType == "image/jpeg") {
                   response->set_mime_type("image/jpeg");
                   std::vector<uchar> chbuf;
                   chbuf.resize(5*1024*1024);
                   cv::imencode(".jpg", data->colorframe, chbuf);
                   std::string s(chbuf.begin(), chbuf.end()); 
                   response->set_image(s);
               } else if (reqMimeType == "image/vnd.viam.rgba") {
                   response->set_mime_type("image/vnd.viam.rgba");
                   cv::Mat cvConverted(data->depth_height, data->depth_width, CV_16UC4);
                   cv::cvtColor(data->depthframe, cvConverted, cv::COLOR_GRAY2RGBA, 4);
                    std::string s(cvConverted.datastart, cvConverted.dataend);
                   response->set_image(s);
               } else { // return png by default
                   response->set_mime_type("image/png");
                   std::vector<uchar> chbuf;
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
           // check if camera is found 
           if (CameraState::get()->cameras.size() < 1) {
               return grpc::Status(grpc::StatusCode::NOT_FOUND, "camera not found");
           }
           // check if camera is ready
           if (!CameraState::get()->ready) {
               return grpc::Status(grpc::StatusCode::UNAVAILABLE, "camera is not ready");
           }
           std::shared_ptr<CameraOutput> mine = CameraState::get()->cameras[0];
           CameraOutput* data = mine.get();
           auto cloud = data->colorcloud;
           // create the pcd file
           std::stringbuf buffer;
           std::ostream oss(&buffer);
           oss << "VERSION .7\n"
               << "FIELDS x y z rgb\n"
               << "SIZE 4 4 4 4\n"
               << "TYPE F F F I\n"
               << "COUNT 1 1 1 1\n"
               << "WIDTH " << cloud.points.size() << "\n"
               << "HEIGHT " << 1 << "\n"
               << "VIEWPOINT 0 0 0 1 0 0 0\n"
               << "POINTS " << cloud.points.size() << "\n"
               << "DATA binary\n";
           for (int i = 0; i < cloud.points.size(); i++) {
               auto point = cloud.points[i];
               float x = point.x;
               float y = point.y;
               float z = point.z;
               int rgb = 0;
               rgb = rgb | (point.r << 16);
               rgb = rgb | (point.g << 8);
               rgb = rgb | (point.b << 0);
               buffer.sputn((const char*)&x, 4);
               buffer.sputn((const char*)&y, 4);
               buffer.sputn((const char*)&z, 4);
               buffer.sputn((const char*)&rgb, 4);
           }
           response->set_mime_type("pointcloud/pcd");
           response->set_point_cloud(buffer.str());
           return grpc::Status::OK;
       }

       ::grpc::Status GetProperties(ServerContext* context,
               const GetPropertiesRequest* request,
               GetPropertiesResponse* response) override {
           // check if camera is found 
           if (CameraState::get()->cameras.size() < 1) {
               return grpc::Status(grpc::StatusCode::NOT_FOUND, "camera not found");
           }
           // check if camera is ready
           if (!CameraState::get()->ready) {
               return grpc::Status(grpc::StatusCode::UNAVAILABLE, "camera is not ready");
           }
           std::shared_ptr<CameraOutput> mine = CameraState::get()->cameras[0];
           CameraOutput* data = mine.get();
	       response->set_supports_pcd(true);
           IntrinsicParameters* intrinsics = response->mutable_intrinsic_parameters();
           auto reqName = request->name();
           if (reqName == "color") {
                intrinsics->set_width_px(data->color_width);
                intrinsics->set_height_px(data->color_height);
                intrinsics->set_focal_x_px(data->color_fx);
                intrinsics->set_focal_y_px(data->color_fy);
                intrinsics->set_center_x_px(data->color_ppx);
                intrinsics->set_center_y_px(data->color_ppy);
           }
           if (reqName == "depth") {
                intrinsics->set_width_px(data->depth_width);
                intrinsics->set_height_px(data->depth_height);
                intrinsics->set_focal_x_px(data->depth_fx);
                intrinsics->set_focal_y_px(data->depth_fy);
                intrinsics->set_center_x_px(data->depth_ppx);
                intrinsics->set_center_y_px(data->depth_ppy);
           }

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
            // color info 
            auto const color_stream = p.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            auto color_intrin = color_stream.get_intrinsics();
            output->color_width = color.get_width();
            output->color_height = color.get_height();
            output->color_fx = color_intrin.fx;
            output->color_fy = color_intrin.fy;
            output->color_ppx = color_intrin.ppx;
            output->color_ppy = color_intrin.ppy;
            output->color_pixel_bytes = color.get_bytes_per_pixel();
            try {
                output->colorframe = cv::Mat(color.get_height(), color.get_width(), CV_8UC3, (void*)(color.get_data()));
            } catch (std::exception& e) {
                // Catch exceptions, since constructing the matrix can fail when the size is 0.
                std::cout << "Exception while constructing matrix for color frame: " << e.what() << std::endl;
                output->colorframe = cv::Mat();
            }
            // depth info
            auto const depth_stream = p.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
            auto depth_intrin = depth_stream.get_intrinsics();
            output->depth_width = depth.get_width();
            output->depth_height = depth.get_height();
            output->depth_fx = depth_intrin.fx;
            output->depth_fy = depth_intrin.fy;
            output->depth_ppx = depth_intrin.ppx;
            output->depth_ppy = depth_intrin.ppy;
            output->depth_pixel_bytes = depth.get_bytes_per_pixel();
            try {
                output->depthframe = cv::Mat(depth.get_height(), depth.get_width(), CV_16U, (void*)(depth.get_data()));
            } catch (std::exception& e) {
                std::cout << "Exception while constructing matrix for depth frame: " << e.what() << std::endl;
                output->depthframe = cv::Mat();
            }
            // pointcloud info
            rs2::pointcloud pc;
            pc.map_to(color);
            auto points = pc.calculate(depth);
            try {
                auto cloud = PCL_Conversion(points, color);
                output->colorcloud = *cloud;
            } catch (std::exception& e) {
                std::cout << "Exception while constructing matrix pointcloud: " << e.what() << std::endl;
                output->colorcloud = pcl::PointCloud<pcl::PointXYZRGB>();
            }
            // save the the output
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
    CameraServiceImpl cameraService;
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
