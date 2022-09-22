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
#include <condition_variable>
#include <functional>
#include <iostream>
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

class RealSenseProperties{
   public:
    RealSenseProperties() {}
    // color camera
    int color_width;
    int color_height;
    float color_fx;
    float color_fy;
    float color_ppx;
    float color_ppy;
    // depth camera
    int depth_width;
    int depth_height;
    float depth_fx;
    float depth_fy;
    float depth_ppx;
    float depth_ppy;
};

class CameraServiceImpl final : public CameraService::Service {
   private:
      std::unique_ptr<RealSenseProperties> m_rsp; 
      rs2::pipeline m_p;
   public:
       CameraServiceImpl(int width, int height){
           m_rsp = make_unique<RealSenseProperties>();
           try {
                m_p = startPipeline(width, height); // initialize pipeline m_p
           } catch (std::exception& e) {
               std::cout << "Exception while starting pipeline for realsense camera: " << e.what() << std::endl;
           }
           // get properties
           auto const color_stream = m_p.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
           auto color_intrin = color_stream.get_intrinsics();
           auto const depth_stream = m_p.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
           auto depth_intrin = depth_stream.get_intrinsics();
           m_rsp->color_width = color_intrin.width;
           m_rsp->color_height = color_intrin.height;
           m_rsp->color_fx = color_intrin.fx;
           m_rsp->color_fy = color_intrin.fy;
           m_rsp->color_ppx = color_intrin.ppx;
           m_rsp->color_ppy = color_intrin.ppy;
           m_rsp->depth_width = depth_intrin.width;
           m_rsp->depth_height = depth_intrin.height;
           m_rsp->depth_fx = depth_intrin.fx;
           m_rsp->depth_fy = depth_intrin.fy;
           m_rsp->depth_ppx = depth_intrin.ppx;
           m_rsp->depth_ppy = depth_intrin.ppy;
       };
       virtual ~CameraServiceImpl() = default;

       rs2::pipeline startPipeline(int width, int height) {
           rs2::context ctx;
           rs2::device_list devices = ctx.query_devices();
           rs2::device selected_device;
           if (devices.size() == 0) {
               std::cerr << "No device connected, please connect a RealSense device" << std::endl;
               exit(-1);
           }
           else {
               selected_device = devices[0];
           }

           rs2::pipeline pipe;
           auto serial = selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
           std::cout << "got serial: " << serial << std::endl;

           rs2::config cfg;
           cfg.enable_device(serial);
           try {
               cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8); // 0 width or height indicates any
           } catch (std::exception& e) {
               std::cout << "Exception while enabling (" << width << ", " << height <<") stream: " << e.what() << std::endl;
               exit(-1);
           }
           cfg.enable_stream(RS2_STREAM_DEPTH);

           rs2::pipeline p(ctx);
           p.start(cfg);

           return p;
       };

	   ::grpc::Status GetImage(ServerContext* context, 
               const GetImageRequest* request, 
               GetImageResponse* response) override {
           // request frames from pipeline
           rs2::frameset frames = m_p.wait_for_frames();
           rs2::align alignment(RS2_STREAM_COLOR); // align to the color camera's origin
           // this handles the geometry so that the x/y of the depth and color are the same
           frames = alignment.process(frames);
           auto reqName = request->name();
           auto reqMimeType = request->mime_type();
           if (reqName == "color") {
               cv::Mat colorframe;
               rs2::video_frame color = frames.get_color_frame();
               try {
                   colorframe = cv::Mat(color.get_height(), color.get_width(), CV_8UC3, (void*)(color.get_data()));
               } catch (std::exception& e) {
                   // Catch exceptions, since constructing the matrix can fail when the size is 0.
                   std::cout << "Exception while constructing matrix for color frame: " << e.what() << std::endl;
               }
               if (colorframe.empty()) {
                  return grpc::Status(grpc::StatusCode::UNAVAILABLE, "no data in this image");
               }
               if (reqMimeType == "image/png") {
                   response->set_mime_type("image/png");
                   std::vector<uchar> chbuf;
                   chbuf.resize(5*1024*1024);
                   cv::Mat cvConverted(m_rsp->color_height, m_rsp->color_width, CV_8UC3);
                   cv::cvtColor(colorframe, cvConverted, cv::COLOR_BGR2RGB);
                   cv::imencode(".png", cvConverted, chbuf);
                   std::string s(chbuf.begin(), chbuf.end()); 
                   response->set_image(s);
               } else if (reqMimeType == "image/vnd.viam.rgba") {
                   response->set_mime_type("image/vnd.viam.rgba");
                   cv::Mat cvConverted(m_rsp->color_height, m_rsp->color_width, CV_16UC4);
                   cv::cvtColor(colorframe, cvConverted, cv::COLOR_BGR2RGBA, 4);
                    std::string s(cvConverted.datastart, cvConverted.dataend);
                   response->set_image(s);
               } else { // return jpeg by default
                   response->set_mime_type("image/jpeg");
                   std::vector<uchar> chbuf;
                   chbuf.resize(5*1024*1024);
                   cv::Mat cvConverted(m_rsp->color_height, m_rsp->color_width, CV_8UC3);
                   cv::cvtColor(colorframe, cvConverted, cv::COLOR_BGR2RGB);
                   cv::imencode(".jpg", cvConverted, chbuf);
                   std::string s(chbuf.begin(), chbuf.end()); 
                   response->set_image(s);
               }
           }
           if (reqName == "depth") {
               cv::Mat depthframe;
               rs2::depth_frame depth = frames.get_depth_frame();
               try {
                   depthframe = cv::Mat(depth.get_height(), depth.get_width(), CV_16U, (void*)(depth.get_data()));
               } catch (std::exception& e) {
                   std::cout << "Exception while constructing matrix for depth frame: " << e.what() << std::endl;
               }
               if (depthframe.empty()) {
                  return grpc::Status(grpc::StatusCode::UNAVAILABLE, "no data in this image");
               }
               if (reqMimeType == "image/jpeg") {
                   response->set_mime_type("image/jpeg");
                   std::vector<uchar> chbuf;
                   chbuf.resize(5*1024*1024);
                   cv::imencode(".jpg", depthframe, chbuf);
                   std::string s(chbuf.begin(), chbuf.end()); 
                   response->set_image(s);
               } else if (reqMimeType == "image/vnd.viam.rgba") {
                   response->set_mime_type("image/vnd.viam.rgba");
                   cv::Mat cvConverted(m_rsp->depth_height, m_rsp->depth_width, CV_16UC4);
                   cv::cvtColor(depthframe, cvConverted, cv::COLOR_GRAY2RGBA, 4);
                    std::string s(cvConverted.datastart, cvConverted.dataend);
                   response->set_image(s);
               } else { // return png by default, jpeg is lossy and will destroy depth pixel info.
                   response->set_mime_type("image/png");
                   std::vector<uchar> chbuf;
                   chbuf.resize(5*1024*1024);
                   cv::imencode(".png", depthframe, chbuf);
                   std::string s(chbuf.begin(), chbuf.end()); 
                   response->set_image(s);
               }
           }
		   return grpc::Status::OK;
       }

       ::grpc::Status GetPointCloud(ServerContext* context, 
               const GetPointCloudRequest* request, 
               GetPointCloudResponse* response) override {
           rs2::frameset frames = m_p.wait_for_frames();
           rs2::align alignment(RS2_STREAM_COLOR); // align to the color camera's origin
           // this handles the geometry so that the x/y of the depth and color are the same
           frames = alignment.process(frames);
           auto color = frames.get_color_frame();
           auto depth = frames.get_depth_frame();
           rs2::pointcloud pc;
           pc.map_to(color);
           auto points = pc.calculate(depth);
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
           try {
               cloud = PCL_Conversion(points, color);
           } catch (std::exception& e) {
               std::cout << "Exception while constructing pointcloud: " << e.what() << std::endl;
           }
           if (cloud->points.size() == 0) {
              return grpc::Status(grpc::StatusCode::UNAVAILABLE, "no data in this pointcloud");
           }
           // create the pcd file
           std::stringbuf buffer;
           std::ostream oss(&buffer);
           oss << "VERSION .7\n"
               << "FIELDS x y z rgb\n"
               << "SIZE 4 4 4 4\n"
               << "TYPE F F F I\n"
               << "COUNT 1 1 1 1\n"
               << "WIDTH " << cloud->points.size() << "\n"
               << "HEIGHT " << 1 << "\n"
               << "VIEWPOINT 0 0 0 1 0 0 0\n"
               << "POINTS " << cloud->points.size() << "\n"
               << "DATA binary\n";
           for (int i = 0; i < cloud->points.size(); i++) {
               auto point = cloud->points[i];
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
	       response->set_supports_pcd(true);
           IntrinsicParameters* intrinsics = response->mutable_intrinsic_parameters();
           auto reqName = request->name();
           if (reqName == "color") {
                intrinsics->set_width_px(m_rsp->color_width);
                intrinsics->set_height_px(m_rsp->color_height);
                intrinsics->set_focal_x_px(m_rsp->color_fx);
                intrinsics->set_focal_y_px(m_rsp->color_fy);
                intrinsics->set_center_x_px(m_rsp->color_ppx);
                intrinsics->set_center_y_px(m_rsp->color_ppy);
           }
           if (reqName == "depth") {
                intrinsics->set_width_px(m_rsp->depth_width);
                intrinsics->set_height_px(m_rsp->depth_height);
                intrinsics->set_focal_x_px(m_rsp->depth_fx);
                intrinsics->set_focal_y_px(m_rsp->depth_fy);
                intrinsics->set_center_x_px(m_rsp->depth_ppx);
                intrinsics->set_center_y_px(m_rsp->depth_ppy);
           }

           return grpc::Status::OK;
        }

    virtual std::string name() const { return std::string("IntelRealSenseServer"); }
};

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
    if (argc == 1) {
        std::cout << "optional arguments are: port_number, image_width, image_height." << std::endl;
    }
    std::string port = "8085";
    std::string x_res = "";
    std::string y_res = "";
    if (argc > 1) {
        port =  argv[1];
    }
    if (argc > 2) {
        x_res =  argv[2];
    }
    if (argc > 3) {
        y_res =  argv[3];
    }
    // start running camera thread
    RobotServiceImpl robotService;
    CameraServiceImpl cameraService(std::atoi(x_res.c_str()), std::atoi(y_res.c_str()));
    grpc::EnableDefaultHealthCheckService(true);
    // grpc::reflection::InitProtoReflectionServerBuilderPlugin();
    ServerBuilder builder;
    std::string address = "0.0.0.0:" + port;
    builder.AddListeningPort(address, grpc::InsecureServerCredentials());
    builder.RegisterService(&robotService);
    builder.RegisterService(&cameraService);
    std::unique_ptr<Server> server(builder.BuildAndStart());
    std::cout << "Starting to listen on " << address << std::endl;
    server->Wait();
    return 0;
}
