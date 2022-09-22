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

class RealSenseProperties{
   public:
    RealSenseProperties() {}
    std::mutex mu;
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

class RealSenseOutput {
   public:
    RealSenseOutput() {}
    std::mutex mu;
    cv::Mat colorframe;
    cv::Mat depthframe;
    pcl::PointCloud<pcl::PointXYZRGB> colorcloud;
};

std::atomic<bool> cameraReady(false);

class CameraServiceImpl final : public CameraService::Service {
   private:
     RealSenseProperties* m_rsp; 
      RealSenseOutput* m_rso;
   public:
       CameraServiceImpl(RealSenseProperties* properties, RealSenseOutput* output): m_rsp(properties), m_rso(output) {};
       virtual ~CameraServiceImpl() = default;

	   ::grpc::Status GetImage(ServerContext* context, 
               const GetImageRequest* request, 
               GetImageResponse* response) override {
           // check if camera is ready
           if (!cameraReady) {
               return grpc::Status(grpc::StatusCode::UNAVAILABLE, "camera is not ready");
           }
           auto reqName = request->name();
           auto reqMimeType = request->mime_type();
           if (reqName == "color") {
               if (m_rso->colorframe.empty()) {
                  return grpc::Status(grpc::StatusCode::UNAVAILABLE, "no data in this image");
               }
               if (reqMimeType == "image/png") {
                   response->set_mime_type("image/png");
                   std::vector<uchar> chbuf;
                   chbuf.resize(5*1024*1024);
                   cv::Mat cvConverted(m_rsp->color_height, m_rsp->color_width, CV_8UC3);
                   cv::cvtColor(m_rso->colorframe, cvConverted, cv::COLOR_BGR2RGB);
                   cv::imencode(".png", cvConverted, chbuf);
                   std::string s(chbuf.begin(), chbuf.end()); 
                   response->set_image(s);
               } else if (reqMimeType == "image/vnd.viam.rgba") {
                   response->set_mime_type("image/vnd.viam.rgba");
                   cv::Mat cvConverted(m_rsp->color_height, m_rsp->color_width, CV_16UC4);
                   cv::cvtColor(m_rso->colorframe, cvConverted, cv::COLOR_BGR2RGBA, 4);
                    std::string s(cvConverted.datastart, cvConverted.dataend);
                   response->set_image(s);
               } else { // return jpeg by default
                   response->set_mime_type("image/jpeg");
                   std::vector<uchar> chbuf;
                   chbuf.resize(5*1024*1024);
                   cv::Mat cvConverted(m_rsp->color_height, m_rsp->color_width, CV_8UC3);
                   cv::cvtColor(m_rso->colorframe, cvConverted, cv::COLOR_BGR2RGB);
                   cv::imencode(".jpg", cvConverted, chbuf);
                   std::string s(chbuf.begin(), chbuf.end()); 
                   response->set_image(s);
               }
           }
           if (reqName == "depth") {
               if (m_rso->depthframe.empty()) {
                  return grpc::Status(grpc::StatusCode::UNAVAILABLE, "no data in this image");
               }
               if (reqMimeType == "image/jpeg") {
                   response->set_mime_type("image/jpeg");
                   std::vector<uchar> chbuf;
                   chbuf.resize(5*1024*1024);
                   cv::imencode(".jpg", m_rso->depthframe, chbuf);
                   std::string s(chbuf.begin(), chbuf.end()); 
                   response->set_image(s);
               } else if (reqMimeType == "image/vnd.viam.rgba") {
                   response->set_mime_type("image/vnd.viam.rgba");
                   cv::Mat cvConverted(m_rsp->depth_height, m_rsp->depth_width, CV_16UC4);
                   cv::cvtColor(m_rso->depthframe, cvConverted, cv::COLOR_GRAY2RGBA, 4);
                    std::string s(cvConverted.datastart, cvConverted.dataend);
                   response->set_image(s);
               } else { // return png by default, jpeg is lossy and will destroy depth pixel info.
                   response->set_mime_type("image/png");
                   std::vector<uchar> chbuf;
                   chbuf.resize(5*1024*1024);
                   cv::imencode(".png", m_rso->depthframe, chbuf);
                   std::string s(chbuf.begin(), chbuf.end()); 
                   response->set_image(s);
               }
           }
		   return grpc::Status::OK;
       }

       ::grpc::Status GetPointCloud(ServerContext* context, 
               const GetPointCloudRequest* request, 
               GetPointCloudResponse* response) override {
           // check if camera is ready
           if (!cameraReady) {
               return grpc::Status(grpc::StatusCode::UNAVAILABLE, "camera is not ready");
           }
           if (m_rso->colorcloud.points.size() == 0) {
              return grpc::Status(grpc::StatusCode::UNAVAILABLE, "no data in this pointcloud");
           }
           auto cloud = m_rso->colorcloud;
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
           // check if camera is ready
           if (!cameraReady) {
               return grpc::Status(grpc::StatusCode::UNAVAILABLE, "camera is not ready");
           }
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

void cameraThread(RealSenseProperties* rsp, RealSenseOutput* rso, int width, int height) {
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

    rs2::align alignment(RS2_STREAM_COLOR);
    // rs2::align alignment(RS2_STREAM_DEPTH);

    // get properties
	auto const color_stream = p.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	auto color_intrin = color_stream.get_intrinsics();
	auto const depth_stream = p.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	auto depth_intrin = depth_stream.get_intrinsics();
    rsp->mu.lock();
	rsp->color_width = color_intrin.width;
	rsp->color_height = color_intrin.height;
	rsp->color_fx = color_intrin.fx;
	rsp->color_fy = color_intrin.fy;
	rsp->color_ppx = color_intrin.ppx;
	rsp->color_ppy = color_intrin.ppy;
	rsp->depth_width = depth_intrin.width;
	rsp->depth_height = depth_intrin.height;
	rsp->depth_fx = depth_intrin.fx;
	rsp->depth_fy = depth_intrin.fy;
	rsp->depth_ppx = depth_intrin.ppx;
	rsp->depth_ppy = depth_intrin.ppy;
    rsp->mu.unlock();

    while (true) {
        auto start = std::chrono::high_resolution_clock::now();

		rs2::frameset frames = p.wait_for_frames();
		// this handles the geometry so that the
		// x/y of the depth and color are the same
		frames = alignment.process(frames);
		rs2::video_frame color = frames.get_color_frame();
		rs2::depth_frame depth = frames.get_depth_frame();
		// color info 
        auto output = make_shared<RealSenseOutput>();
		try {
			output->colorframe = cv::Mat(color.get_height(), color.get_width(), CV_8UC3, (void*)(color.get_data()));
		} catch (std::exception& e) {
			// Catch exceptions, since constructing the matrix can fail when the size is 0.
			std::cout << "Exception while constructing matrix for color frame: " << e.what() << std::endl;
		}
		// depth info
		try {
			output->depthframe = cv::Mat(depth.get_height(), depth.get_width(), CV_16U, (void*)(depth.get_data()));
		} catch (std::exception& e) {
			std::cout << "Exception while constructing matrix for depth frame: " << e.what() << std::endl;
		}
		// pointcloud info
		rs2::pointcloud pc;
		pc.map_to(color);
		auto points = pc.calculate(depth);
		try {
			auto cloud = PCL_Conversion(points, color);
			output->colorcloud = *cloud;
		} catch (std::exception& e) {
			std::cout << "Exception while constructing pointcloud: " << e.what() << std::endl;
		}
        rso->mu.lock();
        rso->colorframe = output->colorframe;
        rso->depthframe = output->depthframe;
        rso->colorcloud = output->colorcloud;
        rso->mu.unlock();
        auto finish = std::chrono::high_resolution_clock::now();
        DEBUG(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() << "ms");
        cameraReady = true;
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
    if (argc == 1) {
        std::cout << "optional arguments are: port_number, image_width, image_height." << std::endl;
        exit(-1);
    }
    std::string port = "8085";
    std::string x_resolution = "";
    std::string y_resolution = "";
    if (argc > 1) {
        port =  argv[1];
    }
    if (argc > 2) {
        x_resolution =  argv[2];
    }
    if (argc > 3) {
        y_resolution =  argv[3];
    }
    auto rsp = make_unique<RealSenseProperties>();
    auto rso = make_unique<RealSenseOutput>();
    // start running camera thread
    std::thread t(cameraThread, rsp.get(), rso.get(), std::atoi(x_resolution.c_str()), std::atoi(y_resolution.c_str())); 
    RobotServiceImpl robotService;
    CameraServiceImpl cameraService(rsp.get(), rso.get());
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
