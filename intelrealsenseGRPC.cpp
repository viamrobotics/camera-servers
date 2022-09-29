#include <assert.h>
#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#include <array>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <queue>
#include <sstream>
#include <thread>
#include <tuple>
#include <vector>

#include "cameraserver.h"
#include "common/v1/common.grpc.pb.h"
#include "common/v1/common.pb.h"
#include "component/camera/v1/camera.grpc.pb.h"
#include "component/camera/v1/camera.pb.h"
#include "robot/v1/robot.grpc.pb.h"
#include "robot/v1/robot.pb.h"

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
using viam::component::camera::v1::DistortionParameters;
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

class RealSenseProperties {
   public:
    RealSenseProperties() {}
    // color camera
    int color_width;
    int color_height;
    float color_fx;
    float color_fy;
    float color_ppx;
    float color_ppy;
    std::string color_distortion_model;
    double color_distortion_parameters[5];
    // depth camera
    int depth_width;
    int depth_height;
    float depth_fx;
    float depth_fy;
    float depth_ppx;
    float depth_ppy;
    std::string depth_distortion_model;
    double depth_distortion_parameters[5];
};

class CameraServiceImpl final : public CameraService::Service {
   private:
    std::shared_ptr<RealSenseProperties> m_rsp;

   public:
    CameraServiceImpl(std::shared_ptr<RealSenseProperties> rsp) : m_rsp(rsp){};
    virtual ~CameraServiceImpl() = default;

    ::grpc::Status GetImage(ServerContext* context, const GetImageRequest* request,
                            GetImageResponse* response) override {
        std::string reqName = request->name();
        std::string reqMimeType = request->mime_type();
        if (!CameraState::get()->ready) {
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, "camera not ready");
        }
        auto output = CameraState::get()->getCameraOutput(0);
        if (reqName == "color") {
            if (reqMimeType.find("image/png") != std::string::npos) {  // look for substring to
                                                                       // catch "Lazy" MIME types
                response->set_mime_type(reqMimeType);
                std::vector<uchar> chbuf;
                chbuf.resize(5 * 1024 * 1024);
                cv::Mat cvConverted(m_rsp->color_height, m_rsp->color_width, CV_8UC3);
                cv::cvtColor(output->pic_cv, cvConverted, cv::COLOR_BGR2RGB);
                cv::imencode(".png", cvConverted, chbuf);
                std::string s(chbuf.begin(), chbuf.end());
                response->set_image(s);
                /* [RSDK-683] currently dont support raw 16bit RGBA inputs
                } else if (reqMimeType.find("image/vnd.viam.rgba") !=
                std::string::npos) { response->set_mime_type(reqMimeType);
                    cv::Mat cvConverted(m_rsp->color_height, m_rsp->color_width,
                CV_16UC4); cv::cvtColor(output->pic_cv, cvConverted,
                cv::COLOR_BGR2RGBA, 4); std::string s(cvConverted.datastart,
                cvConverted.dataend); response->set_image(s);
                */
            } else {
                // return jpeg if none specified
                if (reqMimeType.find("image/jpeg") != std::string::npos) {
                    response->set_mime_type(reqMimeType);
                } else if ((reqMimeType == "") ||
                           (reqMimeType.find("image/vnd.viam.rgba") != std::string::npos)) {
                    response->set_mime_type("image/jpeg");
                } else {
                    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                        "dont have mime type " + reqMimeType);
                }
                std::vector<uchar> chbuf;
                chbuf.resize(5 * 1024 * 1024);
                cv::Mat cvConverted(m_rsp->color_height, m_rsp->color_width, CV_8UC3);
                cv::cvtColor(output->pic_cv, cvConverted, cv::COLOR_BGR2RGB);
                cv::imencode(".jpg", cvConverted, chbuf);
                std::string s(chbuf.begin(), chbuf.end());
                response->set_image(s);
            }
        }
        if (reqName == "depth") {
            if (reqMimeType.find("image/jpeg") != std::string::npos) {
                response->set_mime_type(reqMimeType);
                std::vector<uchar> chbuf;
                chbuf.resize(5 * 1024 * 1024);
                cv::imencode(".jpg", output->depth_cv, chbuf);
                std::string s(chbuf.begin(), chbuf.end());
                response->set_image(s);
                /* [RSDK-683] currently dont support raw 16bit RGBA inputs
                } else if (reqMimeType.find("image/vnd.viam.rgba") !=
                std::string::npos) { response->set_mime_type(reqMimeType);
                    cv::Mat cvConverted(m_rsp->depth_height, m_rsp->depth_width,
                CV_8UC4); cv::cvtColor(output->depth_cv, cvConverted,
                cv::COLOR_GRAY2RGBA, 4); std::string s(cvConverted.datastart,
                cvConverted.dataend); response->set_image(s);
                */
            } else {  // return png if none specified, jpeg is lossy and will
                      // destroy depth pixel info.
                if (reqMimeType.find("image/png") != std::string::npos) {
                    response->set_mime_type(reqMimeType);
                } else if ((reqMimeType == "") ||
                           (reqMimeType.find("image/vnd.viam.rgba") != std::string::npos)) {
                    response->set_mime_type("image/png");
                } else {
                    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                        "dont have mime type " + reqMimeType);
                }
                std::vector<uchar> chbuf;
                chbuf.resize(5 * 1024 * 1024);
                cv::imencode(".png", output->depth_cv, chbuf);
                std::string s(chbuf.begin(), chbuf.end());
                response->set_image(s);
            }
        }
        return grpc::Status::OK;
    }

    ::grpc::Status GetPointCloud(ServerContext* context, const GetPointCloudRequest* request,
                                 GetPointCloudResponse* response) override {
        if (!CameraState::get()->ready) {
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, "camera not ready");
        }
        return grpc::Status(grpc::StatusCode::UNIMPLEMENTED, "GetPointCloud unmiplemented");
        /* point clouds slow the server loop down too much right now
        auto output = CameraState::get()->getCameraOutput(0);
        // create the pcd file
        std::stringbuf buffer;
        std::ostream oss(&buffer);
        oss << "VERSION .7\n"
            << "FIELDS x y z\n"
            << "SIZE 4 4 4 \n"
            << "TYPE F F F \n"
            << "COUNT 1 1 1 \n"
            << "WIDTH " << output->points.size() << "\n"
            << "HEIGHT " << 1 << "\n"
            << "VIEWPOINT 0 0 0 1 0 0 0\n"
            << "POINTS " << output->points.size() << "\n"
            << "DATA binary\n";
        for (int i = 0; i < output->points.size(); i++) {
            auto point = output->points[i];
            float x = point.x;
            float y = point.y;
            float z = point.z;
            buffer.sputn((const char*)&x, 4);
            buffer.sputn((const char*)&y, 4);
            buffer.sputn((const char*)&z, 4);
        }
        response->set_mime_type("pointcloud/pcd");
        response->set_point_cloud(buffer.str());
        return grpc::Status::OK;
        */
    }

    ::grpc::Status GetProperties(ServerContext* context, const GetPropertiesRequest* request,
                                 GetPropertiesResponse* response) override {
        IntrinsicParameters* intrinsics = response->mutable_intrinsic_parameters();
        DistortionParameters* distortion = response->mutable_distortion_parameters();
        auto reqName = request->name();
        if (reqName == "color") {
            response->set_supports_pcd(false);
            intrinsics->set_width_px(m_rsp->color_width);
            intrinsics->set_height_px(m_rsp->color_height);
            intrinsics->set_focal_x_px(m_rsp->color_fx);
            intrinsics->set_focal_y_px(m_rsp->color_fy);
            intrinsics->set_center_x_px(m_rsp->color_ppx);
            intrinsics->set_center_y_px(m_rsp->color_ppy);
            distortion->set_model(m_rsp->color_distortion_model);
            for (int i = 0; i < 5; i++) {
                distortion->add_parameters(m_rsp->color_distortion_parameters[i]);
            }
        }
        if (reqName == "depth") {
            response->set_supports_pcd(true);
            intrinsics->set_width_px(m_rsp->depth_width);
            intrinsics->set_height_px(m_rsp->depth_height);
            intrinsics->set_focal_x_px(m_rsp->depth_fx);
            intrinsics->set_focal_y_px(m_rsp->depth_fy);
            intrinsics->set_center_x_px(m_rsp->depth_ppx);
            intrinsics->set_center_y_px(m_rsp->depth_ppy);
            distortion->set_model(m_rsp->depth_distortion_model);
            for (int i = 0; i < 5; i++) {
                distortion->add_parameters(m_rsp->depth_distortion_parameters[i]);
            }
        }

        return grpc::Status::OK;
    }

    virtual std::string name() const { return std::string("IntelRealSenseServer"); }
};

class RobotServiceImpl final : public RobotService::Service {
   public:
    grpc::Status ResourceNames(ServerContext* context, const ResourceNamesRequest* request,
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

void cameraThread(rs2::pipeline p) {
    CameraState::get()->addCamera();
    CameraState::get()->ready = 0;
    rs2::align alignment(RS2_STREAM_COLOR);  // align to the color camera's origin
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();
        auto output = std::make_shared<CameraOutput>();
        rs2::frameset frames;
        uint timeout = 1000;
        bool succ = p.try_wait_for_frames(&frames, timeout);
        if (!succ) {
            std::cout << "intelGRPCserver: could not get frames from realsense after " << timeout
                      << "ms" << std::endl;
            continue;
        }
        try {
            frames = alignment.process(frames);
        } catch (std::exception& e) {
            // Catch exceptions, since alignment can fail
            std::cout << "intelGRPCserver: Exception while aligning images: " << e.what()
                      << std::endl;
            continue;
        }
        auto color = frames.get_color_frame();
        auto depth = frames.get_depth_frame();
        // get color
        try {
            output->pic_cv =
                cv::Mat(color.get_height(), color.get_width(), CV_8UC3, (void*)(color.get_data()));
        } catch (std::exception& e) {
            std::cout << "intelGRPCserver: Exception while constructing matrix "
                         "for color frame: "
                      << e.what() << std::endl;
            continue;
        }
        // get depth
        try {
            output->depth_cv =
                cv::Mat(depth.get_height(), depth.get_width(), CV_16U, (void*)(depth.get_data()));
            output->depth_cv.forEach<uint16_t>([](uint16_t &p, const int * position) -> void {
                p = p * 0.25;
            });
        } catch (std::exception& e) {
            std::cout << "intelGRPCserver: Exception while constructing matrix "
                         "for depth frame: "
                      << e.what() << std::endl;
            continue;
        }
        // set output
        CameraState::get()->setCameraOutput(0, output);
        auto finish = std::chrono::high_resolution_clock::now();
        DEBUG("time: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count()
              << "ms");
        if (time(0) - CameraState::get()->getLastRequest() > 30) {
            DEBUG("sleeping");
            sleep(1);
        }
        CameraState::get()->ready = 1;
    }
};

rs2::pipeline startPipeline(int width, int height, RealSenseProperties* rsp) {
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device selected_device;
    if (devices.size() == 0) {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        exit(-1);
    }
    selected_device = devices[0];

    rs2::pipeline pipe;
    auto serial = selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    std::cout << "intelGRPCserver: got serial: " << serial << std::endl;

    rs2::config cfg;
    cfg.enable_device(serial);
    try {
        cfg.enable_stream(RS2_STREAM_COLOR, width, height,
                          RS2_FORMAT_RGB8);  // 0 width or height indicates any
    } catch (std::exception& e) {
        std::cout << "intelGRPCserver: Exception while enabling (" << width << ", " << height
                  << ") stream: " << e.what() << std::endl;
        exit(-1);
    }
    cfg.enable_stream(RS2_STREAM_DEPTH);

    rs2::pipeline p(ctx);
    p.start(cfg);

    // get properties
    auto const color_stream =
        p.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto color_intrin = color_stream.get_intrinsics();
    auto const depth_stream =
        p.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto depth_intrin = depth_stream.get_intrinsics();
    rsp->color_width = color_intrin.width;
    rsp->color_height = color_intrin.height;
    rsp->color_fx = color_intrin.fx;
    rsp->color_fy = color_intrin.fy;
    rsp->color_ppx = color_intrin.ppx;
    rsp->color_ppy = color_intrin.ppy;
    rsp->color_distortion_model = "brown_conrady";
    for (int i = 0; i < 5; i++) {
        rsp->color_distortion_parameters[i] = double(color_intrin.coeffs[i]);
    }
    rsp->depth_width = depth_intrin.width;
    rsp->depth_height = depth_intrin.height;
    rsp->depth_fx = depth_intrin.fx;
    rsp->depth_fy = depth_intrin.fy;
    rsp->depth_ppx = depth_intrin.ppx;
    rsp->depth_ppy = depth_intrin.ppy;
    rsp->depth_distortion_model = "no_distortion";
    for (int i = 0; i < 5; i++) {
        rsp->depth_distortion_parameters[i] = double(depth_intrin.coeffs[i]);
    }

    return p;
};

int main(int argc, char* argv[]) {
    if (argc == 1) {
        std::cout << "intelGRPCserver: optional arguments are: port_number, "
                     "image_width, image_height."
                  << std::endl;
    }
    std::string port = "8085";
    std::string x_res = "";
    std::string y_res = "";
    if (argc > 1) {
        port = argv[1];
    }
    if (argc > 2) {
        x_res = argv[2];
    }
    if (argc > 3) {
        y_res = argv[3];
    }
    // get the pipeline and fill the properties
    auto rsp = make_shared<RealSenseProperties>();
    rs2::pipeline pipe =
        startPipeline(std::atoi(x_res.c_str()), std::atoi(y_res.c_str()), rsp.get());
    std::thread t(cameraThread, pipe);
    // define the services
    RobotServiceImpl robotService;
    CameraServiceImpl cameraService(rsp);
    grpc::EnableDefaultHealthCheckService(true);
    ServerBuilder builder;
    std::string address = "0.0.0.0:" + port;
    builder.AddListeningPort(address, grpc::InsecureServerCredentials());
    builder.RegisterService(&robotService);
    builder.RegisterService(&cameraService);
    std::unique_ptr<Server> server(builder.BuildAndStart());
    std::cout << "intelGRPCserver: Starting to listen on " << address << std::endl;
    server->Wait();
    return 0;
}
