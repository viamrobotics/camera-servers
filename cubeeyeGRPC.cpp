#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>

#include "proto/api/common/v1/common.grpc.pb.h"
#include "proto/api/common/v1/common.pb.h"
#include "proto/api/component/camera/v1/camera.grpc.pb.h"
#include "proto/api/component/camera/v1/camera.pb.h"
#include "proto/api/service/metadata/v1/metadata.grpc.pb.h"
#include "proto/api/service/metadata/v1/metadata.pb.h"

// clang-format off
#include <CubeEye/CubeEyeSink.h>
#include <CubeEye/CubeEyeBasicFrame.h>
#include <CubeEye/CubeEyePointCloudFrame.h>
// clang-format on

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

#include "cameraserver.h"
#define DEBUG(x)
using namespace std;
using namespace meere;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::ServerReaderWriter;
using grpc::ServerWriter;
using proto::api::common::v1::ResourceName;
using proto::api::component::camera::v1::CameraService;
using proto::api::component::camera::v1::GetFrameRequest;
using proto::api::component::camera::v1::GetFrameResponse;
using proto::api::component::camera::v1::GetPointCloudRequest;
using proto::api::component::camera::v1::GetPointCloudResponse;
using proto::api::service::metadata::v1::MetadataService;
using proto::api::service::metadata::v1::ResourcesRequest;
using proto::api::service::metadata::v1::ResourcesResponse;

std::atomic<bool> TOFdone{false};
bool TOFerror = false;

class MetadataServiceImpl final : public MetadataService::Service {
   public:
    grpc::Status Resources(ServerContext* context,
                           const ResourcesRequest* request,
                           ResourcesResponse* response) override {
        ResourceName* name = response->add_resources();
        name->set_namespace_("rdk");
        name->set_type("component");
        name->set_subtype("camera");
        name->set_name("Gray");

        ResourceName* name2 = response->add_resources();
        name2->set_namespace_("rdk");
        name2->set_type("component");
        name2->set_subtype("camera");
        name2->set_name("Depth");

        // leaving the both camera commented out for when we add this type to
        // rdk @JOHN

        // ResourceName* name3 = response->add_resources();
        //  name3->set_namespace_("rdk");
        //  name3->set_type("component");
        //  name3->set_subtype("camera");
        //  name3->set_name("Both");
        return grpc::Status::OK;
    }
};

class CameraServiceImpl final : public CameraService::Service,
                                public meere::sensor::sink,
                                public meere::sensor::prepared_listener {
   public:
    ::grpc::Status GetFrame(ServerContext* context,
                            const GetFrameRequest* request,
                            GetFrameResponse* response) override {
        if (mReadFrameThreadStart) {
            if (mFrameListQueue.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            auto _frames = std::move(mFrameListQueue.front());
            mFrameListQueue.pop();

            float max = 0;
            float min = 100000;
            int _frame_index = 0;
            auto reqName = request->name();
            char alpha = 255;
            for (auto itframe : (*_frames)) {
                if (itframe->frameType() ==
                    meere::sensor::CubeEyeFrame::FrameType_Depth) {
                    std::stringbuf buffer;
                    std::ostream os(&buffer);
                    auto _sptr_basic_frame =
                        meere::sensor::frame_cast_basic16u(itframe);
                    auto _sptr_frame_data =
                        _sptr_basic_frame->frameData();  // depth data array
                    int dim_x = _sptr_basic_frame->frameWidth();
                    int dim_y = _sptr_basic_frame->frameHeight();
                    response->set_width_px(dim_x);
                    response->set_height_px(dim_y);
                    // if (reqName == "Both")
                    //     response->set_mime_type("image/both");
                    if (reqName == "Depth")
                        response->set_mime_type("image/raw-depth");
                    if (reqName == "Gray")
                        response->set_mime_type("image/raw-rgba");
                    if ((reqName == "Both") || (reqName == "Depth")) {
                        os << "VERSIONX\n";
                        os << "2\n";
                        os << ".001\n";
                        os << dim_x << "\n";
                        os << dim_y << "\n";
                    }
                    for (int y = 0; y < dim_y; y++) {
                        for (int x = 0; x < dim_x; x++) {
                            _frame_index = y * dim_x + x;
                            short s = (*_sptr_frame_data)[_frame_index];

                            if (max < s) max = s;
                            if (min > s) min = s;
                            if ((reqName == "Both") || (reqName == "Depth")) {
                                buffer.sputn((const char*)&s, 2);
                            }
                        }
                    }
                    if ((reqName == "Both") || (reqName == "Gray")) {
                        float span = max - min;
                        // if (reqName == "Both")
                        //     os << "P6\n" << dim_x << " " << dim_y <<
                        //     "\n255\n";
                        for (int y = 0; y < dim_y; y++) {
                            for (int x = 0; x < dim_x; x++) {
                                _frame_index = y * dim_x + x;
                                short val = (*_sptr_frame_data)[_frame_index];
                                char clr = 0;
                                if (val > 0) {
                                    auto ratio = (val - min) / span;
                                    clr = (char)(60 + (int)(ratio * 192));
                                    if (clr > 250) clr = 250;
                                    if (clr < 0) clr = 0;
                                }
                                os << (char)clr;
                                os << (char)clr;
                                os << (char)clr;
                                if (reqName == "Gray") os << (char)alpha;
                            }
                        }
                    }
                    response->set_image(buffer.str());
                }
            }
        }
        return grpc::Status::OK;
    }

    ::grpc::Status GetPointCloud(ServerContext* context,
                                 const GetPointCloudRequest* request,
                                 GetPointCloudResponse* response) override {
        if (mReadFrameThreadStart) {
            if (mFrameListQueue.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            auto _frames = std::move(mFrameListQueue.front());
            mFrameListQueue.pop();

            float max = 0;
            float min = 100000;
            int _frame_index = 0;

            for (auto itframe : (*_frames)) {
                if (itframe->frameType() ==
                    meere::sensor::CubeEyeFrame::FrameType_PointCloud) {
                    // 32bits floating-point
                    if (itframe->frameDataType() ==
                        meere::sensor::CubeEyeData::DataType_32F) {
                        // casting 32bits point cloud frame
                        response->set_mime_type("pointcloud/pcd");
                        auto _sptr_pointcloud_frame =
                            meere::sensor::frame_cast_pcl32f(itframe);
                        auto _sptr_frame_dataX =
                            _sptr_pointcloud_frame
                                ->frameDataX();  // x-point data array
                        auto _sptr_frame_dataY =
                            _sptr_pointcloud_frame
                                ->frameDataY();  // y-point data array
                        auto _sptr_frame_dataZ =
                            _sptr_pointcloud_frame
                                ->frameDataZ();  // z-point data array
                        std::ostringstream oss;
                        oss << "VERSION .7\n"
                            << "FIELDS x y z rgb\n"
                            << "SIZE 4 4 4 4\n"
                            << "TYPE F F F I\n"
                            << "COUNT 1 1 1 1\n"
                            << "WIDTH "
                            << _sptr_pointcloud_frame->frameHeight() *
                                   _sptr_pointcloud_frame->frameWidth()
                            << "\n"
                            << "HEIGHT " << 1 << "\n"
                            << "VIEWPOINT 0 0 0 1 0 0 0\n"
                            << "POINTS "
                            << _sptr_pointcloud_frame->frameHeight() *
                                   _sptr_pointcloud_frame->frameWidth()
                            << "\n"
                            << "DATA ascii\n";

                        for (int y = 0;
                             y < _sptr_pointcloud_frame->frameHeight(); y++) {
                            for (int x = 0;
                                 x < _sptr_pointcloud_frame->frameWidth();
                                 x++) {
                                _frame_index =
                                    y * _sptr_pointcloud_frame->frameWidth() +
                                    x;
                                float s = (*_sptr_frame_dataZ)[_frame_index];
                                if (max < s) max = s;
                                if (min > s) min = s;
                            }
                        }
                        float span = max - min;
                        for (int y = 0;
                             y < _sptr_pointcloud_frame->frameHeight(); y++) {
                            for (int x = 0;
                                 x < _sptr_pointcloud_frame->frameWidth();
                                 x++) {
                                _frame_index =
                                    y * _sptr_pointcloud_frame->frameWidth() +
                                    x;

                                float val = (*_sptr_frame_dataZ)[_frame_index];
                                char clr = 0;
                                if (val > 0) {
                                    auto ratio = (val - min) / span;
                                    clr = (char)(60 + (int)(ratio * 192));
                                    if (clr > 255) clr = 255;
                                    if (clr < 0) clr = 0;
                                }
                                int rgb = 0;
                                rgb = rgb | (clr << 16);
                                rgb = rgb | (clr << 8);
                                rgb = rgb | (clr << 0);
                                float xframe =
                                    (*_sptr_frame_dataX)[_frame_index];
                                float yframe =
                                    (*_sptr_frame_dataY)[_frame_index];
                                float zframe =
                                    (*_sptr_frame_dataZ)[_frame_index];
                                oss << xframe << " " << yframe << " " << zframe
                                    << " " << rgb << "\n";
                            }
                        }
                        response->set_point_cloud(oss.str());
                    }
                }
            }
        }

        return grpc::Status::OK;
    }
    virtual std::string name() const { return std::string("CubeEyeServer"); }
    virtual void onCubeEyeCameraState(const meere::sensor::ptr_source source,
                                      meere::sensor::State state) {
        if (meere::sensor::State::Running == state) {
            std::cout << "Camera State = " << state << " Running" << std::endl;
            mReadFrameThreadStart = true;
        } else if (meere::sensor::State::Released == state) {
            std::cout << "Camera State = " << state << " Released" << std::endl;
        } else if (meere::sensor::State::Prepared == state) {
            std::cout << "Camera State = " << state << " Prepared" << std::endl;
        } else if (meere::sensor::State::Stopped == state) {
            std::cout << "Camera State = " << state << " Stopped" << std::endl;
            mReadFrameThreadStart = false;
        }
    }
    virtual void onCubeEyeCameraError(const meere::sensor::ptr_source source,
                                      meere::sensor::Error error) {
        // CubeEye.h has list of errors
        if (!TOFerror) {
            std::cerr << "Error with the camera device, error string : "
                      << error << endl;
            TOFerror = true;  // flag to try turning it off and on again
        }
    }
    virtual void onCubeEyeFrameList(
        const meere::sensor::ptr_source source,
        const meere::sensor::sptr_frame_list& frames) {
        if (mReadFrameThreadStart) {
            static constexpr size_t _MAX_FRAMELIST_SIZE = 4;
            if (_MAX_FRAMELIST_SIZE > mFrameListQueue.size()) {
                auto _copied_frame_list =
                    meere::sensor::copy_frame_list(frames);
                if (_copied_frame_list) {
                    mFrameListQueue.push(std::move(_copied_frame_list));
                }
            } else {
                mFrameListQueue.pop();
            }
        }
    }

   public:
    virtual void onCubeEyeCameraPrepared(
        const meere::sensor::ptr_camera camera) {
        std::cout << __FUNCTION__ << ":" << __LINE__ << " source("
                  << camera->source()->uri().c_str() << ")" << std::endl;
    }

   public:
    CameraServiceImpl() = default;
    virtual ~CameraServiceImpl() = default;

   protected:
    bool mReadFrameThreadStart;
    // std::thread mReadFrameThread;
    std::queue<meere::sensor::sptr_frame_list> mFrameListQueue;
};

void signal_callback_handler(int signum) {
    // On kill signal turn flag to allow main to finish executing
    // Which closes the webserver and camera
    TOFdone = true;
}

int main(int argc, char* argv[]) {
    // if (argc < 2) {
    //     std::cerr << "must supply grpc address" << std::endl;
    //     return 1;
    // }

    MetadataServiceImpl metadataService;
    CameraServiceImpl cameraService;
    ServerBuilder builder;
    builder.AddListeningPort("localhost:8085",
                             grpc::InsecureServerCredentials());
    builder.RegisterService(&metadataService);
    builder.RegisterService(&cameraService);
    // setup listener thread
    // MyListener listener;
    meere::sensor::add_prepared_listener(&cameraService);
    // search ToF camera source
    int selected_source = -1;
    meere::sensor::sptr_source_list _source_list =
        meere::sensor::search_camera_source();
    if (nullptr != _source_list && 0 < _source_list->size())
        selected_source = 0;
    else {
        std::cerr << "cannot find CubeEye! Try a hard restart. " << std::endl;
        return -1;
    }

    // create ToF camera
    meere::sensor::result _rt;
    meere::sensor::sptr_camera _camera =
        meere::sensor::create_camera(_source_list->at(selected_source));
    if (nullptr != _camera) {
        _camera->addSink(&cameraService);

        _rt = _camera->prepare();
        assert(meere::sensor::success == _rt);
        if (meere::sensor::success != _rt) {
            std::cerr << "_camera->prepare() failed." << std::endl;
            meere::sensor::destroy_camera(_camera);
            return -1;
        }

        // set wanted frame type : depth
        int wantedFrame = meere::sensor::CubeEyeFrame::FrameType_Depth |
                          meere::sensor::CubeEyeFrame::FrameType_PointCloud;

        _rt = _camera->run(wantedFrame);
        assert(meere::sensor::success == _rt);
        if (meere::sensor::success != _rt) {
            std::cerr << "_camera->run() failed." << std::endl;
            meere::sensor::destroy_camera(_camera);
            return -1;
        }
        // change camera framerate to 30 fps
        std::string key("");
        meere::sensor::sptr_property _prop;
        meere::sensor::result_property _rt_prop;
        key = "framerate";
        _prop = meere::sensor::make_property_8u(key, 30);
        _rt = _camera->setProperty(_prop);

        if (!TOFdone.is_lock_free())
            return 10;  // make sure we can actually use signal
        signal(SIGTERM,
               signal_callback_handler);  // if we kill, dont break the camera
                                          // by safely turning stuff off
        signal(SIGINT, signal_callback_handler);  // in this case exiting the
                                                  // while loop below
        signal(SIGQUIT, signal_callback_handler);

        std::unique_ptr<Server> server(builder.BuildAndStart());
        std::cout << "Server listening on "
                  << "localhost:8085" << std::endl;
        while (!TOFdone) {  // keep going until we stop
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            // if an error in the camera occurs(sometimes timeout error on
            // startup) restart the camera
            if (TOFerror) {
                std::cerr << "Restarting Cube Eye to Fix " << endl;
                _camera->stop();

                _rt = _camera->run(wantedFrame);
                TOFerror = false;
            }
        }
        // turn stuff off

        meere::sensor::destroy_camera(_camera);
        // _camera.reset();
    }
    return 0;
}
