#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include "proto/api/service/v1/metadata.grpc.pb.h"
#include "proto/api/service/v1/metadata.pb.h"
#include "proto/api/v1/robot.pb.h"
#include "proto/api/v1/robot.grpc.pb.h"
#include "proto/api/common/v1/common.pb.h"
#include "proto/api/common/v1/common.grpc.pb.h"
#include "proto/api/component/v1/camera.grpc.pb.h"
#include "proto/api/component/v1/camera.pb.h"


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
using proto::api::v1::RobotService;
using proto::api::service::v1::MetadataService;
using proto::api::v1::StatusRequest;
using proto::api::v1::StatusResponse;
using proto::api::v1::ConfigRequest;
using proto::api::v1::ConfigResponse;
using proto::api::service::v1::ResourcesRequest;
using proto::api::service::v1::ResourcesResponse;
using proto::api::service::v1::ResourceName;
using proto::api::component::v1::CameraService;
using proto::api::component::v1::CameraServiceFrameRequest;
using proto::api::component::v1::CameraServiceFrameResponse;
using proto::api::component::v1::CameraServicePointCloudRequest;
using proto::api::component::v1::CameraServicePointCloudResponse;

std::atomic<bool> TOFdone{false};
bool TOFerror = false;

class RobotServiceImpl final : public RobotService::Service {
 public:
 grpc::Status Config(ServerContext* context, const ConfigRequest* request, ConfigResponse* response) override{
return grpc::Status::OK;
 }
  grpc::Status Status(ServerContext* context, const StatusRequest* request, StatusResponse* response) override {
    (*response->mutable_status()->mutable_bases())["base1"] = true;
    return grpc::Status::OK;
  }

};

class MetadataServiceImpl final : public MetadataService::Service  {
 public:
  grpc::Status Resources(ServerContext* context, const ResourcesRequest* request, ResourcesResponse* response) override{

    ResourceName* name = response->add_resources();    
    name->set_namespace_("core");
    name->set_type("component");
    name->set_subtype("camera");
    name->set_name("CubeEye");

    return grpc::Status::OK;
  }
};

class CameraServiceImpl final :  public CameraService::Service,
                                public meere::sensor::sink,
                                public meere::sensor::prepared_listener {
 public:
::grpc::Status Frame(ServerContext* context, const CameraServiceFrameRequest* request, CameraServiceFrameResponse* response) override{
    int sanitycnt = 0;
    char alpha = 255;
    if(mReadFrameThreadStart){
        if (mFrameListQueue.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            auto _frames = std::move(mFrameListQueue.front());
            mFrameListQueue.pop();

            float max = 0;//
            float min = 100000;//
            int _frame_index = 0;
            
            for (auto itframe : (*_frames)) {
                
                
                if(itframe->frameType() == meere::sensor::CubeEyeFrame::FrameType_Depth){
                    std::stringbuf buffer;
                    auto _sptr_basic_frame =meere::sensor::frame_cast_basic16u(itframe);
                    auto _sptr_frame_data =_sptr_basic_frame->frameData();  // depth data array
                    response->set_dim_x(_sptr_basic_frame->frameWidth());
                    response->set_dim_y(_sptr_basic_frame->frameHeight());
                    response->set_mime_type("image/raw-rgba");
                    
                    for (int y = 0; y < _sptr_basic_frame->frameHeight();y++) {
                        for (int x = 0;x < _sptr_basic_frame->frameWidth();x++) {
                            _frame_index = y * _sptr_basic_frame->frameWidth() + x;
                            short s =(*_sptr_frame_data)[_frame_index];
                                        
                            if (max < s) max = s;
                            if (min > s) min = s;
                            // buffer.sputn((const char*)&s, 2);
                        }
                    }
                    float span = max - min;
                    for (int y = 0; y < _sptr_basic_frame->frameHeight();y++) {
                        for (int x = 0;x < _sptr_basic_frame->frameWidth();x++) {
                            _frame_index = y * _sptr_basic_frame->frameWidth() + x;
                            short val =(*_sptr_frame_data)[_frame_index];
                            char clr = 0;
                            if (val > 0){
                                auto ratio = (val - min) / span;
                                clr = (char)(60 + (int)(ratio * 192));  
                                if (clr > 250) clr = 250;
                                if (clr < 0) clr = 0;
                            }
                            buffer.sputn((const char*)&clr, 1);
                            buffer.sputn((const char*)&clr, 1);
                            buffer.sputn((const char*)&clr, 1);
                            buffer.sputn((const char*)&alpha, 1);
                            // if (640/2 == x && 480/2 == y) {
                            //     std::cout << clr << std::endl;
                            //     std::cout << val << std::endl;
                            // }
                        }
                    }
                    response->set_frame(buffer.str());
                }
            
                sanitycnt++;
                // std::cout << sanitycnt << std::endl;
            }
            
        // }
    }
return grpc::Status::OK;
}
::grpc::Status PointCloud(ServerContext* context, const CameraServicePointCloudRequest* request, CameraServicePointCloudResponse* response) override{
char alpha = 255;
    if(mReadFrameThreadStart){
        if (mFrameListQueue.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            auto _frames = std::move(mFrameListQueue.front());
            mFrameListQueue.pop();

            float max = 0;//
            float min = 100000;//
            int _frame_index = 0;
            
            for (auto itframe : (*_frames)) {
                if (itframe->frameType() == meere::sensor::CubeEyeFrame::FrameType_PointCloud) {
                    // 32bits floating-point
                    if (itframe->frameDataType() == meere::sensor::CubeEyeData::DataType_32F) {
                        // casting 32bits point cloud frame
                        response->set_mime_type("pointcloud/pcd");
                        auto _sptr_pointcloud_frame = meere::sensor::frame_cast_pcl32f(itframe);
                        auto _sptr_frame_dataX = _sptr_pointcloud_frame->frameDataX(); // x-point data array
                        auto _sptr_frame_dataY = _sptr_pointcloud_frame->frameDataY(); // y-point data array
                        auto _sptr_frame_dataZ = _sptr_pointcloud_frame->frameDataZ(); // z-point data array
                        std::ostringstream oss;
                        oss << "VERSION .7\n"
                            << "FIELDS x y z rgb\n"
                            << "SIZE 4 4 4 4\n"
                            << "TYPE F F F I\n"
                            << "COUNT 1 1 1 1\n"
                            << "WIDTH " << _sptr_pointcloud_frame->frameHeight()*_sptr_pointcloud_frame->frameWidth() << "\n"
                            << "HEIGHT " << 1 << "\n"
                            << "VIEWPOINT 0 0 0 1 0 0 0\n"
                            << "POINTS " << _sptr_pointcloud_frame->frameHeight()*_sptr_pointcloud_frame->frameWidth() << "\n"
                            << "DATA ascii\n";

                        for (int y = 0 ; y < _sptr_pointcloud_frame->frameHeight(); y++) {
                            for (int x = 0 ; x < _sptr_pointcloud_frame->frameWidth(); x++) {
                                _frame_index = y * _sptr_pointcloud_frame->frameWidth() + x;
                                float s = (*_sptr_frame_dataZ)[_frame_index];
                                if (max < s) max = s;
                                if (min > s) min = s;
                            }
                        }
                        float span = max - min;
                        for (int y = 0 ; y < _sptr_pointcloud_frame->frameHeight(); y++) {
                            for (int x = 0 ; x < _sptr_pointcloud_frame->frameWidth(); x++) {
                                _frame_index = y * _sptr_pointcloud_frame->frameWidth() + x;

                                float val = (*_sptr_frame_dataZ)[_frame_index];
                                char clr = 0;
                                if (val > 0){
                                    auto ratio = (val - min) / span;
                                    clr = (char)(60 + (int)(ratio * 192));  
                                    if (clr > 250) clr = 250;
                                    if (clr < 0) clr = 0;
                                }
                                int rgb = 0;
                                rgb = rgb | (clr << 16);
                                rgb = rgb | (clr << 8);
                                rgb = rgb | (clr << 0);
                                float xframe = (*_sptr_frame_dataX)[_frame_index];
                                float yframe = (*_sptr_frame_dataY)[_frame_index];
                                float zframe = (*_sptr_frame_dataZ)[_frame_index];
                                oss << xframe << " " << yframe << " " << zframe << " " << rgb << "\n";
                                if (x == 640/2 && y == 480/2) 
                                std::cout << xframe << " " << yframe << " " << zframe << " " << rgb << std::endl;
                            }
                        }
                        response->set_frame(oss.str());
                    }
                }
            }
    }

return grpc::Status::OK;
}
 virtual std::string name() const { return std::string("CubeEyeServer"); }
 virtual void onCubeEyeCameraState(const meere::sensor::ptr_source source,
                                      meere::sensor::State state) {
        std::cout << "Camera State = " << state << std::endl;
        if (meere::sensor::State::Running == state) {
            std::cout << " Running" << std::endl;
            mReadFrameThreadStart = true;
        } else if (meere::sensor::State::Released == state) {
            std::cout << " Released" << std::endl;
        } else if (meere::sensor::State::Prepared == state) {
            std::cout << " Prepared" << std::endl;
        } else if (meere::sensor::State::Stopped == state) {
            std::cout << " Stopped" << std::endl;
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
            }else{
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

    void getLensInfo(meere::sensor::sptr_camera camera,
                     meere::sensor::result _rt) {
        // get lens parameters
        {
            auto _lenses = camera->lenses();
            std::cout << "count of Lenses : " << _lenses << std::endl;
            for (size_t i = 0; i < _lenses; i++) {
                std::cout << "Lens index : " << i << std::endl;

                auto _fov = camera->fov(i);
                std::cout << "    FoV : " << std::get<0>(_fov) << "(H) x "
                          << std::get<1>(_fov) << "(V)" << std::endl;

                meere::sensor::IntrinsicParameters parameters;
                if (meere::sensor::success ==
                    (_rt = camera->intrinsicParameters(parameters, i))) {
                    std::cout << "    IntrinsicParameters :" << std::endl;
                    std::cout
                        << "        ForcalLength(fx) = " << parameters.forcal.fx
                        << std::endl;
                    std::cout
                        << "        ForcalLength(fy) = " << parameters.forcal.fy
                        << std::endl;
                    std::cout << "        PrincipalPoint(cx) = "
                              << parameters.principal.cx << std::endl;
                    std::cout << "        PrincipalPoint(cy) = "
                              << parameters.principal.cy << std::endl;
                }

                meere::sensor::DistortionCoefficients coefficients;
                if (meere::sensor::success ==
                    (_rt = camera->distortionCoefficients(coefficients, i))) {
                    std::cout << "    DistortionCoefficients :" << std::endl;
                    std::cout << "        RadialCoefficient(K1) = "
                              << coefficients.radial.k1 << std::endl;
                    std::cout << "        RadialCoefficient(K2) = "
                              << coefficients.radial.k2 << std::endl;
                    std::cout << "        RadialCoefficient(K3) = "
                              << coefficients.radial.k3 << std::endl;
                    std::cout << "        TangentialCoefficient(P1) = "
                              << coefficients.tangential.p1 << std::endl;
                    std::cout << "        TangentialCoefficient(P2) = "
                              << coefficients.tangential.p2 << std::endl;
                    std::cout << "        skewCoefficient = "
                              << coefficients.skewCoefficient << std::endl;
                }
            }
        }
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
if (argc < 2) {
    std::cerr << "must supply grpc address" << std::endl;
    return 1;
  }
  RobotServiceImpl robotService;
  MetadataServiceImpl metadataService;
  CameraServiceImpl cameraService;
  ServerBuilder builder;
  builder.AddListeningPort(argv[1], grpc::InsecureServerCredentials());
  builder.RegisterService(&robotService);
  builder.RegisterService(&metadataService);
  builder.RegisterService(&cameraService);
    // setup listener thread
    // MyListener listener;
    meere::sensor::add_prepared_listener(&cameraService);

    // search ToF camera source
    int selected_source = -1;
    meere::sensor::sptr_source_list _source_list =
        meere::sensor::search_camera_source();

    if (nullptr != _source_list) {
        int i = 0;
        for (auto it : (*_source_list)) {
            std::cout << i++ << ") source name : " << it->name()
                      << ", serialNumber : " << it->serialNumber()
                      << ", uri : " << it->uri() << std::endl;
        }
    }
    if (nullptr != _source_list && 0 < _source_list->size()) {
        if (1 < _source_list->size()) {
            std::cout << "Please enter the desired source number." << std::endl;
            scanf("%d", &selected_source);
            getchar();
        } else {
            selected_source = 0;
        }
    } else {
        std::cerr << "no search device!" << std::endl;
        return -1;
    }

    if (0 > selected_source) {
        std::cerr << "invalid selected source number!" << std::endl;
        return -1;
    }

    // CameraState::get()->cameras.push_back(0);

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
        int wantedFrame = meere::sensor::CubeEyeFrame::FrameType_Depth | meere::sensor::CubeEyeFrame::FrameType_PointCloud;

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
        std::cout << "Server listening on " << argv[1] << std::endl;
        while (!TOFdone) {          // keep going until we stop
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
