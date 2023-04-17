#include <arpa/inet.h>
#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <turbojpeg.h>

#include <future>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <thread>
#include <tuple>
#include <vector>

#include "common/v1/common.grpc.pb.h"
#include "common/v1/common.pb.h"
#include "component/camera/v1/camera.grpc.pb.h"
#include "component/camera/v1/camera.pb.h"
#include "robot/v1/robot.grpc.pb.h"
#include "robot/v1/robot.pb.h"
#include "third_party/fpng.h"
#include "third_party/lodepng.h"

using namespace std;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using viam::common::v1::ResourceName;
using viam::component::camera::v1::CameraService;
using viam::component::camera::v1::DistortionParameters;
using viam::component::camera::v1::GetImageRequest;
using viam::component::camera::v1::GetImageResponse;
using viam::component::camera::v1::GetPropertiesRequest;
using viam::component::camera::v1::GetPropertiesResponse;
using viam::component::camera::v1::IntrinsicParameters;
using viam::robot::v1::ResourceNamesRequest;
using viam::robot::v1::ResourceNamesResponse;
using viam::robot::v1::RobotService;

#define htonll(x) \
    ((1 == htonl(1)) ? (x) : ((uint64_t)htonl((x)&0xFFFFFFFF) << 32) | htonl((x) >> 32))

struct DeviceProperties {
    const int colorWidth;
    const int colorHeight;
    const bool disableColor;
    const int depthWidth;
    const int depthHeight;
    const bool disableDepth;
    bool shouldRun;
    bool isRunning;
    std::mutex mutex;

    DeviceProperties(int colorWidth_, int colorHeight_, bool disableColor_, int depthWidth_,
                     int depthHeight_, bool disableDepth_)
        : colorWidth(colorWidth_),
          colorHeight(colorHeight_),
          disableColor(disableColor_),
          depthWidth(depthWidth_),
          depthHeight(depthHeight_),
          disableDepth(disableDepth_),
          shouldRun(true),
          isRunning(false) {}
};

struct CameraProperties {
    int width;
    int height;
    float fx;
    float fy;
    float ppx;
    float ppy;
    string distortionModel;
    double distortionParameters[5];
};

struct RealSenseProperties {
    CameraProperties color;
    CameraProperties depth;
    float depthScaleMm;
};

struct PipelineWithProperties {
    rs2::pipeline pipeline;
    RealSenseProperties properties;
};

struct AtomicFrameSet {
    std::mutex mutex;
    rs2::frame colorFrame;
    shared_ptr<vector<uint16_t>> depthFrame;
};

// Global AtomicFrameSet
AtomicFrameSet latestFrames;

bool DEBUG = false;
const uint32_t rgbaMagicNumber =
    htonl(1380401729);  // the utf-8 binary encoding for "RGBA", big-endian
const size_t rgbaMagicByteCount =
    sizeof(uint32_t);  // number of bytes used to represent the rgba magic number
const size_t rgbaWidthByteCount =
    sizeof(uint32_t);  // number of bytes used to represent rgba image width
const size_t rgbaHeightByteCount =
    sizeof(uint32_t);  // number of bytes used to represent rgba image height

const uint64_t depthMagicNumber =
    htonll(4919426490892632400);  // the utf-8 binary encoding for "DEPTHMAP", big-endian
const size_t depthMagicByteCount =
    sizeof(uint64_t);  // number of bytes used to represent the depth magic number
const size_t depthWidthByteCount =
    sizeof(uint64_t);  // number of bytes used to represent depth image width
const size_t depthHeightByteCount =
    sizeof(uint64_t);  // number of bytes used to represent depth image height

// COLOR responses
tuple<vector<uint8_t>, bool> encodeColorPNG(const uint8_t* data, const int width,
                                            const int height) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (DEBUG) {
        start = chrono::high_resolution_clock::now();
    }

    vector<uint8_t> encoded;
    if (!fpng::fpng_encode_image_to_memory(data, width, height, 3, encoded)) {
        cerr << "[GetImage]  failed to encode color PNG" << endl;
        return {encoded, false};
    }

    if (DEBUG) {
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
        cout << "[GetImage]  PNG color encode:      " << duration.count() << "ms\n";
    }

    return {encoded, true};
}

grpc::Status encodeColorPNGToResponse(GetImageResponse* response, const uint8_t* data,
                                      const int width, const int height) {
    const auto& [encoded, ok] = encodeColorPNG(data, width, height);
    if (!ok) {
        return grpc::Status(grpc::StatusCode::INTERNAL, "failed to encode color PNG");
    }
    response->set_mime_type("image/png");
    response->set_image(encoded.data(), encoded.size());
    return grpc::Status::OK;
}

tuple<unsigned char*, long unsigned int, bool> encodeJPEG(const unsigned char* data,
                                                          const int width, const int height) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (DEBUG) {
        start = chrono::high_resolution_clock::now();
    }

    unsigned char* encoded = nullptr;
    long unsigned int encodedSize = 0;
    tjhandle handle = tjInitCompress();
    if (handle == nullptr) {
        cerr << "[GetImage]  failed to init JPEG compressor" << endl;
        return {encoded, encodedSize, false};
    }
    tjCompress2(handle, data, width, 0, height, TJPF_RGB, &encoded, &encodedSize, TJSAMP_420, 75,
                TJFLAG_FASTDCT);
    tjDestroy(handle);

    if (DEBUG) {
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
        cout << "[GetImage]  JPEG color encode:     " << duration.count() << "ms\n";
    }

    return {encoded, encodedSize, true};
}

grpc::Status encodeJPEGToResponse(GetImageResponse* response, const unsigned char* data,
                                  const int width, const int height) {
    const auto& [encoded, encodedSize, ok] = encodeJPEG(data, width, height);
    if (!ok) {
        return grpc::Status(grpc::StatusCode::INTERNAL, "failed to encode color JPEG");
    }
    response->set_mime_type("image/jpeg");
    response->set_image(encoded, encodedSize);
    tjFree(encoded);
    return grpc::Status::OK;
}

tuple<unsigned char*, size_t, bool> encodeColorRAW(const unsigned char* data, const uint32_t width,
                                                   const uint32_t height) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (DEBUG) {
        start = chrono::high_resolution_clock::now();
    }
    // set size of raw file
    size_t pixelByteCount = 4 * width * height;
    uint32_t widthToEncode = htonl(width);    // make sure everything is big-endian
    uint32_t heightToEncode = htonl(height);  // make sure everything is big-endian
    size_t totalByteCount =
        rgbaMagicByteCount + rgbaWidthByteCount + rgbaHeightByteCount + pixelByteCount;
    // memcpy data into buffer
    unsigned char* rawBuf = new unsigned char[totalByteCount];
    int offset = 0;
    std::memcpy(rawBuf + offset, &rgbaMagicNumber, rgbaMagicByteCount);
    offset += rgbaMagicByteCount;
    std::memcpy(rawBuf + offset, &widthToEncode, rgbaWidthByteCount);
    offset += rgbaWidthByteCount;
    std::memcpy(rawBuf + offset, &heightToEncode, rgbaHeightByteCount);
    offset += rgbaHeightByteCount;
    int pixelOffset = 0;
    uint8_t alphaValue = 255;  // alpha  channel is always 255 for color images
    for (int i = 0; i < width * height; i++) {
        std::memcpy(rawBuf + offset, data + pixelOffset, 3);  // 3 bytes for RGB
        std::memcpy(rawBuf + offset + 3, &alphaValue, 1);     // 1 byte for A
        pixelOffset += 3;
        offset += 4;
    }
    if (DEBUG) {
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
        cout << "[GetImage]  RAW color encode:      " << duration.count() << "ms\n";
    }

    return {rawBuf, totalByteCount, true};
}

grpc::Status encodeColorRAWToResponse(GetImageResponse* response, const unsigned char* data,
                                      const uint width, const uint height) {
    const auto& [encoded, encodedSize, ok] = encodeColorRAW(data, width, height);
    if (!ok) {
        std::free(encoded);
        return grpc::Status(grpc::StatusCode::INTERNAL, "failed to encode color RAW");
    }
    response->set_mime_type("image/vnd.viam.rgba");
    response->set_image(encoded, encodedSize);
    std::free(encoded);
    return grpc::Status::OK;
}

// DEPTH responses
tuple<unsigned char*, size_t, bool> encodeDepthPNG(const unsigned char* data, const uint width,
                                                   const uint height) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (DEBUG) {
        start = chrono::high_resolution_clock::now();
    }

    unsigned char* encoded = 0;
    size_t encoded_size = 0;
    unsigned result =
        lodepng_encode_memory(&encoded, &encoded_size, data, width, height, LCT_GREY, 16);
    if (result != 0) {
        cerr << "[GetImage]  failed to encode depth PNG" << endl;
        return {encoded, encoded_size, false};
    }

    if (DEBUG) {
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
        cout << "[GetImage]  PNG depth encode:      " << duration.count() << "ms\n";
    }

    return {encoded, encoded_size, true};
}

grpc::Status encodeDepthPNGToResponse(GetImageResponse* response, const unsigned char* data,
                                      const uint width, const uint height) {
    const auto& [encoded, encoded_size, ok] = encodeDepthPNG(data, width, height);
    if (!ok) {
        return grpc::Status(grpc::StatusCode::INTERNAL, "failed to encode depth PNG");
    }
    response->set_mime_type("image/png");
    response->set_image(encoded, encoded_size);
    std::free(encoded);
    return grpc::Status::OK;
}

tuple<unsigned char*, size_t, bool> encodeDepthRAW(const unsigned char* data, const uint64_t width,
                                                   const uint64_t height) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (DEBUG) {
        start = chrono::high_resolution_clock::now();
    }
    // Depth header contains 8 bytes worth of magic number, followed by 8 bytes for width and
    // another 8 bytes for height each pixel has 2 bytes.
    size_t pixelByteCount = 2 * width * height;
    uint64_t widthToEncode = htonll(width);    // make sure everything is big-endian
    uint64_t heightToEncode = htonll(height);  // make sure everything is big-endian
    size_t totalByteCount =
        depthMagicByteCount + depthWidthByteCount + depthHeightByteCount + pixelByteCount;
    // memcpy data into buffer
    unsigned char* rawBuf = new unsigned char[totalByteCount];
    int offset = 0;
    std::memcpy(rawBuf + offset, &depthMagicNumber, depthMagicByteCount);
    offset += depthMagicByteCount;
    std::memcpy(rawBuf + offset, &widthToEncode, depthWidthByteCount);
    offset += depthWidthByteCount;
    std::memcpy(rawBuf + offset, &heightToEncode, depthHeightByteCount);
    offset += depthHeightByteCount;
    std::memcpy(rawBuf + offset, data, pixelByteCount);

    if (DEBUG) {
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
        cout << "[GetImage]  RAW depth encode:      " << duration.count() << "ms\n";
    }

    return {rawBuf, totalByteCount, true};
}

grpc::Status encodeDepthRAWToResponse(GetImageResponse* response, const unsigned char* data,
                                      const uint width, const uint height) {
    const auto& [encoded, encodedSize, ok] = encodeDepthRAW(data, width, height);
    if (!ok) {
        std::free(encoded);
        return grpc::Status(grpc::StatusCode::INTERNAL, "failed to encode depth RAW");
    }
    response->set_mime_type("image/vnd.viam.dep");
    response->set_image(encoded, encodedSize);
    std::free(encoded);
    return grpc::Status::OK;
}

// CAMERA service
class CameraServiceImpl final : public CameraService::Service {
   private:
    RealSenseProperties props;
    AtomicFrameSet& frameSet;
    const bool disableColor;
    const bool disableDepth;

   public:
    CameraServiceImpl(RealSenseProperties props, AtomicFrameSet& frameSet, const bool disableColor,
                      const bool disableDepth)
        : props(props),
          frameSet(frameSet),
          disableColor(disableColor),
          disableDepth(disableDepth){};

    ::grpc::Status GetImage(ServerContext* context, const GetImageRequest* request,
                            GetImageResponse* response) override {
        const string reqName = request->name();
        const string reqMimeType = request->mime_type();

        auto start = chrono::high_resolution_clock::now();

        // FUTURE(erd): we could track the last frame encode so as to not duplicate work if we
        // are ahead of the frame loop.
        this->frameSet.mutex.lock();
        auto latestColorFrame = this->frameSet.colorFrame;
        auto latestDepthFrame = this->frameSet.depthFrame;
        this->frameSet.mutex.unlock();

        if (reqName.compare("color") == 0) {
            if (this->disableColor) {
                return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "color disabled");
            }
            if (reqMimeType.compare("image/png") == 0 ||
                reqMimeType.compare("image/png+lazy") == 0) {
                encodeColorPNGToResponse(response, (const uint8_t*)latestColorFrame.get_data(),
                                         this->props.color.width, this->props.color.height);
            } else if (reqMimeType.compare("image/vnd.viam.rgba") == 0) {
                encodeColorRAWToResponse(response,
                                         (const unsigned char*)latestColorFrame.get_data(),
                                         this->props.color.width, this->props.color.height);
            } else {
                encodeJPEGToResponse(response, (const unsigned char*)latestColorFrame.get_data(),
                                     this->props.color.width, this->props.color.height);
            }
        } else if (reqName.compare("depth") == 0) {
            if (this->disableDepth) {
                return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "depth disabled");
            }
            if (reqMimeType.compare("image/vnd.viam.dep") == 0) {
                encodeDepthRAWToResponse(response, (const unsigned char*)latestDepthFrame->data(),
                                         this->props.depth.width, this->props.depth.height);
            } else {
                encodeDepthPNGToResponse(response, (const unsigned char*)latestDepthFrame->data(),
                                         this->props.depth.width, this->props.depth.height);
            }
        }

        if (DEBUG) {
            auto stop = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
            cout << "[GetImage]  total:           " << duration.count() << "ms\n";
        }

        return grpc::Status::OK;
    }

    ::grpc::Status GetProperties(ServerContext* context, const GetPropertiesRequest* request,
                                 GetPropertiesResponse* response) override {
        IntrinsicParameters* intrinsics = response->mutable_intrinsic_parameters();
        DistortionParameters* distortion = response->mutable_distortion_parameters();

        const string reqName = request->name();

        auto fillResp = [response, intrinsics, distortion](auto props, bool supportsPCD) {
            response->set_supports_pcd(supportsPCD);
            intrinsics->set_width_px(props.width);
            intrinsics->set_height_px(props.height);
            intrinsics->set_focal_x_px(props.fx);
            intrinsics->set_focal_y_px(props.fy);
            intrinsics->set_center_x_px(props.ppx);
            intrinsics->set_center_y_px(props.ppy);
            distortion->set_model(props.distortionModel);
            for (int i = 0; i < 5; i++) {
                distortion->add_parameters(props.distortionParameters[i]);
            }
        };

        if (reqName.compare("color") == 0) {
            fillResp(this->props.color, false);
        } else if (reqName.compare("depth") == 0) {
            fillResp(this->props.depth, true);
        }

        return grpc::Status::OK;
    }
};

class RobotServiceImpl final : public RobotService::Service {
   public:
    grpc::Status ResourceNames(ServerContext* context, const ResourceNamesRequest* request,
                               ResourceNamesResponse* response) override {
        ResourceName* colorName = response->add_resources();
        colorName->set_namespace_("rdk");
        colorName->set_type("component");
        colorName->set_subtype("camera");
        colorName->set_name("color");

        ResourceName* depthName = response->add_resources();
        depthName->set_namespace_("rdk");
        depthName->set_type("component");
        depthName->set_subtype("camera");
        depthName->set_name("depth");
        return grpc::Status::OK;
    }
};

// align to the color camera's origin when color and depth enabled
const rs2::align FRAME_ALIGNMENT = RS2_STREAM_COLOR;

void frameLoop(rs2::pipeline pipeline, AtomicFrameSet& frameSet, promise<void>& ready,
               DeviceProperties& deviceProps, float depthScaleMm) {
    bool readyOnce = false;
    {
        std::lock_guard<std::mutex> lock(deviceProps.mutex);
        deviceProps.shouldRun = true;
        deviceProps.isRunning = true;
    }
    while (true) {
        {
            std::lock_guard<std::mutex> lock(deviceProps.mutex);
            if (!deviceProps.shouldRun) {
                pipeline.stop();
                cout << "[frameLoop] pipeline stopped exiting thread" << endl;
                deviceProps.isRunning = false;
                break;
            }
        }
        auto failureWait = 5ms;

        auto start = chrono::high_resolution_clock::now();

        rs2::frameset frames;
        const uint timeoutMillis = 2000;
        /*
            D435 1920x1080 RGB + Depth ~20ms on a Raspberry Pi 4 Model B
        */
        bool succ = pipeline.try_wait_for_frames(&frames, timeoutMillis);
        if (!succ) {
            if (DEBUG) {
                cerr << "[frameLoop] could not get frames from realsense after " << timeoutMillis
                     << "ms" << endl;
            }
            this_thread::sleep_for(failureWait);
            continue;
        }
        if (DEBUG) {
            auto stop = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
            cout << "[frameLoop] wait for frames: " << duration.count() << "ms\n";
        }

        if (!deviceProps.disableColor && !deviceProps.disableDepth) {
            auto start = chrono::high_resolution_clock::now();

            try {
                frames = FRAME_ALIGNMENT.process(frames);
            } catch (const exception& e) {
                cerr << "[frameLoop] exception while aligning images: " << e.what() << endl;
                this_thread::sleep_for(failureWait);
                continue;
            }

            if (DEBUG) {
                auto stop = chrono::high_resolution_clock::now();
                auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
                cout << "[frameLoop] frame alignment: " << duration.count() << "ms\n";
            }
        }
        // scale every pixel value to be depth in units of mm
        unique_ptr<vector<uint16_t>> depthFrameScaled;
        if (!deviceProps.disableDepth) {
            auto depthFrame = frames.get_depth_frame();
            auto depthWidth = depthFrame.get_width();
            auto depthHeight = depthFrame.get_height();
            const uint16_t* depthFrameData = (const uint16_t*)depthFrame.get_data();
            // NOTE(erd): this is fast enough in -O3 (1920x1080 -> ~15ms) but could probably be
            // better
            depthFrameScaled = make_unique<vector<uint16_t>>(depthWidth * depthHeight);
            for (int y = 0; y < depthHeight; y++) {
                for (int x = 0; x < depthWidth; x++) {
                    auto px = (y * depthWidth) + x;
                    uint16_t depthScaled = depthScaleMm * depthFrameData[px];
                    (*depthFrameScaled)[px] = depthScaled;
                }
            }
        }
        frameSet.mutex.lock();
        frameSet.colorFrame = frames.get_color_frame();
        frameSet.depthFrame = move(depthFrameScaled);
        frameSet.mutex.unlock();

        if (DEBUG) {
            auto stop = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
            cout << "[frameLoop] total:           " << duration.count() << "ms\n";
        }

        if (!readyOnce) {
            readyOnce = true;
            ready.set_value();
        }
    }
};

// gives the pixel to mm conversion for the depth sensor
float getDepthScale(rs2::device dev) {
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors()) {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {
            return dpt.get_depth_scale() * 1000.0;  // rs2 gives pix2meters
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

tuple<rs2::pipeline, RealSenseProperties> startPipeline(DeviceProperties& devProps) {
    rs2::context ctx;
    auto devices = ctx.query_devices();
    if (devices.size() == 0) {
        throw runtime_error("no device connected; please connect an Intel RealSense device");
    }
    rs2::device selected_device = devices.front();

    auto serial = selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    cout << "found device:\n";
    cout << "name:      " << selected_device.get_info(RS2_CAMERA_INFO_NAME) << "\n";
    cout << "serial:    " << serial << "\n";
    cout << "firmware:  " << selected_device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << "\n";
    cout << "port:      " << selected_device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) << "\n";
    cout << "usb type:  " << selected_device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) << "\n";

    float depthScaleMm = 0.0;
    if (!devProps.disableDepth) {
        depthScaleMm = getDepthScale(selected_device);
    }

    rs2::config cfg;
    cfg.enable_device(serial);

    if (!devProps.disableColor) {
        cfg.enable_stream(RS2_STREAM_COLOR, devProps.colorWidth, devProps.colorHeight,
                          RS2_FORMAT_RGB8);
    }

    if (!devProps.disableDepth) {
        cfg.enable_stream(RS2_STREAM_DEPTH, devProps.depthWidth, devProps.depthHeight,
                          RS2_FORMAT_Z16);
    }

    rs2::pipeline pipeline(ctx);
    pipeline.start(cfg);

    auto fillProps = [](auto intrinsics, string distortionModel) -> CameraProperties {
        CameraProperties camProps;
        camProps.width = intrinsics.width;
        camProps.height = intrinsics.height;
        camProps.fx = intrinsics.fx;
        camProps.fy = intrinsics.fy;
        camProps.ppx = intrinsics.ppx;
        camProps.ppy = intrinsics.ppy;
        camProps.distortionModel = distortionModel;
        for (int i = 0; i < 5; i++) {
            camProps.distortionParameters[i] = double(intrinsics.coeffs[i]);
        }
        return camProps;
    };

    RealSenseProperties props;
    props.depthScaleMm = depthScaleMm;
    if (!devProps.disableColor) {
        auto const stream = pipeline.get_active_profile()
                                .get_stream(RS2_STREAM_COLOR)
                                .as<rs2::video_stream_profile>();
        auto intrinsics = stream.get_intrinsics();
        props.color = fillProps(intrinsics, "brown_conrady");
    }
    if (!devProps.disableDepth) {
        auto const stream = pipeline.get_active_profile()
                                .get_stream(RS2_STREAM_DEPTH)
                                .as<rs2::video_stream_profile>();
        auto intrinsics = stream.get_intrinsics();
        props.depth = fillProps(intrinsics, "no_distortion");
        if (!devProps.disableColor) {
            props.depth.width = props.color.width;
            props.depth.height = props.color.height;
        }
    }

    cout << "pipeline started with:\n";
    cout << "color_enabled:  " << boolalpha << !devProps.disableColor << "\n";
    if (!devProps.disableColor) {
        cout << "color_width:    " << props.color.width << "\n";
        cout << "color_height:   " << props.color.height << "\n";
    }
    cout << "depth_enabled:  " << !devProps.disableDepth << endl;
    if (!devProps.disableDepth) {
        auto alignedText = "";
        if (!devProps.disableColor) {
            alignedText = " (aligned to color)";
        }
        cout << "depth_width:    " << props.depth.width << alignedText << "\n";
        cout << "depth_height:   " << props.depth.height << alignedText << endl;
    }

    return make_tuple(pipeline, props);
};

void on_device_reconnect(rs2::event_information& info, DeviceProperties& context,
                         rs2::pipeline pipeline) {
    if (info.was_added(info.get_new_devices().front())) {
        std::cout << "Device was reconnected, restarting pipeline" << std::endl;
        {
            std::lock_guard<std::mutex> lock(context.mutex);
            context.shouldRun = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // Find and start the first available device
        RealSenseProperties props;
        try {
            tie(pipeline, props) = startPipeline(context);
        } catch (const exception& e) {
            cout << "caught exception: \"" << e.what() << "\"" << endl;
            return;
        }
        // Start the camera thread
        promise<void> ready;
        thread cameraThread(frameLoop, pipeline, ref(latestFrames), ref(ready), ref(context),
                            props.depthScaleMm);
        cout << "waiting for camera frame loop thread to be ready..." << flush;
        ready.get_future().wait();
        cout << " ready!" << endl;
        cameraThread.detach();
    } else {
        std::cout << "Device disconnected, stopping frame pipeline" << std::endl;
        {
            std::lock_guard<std::mutex> lock(context.mutex);
            context.shouldRun = false;
        }
    }
};

int main(const int argc, const char* argv[]) {
    fpng::fpng_init();

    cout << "Intel RealSense gRPC server" << endl;
    if (argc == 2 && string("--help").compare(string(argv[1])) == 0) {
        cout << "usage: intelrealgrpcserver [port_number] [color_width] [color_height] "
                "[depth_width] [depth_height]"
                "[--disable-depth] [--disable-color]"
             << endl;
        return 0;
    }
    string port = "8085";
    int colorWidth = 0;
    int colorHeight = 0;
    int depthWidth = 0;
    int depthHeight = 0;
    if (argc > 1) {
        port = argv[1];
    }

    auto parseIntArg = [argc, argv](const int pos, const string& name) -> tuple<int, bool> {
        if (argc <= pos) {
            return {0, false};
        }
        char* endParse;
        auto parsed = strtol(argv[pos], &endParse, 10);
        if (argv[pos] == endParse) {
            cerr << "failed to parse " << name << " \"" << argv[pos] << "\"" << endl;
            return {0, true};
        }
        return {parsed, false};
    };

    bool err;
    tie(colorWidth, err) = parseIntArg(2, "color_width");
    if (err) {
        return 1;
    }
    tie(colorHeight, err) = parseIntArg(3, "color_height");
    if (err) {
        return 1;
    }
    tie(depthWidth, err) = parseIntArg(4, "depth_width");
    if (err) {
        return 1;
    }
    tie(depthHeight, err) = parseIntArg(5, "depth_height");
    if (err) {
        return 1;
    }

    if (colorWidth == 0 || colorHeight == 0) {
        cout << "note: will pick any suitable color_width and color_height" << endl;
    }
    if (depthWidth == 0 || depthHeight == 0) {
        cout << "note: will pick any suitable depth_width and depth_height" << endl;
    }

    bool disableDepth = false;
    bool disableColor = false;
    for (int i = 6; i < argc; i++) {
        auto argVal = string(argv[i]);
        if (string("--disable-depth").compare(argVal) == 0) {
            disableDepth = true;
        } else if (string("--disable-color").compare(argVal) == 0) {
            disableColor = true;
        } else if (string("--debug").compare(argVal) == 0) {
            DEBUG = true;
        }
    }

    if (disableColor && disableDepth) {
        cerr << "cannot disable both color and depth" << endl;
        return 1;
    }

    // DeviceProperties context also holds a bool that can stop the thread if device gets
    // disconnected
    DeviceProperties deviceProps(colorWidth, colorHeight, disableColor, depthWidth, depthHeight,
                                 disableDepth);

    // First start of Pipeline
    rs2::pipeline pipe;
    RealSenseProperties props;
    try {
        tie(pipe, props) = startPipeline(ref(deviceProps));
    } catch (const exception& e) {
        cout << "caught exception: \"" << e.what() << "\"" << endl;
        return 1;
    }
    // First start of camera thread
    promise<void> ready;
    thread cameraThread(frameLoop, pipe, ref(latestFrames), ref(ready), ref(deviceProps),
                        props.depthScaleMm);
    cout << "waiting for camera frame loop thread to be ready..." << flush;
    ready.get_future().wait();
    cout << " ready!" << endl;
    // start the callback function that will look for camera disconnects and reconnects.
    // on reconnects, it will close and restart the pipeline and thread.
    rs2::context ctx;
    ctx.set_devices_changed_callback(
        [&](rs2::event_information& info) { on_device_reconnect(info, deviceProps, pipe); });

    // Start the gRPC server
    RobotServiceImpl robotService;
    CameraServiceImpl cameraService(props, latestFrames, deviceProps.disableColor,
                                    deviceProps.disableDepth);
    const string address = "0.0.0.0:" + port;
    ServerBuilder builder;
    builder.AddListeningPort(address, grpc::InsecureServerCredentials());
    builder.RegisterService(&robotService);
    builder.RegisterService(&cameraService);
    unique_ptr<Server> server(builder.BuildAndStart());

    cout << "listening on " << address << endl;
    server->Wait();
    return 0;
}
