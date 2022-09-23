// cameraserver.cpp

#include "cameraserver.h"

#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <sstream>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

void CameraOutput::add_depth(int bytesPerPixel, float units, int width,
                             int height, const char* data) {
    std::stringbuf buffer;
    std::ostream os(&buffer);

    os << "VERSIONX\n";
    os << bytesPerPixel << "\n";
    os << units << "\n";
    os << width << "\n";
    os << height << "\n";

    buffer.sputn(data, width * height * bytesPerPixel);
    depth = buffer.str();
}

CameraState* myCameraState = 0;

CameraState* CameraState::get() {
    if (!myCameraState) {
        myCameraState = new CameraState();
    }
    return myCameraState;
}

void CameraState::setLastRequest(time_t lastRequest) {
    std::lock_guard<std::mutex> lock(_mutex);
    _lastRequest = lastRequest;
}

time_t CameraState::getLastRequest() {
    std::lock_guard<std::mutex> lock(_mutex);
    return _lastRequest;
}

void CameraState::addCamera() {
    std::lock_guard<std::mutex> lock(_mutex);
    _cameras.push_back(0);
}

void CameraState::setCameraOutput(size_t i,
                                  std::shared_ptr<CameraOutput> output) {
    std::lock_guard<std::mutex> lock(_mutex);
    _cameras[i] = output;
}

std::shared_ptr<CameraOutput> CameraState::getCameraOutput(size_t i) {
    std::lock_guard<std::mutex> lock(_mutex);
    return _cameras[i];
}

size_t CameraState::getNumCameras() {
    std::lock_guard<std::mutex> lock(_mutex);
    return _cameras.size();
}

using namespace httpserver;

class hello_world_resource : public http_resource {
   public:
    hello_world_resource(CameraState* cam) : _cams(cam) {}

    const std::shared_ptr<http_response> render(const http_request&) {
        std::stringbuf buffer;
        std::ostream os(&buffer);

        os << "<html>";
        os << "<meta http-equiv=\"refresh\" content=\"1\" />";
        os << "<body>";
        for (int i = 0; i < _cams->getNumCameras(); i++) {
            os << "<img width=600 src='/pic.png?num=" << i << "'/>";
        }
        os << "</body></html>";

        return std::shared_ptr<http_response>(
            new string_response(buffer.str(), 200, "text/html"));
    }

   private:
    CameraState* _cams;
};

class picture_resource : public camera_resource {
   public:
    picture_resource(CameraState* cam) : camera_resource(cam) {}

    const std::shared_ptr<http_response> myRender(CameraOutput* mine) {
        return std::shared_ptr<http_response>(
            new string_response(mine->ppmdata, 200, "image/ppm"));
    }
};

class picture_resource_png : public camera_resource {
   public:
    picture_resource_png(CameraState* cam) : camera_resource(cam) {}

    const std::shared_ptr<http_response> myRender(CameraOutput* mine) {
        std::string s;

        if (!mine->pic_cv.empty()) {
            // Use imencode if possible, since it's faster.
            std::vector<uchar> chbuf;
            chbuf.resize(512 * 1024);
            cv::imencode(".png", mine->pic_cv, chbuf);
            s = std::string(chbuf.begin(), chbuf.end());
        } else {
            const char* raw_data = mine->ppmdata.c_str();
            int len = mine->width * mine->height * 3;
            raw_data = raw_data + (mine->ppmdata.size() - len);

            int pngLen;
            auto out = stbi_write_png_to_mem((const unsigned char*)raw_data,
                                             mine->width * 3, mine->width,
                                             mine->height, 3, &pngLen);
            s = std::string((char*)out, pngLen);
            STBIW_FREE(out);
        }

        return std::shared_ptr<http_response>(
            new string_response(s, 200, "image/png"));
    }
};

class depth_resource_png : public camera_resource {
   public:
    depth_resource_png(CameraState* cam) : camera_resource(cam) {}

    const std::shared_ptr<http_response> myRender(CameraOutput* mine) {
        std::vector<uchar> chbuf;
        chbuf.resize(
            512 *
            1024);  // just needs to be big enough. prob should be smarter.
        cv::imencode(".png", mine->depth_cv, chbuf);
        std::string s(chbuf.begin(), chbuf.end());

        return std::shared_ptr<http_response>(
            new string_response(s, 200, "image/png"));
    }
};

const int maxJpegSize = 512 * 1024;
struct jpeg_out {
    char buf[maxJpegSize];
    int size;
};

void my_jpg_write(void* context, void* data, int size) {
    jpeg_out* out = (jpeg_out*)context;
    if (size + out->size > maxJpegSize) {
        std::cerr << "size too big" << std::endl;
        return;
    }
    memcpy(out->buf + out->size, data, size);
    out->size += size;
}

class picture_resource_jpg : public camera_resource {
   public:
    picture_resource_jpg(CameraState* cam) : camera_resource(cam) {}

    const std::shared_ptr<http_response> myRender(CameraOutput* mine) {
        const char* raw_data = mine->ppmdata.c_str();
        int len = mine->width * mine->height * 3;
        raw_data = raw_data + (mine->ppmdata.size() - len);

        jpeg_out out;
        out.size = 0;
        auto rv = stbi_write_jpg_to_func(my_jpg_write, &out, mine->width,
                                         mine->height, 3, raw_data, 20);
        std::string s(out.buf, out.size);
        return std::shared_ptr<http_response>(
            new string_response(s, 200, "image/jpg"));
    }
};

class depth_resource : public camera_resource {
   public:
    depth_resource(CameraState* cam) : camera_resource(cam) {}
    const std::shared_ptr<http_response> myRender(CameraOutput* mine) {
        return std::shared_ptr<http_response>(
            new string_response(mine->depth, 200, "binary"));
    }
};

class combined_resource : public camera_resource {
   public:
    combined_resource(CameraState* cam) : camera_resource(cam) {}
    const std::shared_ptr<http_response> myRender(CameraOutput* mine) {
        std::string both = mine->depth + mine->ppmdata;

        return std::shared_ptr<http_response>(
            new string_response(both, 200, "binary"));
    }
};

void installWebHandlers(httpserver::webserver* ws) {
    auto x = CameraState::get();

    // TODO(erh): these all leak

    ws->register_resource("/", new hello_world_resource(x));
    ws->register_resource("/pic.ppm", new picture_resource(x));
    ws->register_resource("/pic.png", new picture_resource_png(x));
    ws->register_resource("/pic.jpg", new picture_resource_jpg(x));
    ws->register_resource("/depth.png", new depth_resource_png(x));
    ws->register_resource("/depth.dat", new depth_resource(x));
    ws->register_resource("/both", new combined_resource(x));
}

std::string my_write_ppm(const char* pixels, int x, int y,
                         int bytes_per_pixel) {
    std::stringbuf buffer;
    std::ostream os(&buffer);

    os << "P6\n" << x << " " << y << "\n255\n";
    buffer.sputn((const char*)pixels, x * y * bytes_per_pixel);

    return buffer.str();
}
