// cameraserver.h

#pragma once

#include <atomic>
#include <httpserver.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>

class Vertex {
   public:
    Vertex(){};
    Vertex(float _x, float _y, float _z) : x(_x), y(_y), z(_z){};

    float x;
    float y;
    float z;
};

class CameraOutput {
   public:
    CameraOutput() {}

    void add_depth(int bytesPerPixel, float units, int width, int height,
                   const char* data);
    int width;
    int height;
    int depth_width;
    int depth_height;
    std::string ppmdata;
    std::string depth;
    cv::Mat pic_cv;
    cv::Mat depth_cv;
    std::vector<Vertex> points;
};

class CameraState {
   public:
    static CameraState* get();

    void setLastRequest(time_t lastRequest);
    time_t getLastRequest();
    void addCamera();
    void setCameraOutput(size_t i, std::shared_ptr<CameraOutput> output);
    std::shared_ptr<CameraOutput> getCameraOutput(size_t i);
    size_t getNumCameras();

    std::atomic<bool> ready;

   private:
    CameraState() : ready(0), _lastRequest(0) {}

    std::mutex _mutex;
    std::vector<std::shared_ptr<CameraOutput>> _cameras;
    volatile time_t _lastRequest;
};

class camera_resource : public httpserver::http_resource {
   public:
    camera_resource(CameraState* cam) : _cams(cam) {}

    int getCameraNumber(const httpserver::http_request& r) {
        auto s = r.get_arg("num");
        if (s == "") {
            return 0;
        }
        return std::stoi(s);
    }

    const std::shared_ptr<httpserver::http_response> render(
        const httpserver::http_request& r) {
        _cams->setLastRequest(time(0));

        if (!_cams->ready) {
            return std::shared_ptr<httpserver::http_response>(
                new httpserver::string_response("not ready\n"));
        }

        int camNumera = getCameraNumber(r);
        if (camNumera >= _cams->getNumCameras()) {
            return std::shared_ptr<httpserver::http_response>(
                new httpserver::string_response("invalid camera\n"));
        }

        std::shared_ptr<CameraOutput> mine(_cams->getCameraOutput(camNumera));
        return myRender(mine.get());
    }

    virtual const std::shared_ptr<httpserver::http_response> myRender(
        CameraOutput* co) = 0;

   private:
    CameraState* _cams;
};

void installWebHandlers(httpserver::webserver* ws);

std::string my_write_ppm(const char* pixels, int x, int y, int bytes_per_pixel);
