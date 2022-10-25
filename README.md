# Camera Servers

## Quick Install 

For Linux Distros, the simplest way of getting the camera servers is by downloading the binaries from our package servers depending on your computer's architecture.

For example: 
```
sudo curl -o /usr/local/bin/intelrealgrpcserver http://packages.viam.com/apps/camera-servers/intelrealgrpcserver-latest-aarch64.AppImage

sudo chmod a+rx /usr/local/bin/intelrealgrpcserver
```

For MacOS, you can use our homebrew formula from the Viam tap:
```
brew install viamrobotics/brews/intel-real-grpc-server
```
if you would like to use brew with Linux, the formula for openCV on Linux might not work out of the box, and might require an installation from source
```
brew install -s opencv
```

### URLs

- Intel Realsense GRPC server x86\_64
  - http://packages.viam.com/apps/camera-servers/intelrealgrpcserver-latest-x86_64.AppImage
- Intel Realsense GRPC server aarch64
  - http://packages.viam.com/apps/camera-servers/intelrealgrpcserver-latest-aarch64.AppImage

## Building GRPC server from Source

### Dependencies
* [librealsense](https://github.com/IntelRealSense/librealsense)
  * Needed only for Intel RealSense cameras
  * [Installation for Linux](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
* [libopencv](https://opencv.org/releases/)
  * In most apt repos `sudo apt install libopencv-dev`
  * Needed for all servers in order to create depth png endpoint.
* [libhttpserver](https://github.com/etr/libhttpserver)
  * Needed for HTTP camera servers
* [The Viam API](https://github.com/viamrobotics/api)
  * Needed for gRPC camera servers
* [openssl](https://www.openssl.org/)
  * Needed for gRPC camera servers
* [protobuf](https://developers.google.com/protocol-buffers/docs/downloads)
  * Needed for gRPC camera servers
  * can try `make bufsetup` to install dependencies.

### macOS dependency install
Run `make setupmacos`

### Linux dependency install
**If on Raspberry Pi (Debian):** 

```
sudo apt install xorg-dev
sudo apt install libopencv-dev
```

#### Installing `librealsense` from source
```bash
sudo apt install libglfw3-dev libusb-1.0-0-dev libgl1-mesa-dev libglu1-mesa-dev
git clone git@github.com:IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build
cmake ..
make -j 4
sudo make install
```
    
#### Installing `libhttpserver` from source
```bash
sudo apt install libmicrohttpd-dev libtool
git clone git@github.com:etr/libhttpserver.git
cd libhttpserver
./bootstrap
mkdir build && cd build
../configure
make -j 4
sudo make install
```

#### If none of that works, try this:

https://github.com/IntelRealSense/librealsense/blob/master/doc/libuvc_installation.md

### Build the binary

run `make intelrealgrpcserver`

You can then export an AppImage of the binary using
```
cd packaging/appimages && appimage-builder --recipe mycameraserver-`uname -m`.yml
```
## Query a VIAM GRPC camera server

If you have a GRPC camera server running and would like to directly query the camera, here are the instructions of how to do so.

Make sure you have protobuf and then do `make buf`. 

### Know what address and port the server is running on

e.g. `my-server-url.local:8085`

and make sure the server is running

### Install [grpcurl](https://github.com/fullstorydev/grpcurl)
on macOS: brew install grpcurl

On Linux: you can install the releases from github: https://github.com/fullstorydev/grpcurl/releases

### Install the [Viam API](https://github.com/viamrobotics/api)
You need the viam API in order for grpcurl to know what methods are available.

```
git clone git@github.com:viamrobotics/api.git
cd api
```

### Using GRPCurl to query the camera

You can use grpcurl like curl, but for GRPC servers, rather than HTTP servers

The available VIAM camera methods are 
- viam.robot.v1.RobotService/ResourceNames
- viam.component.camera.v1.CameraService/GetPointCloud
- viam.component.camera.v1.CameraService/GetImage
- viam.component.camera.v1.CameraService/GetProperties

From within the API directory, you can run commands like

```
$ grpcurl -plaintext -protoset <(buf build -o -) my-server-url.local:8085 viam.robot.v1.RobotService/ResourceNames


$ grpcurl -plaintext -d '{ "name": "MyCamera" }' -protoset <(buf build -o -) my-server-url.local:8085 viam.component.camera.v1.CameraService/GetProperties


$ grpcurl -max-msg-sz 10485760 -plaintext -d '{ "name": "MyCamera", "mimeType": "image/png" }' -protoset <(buf build -o -) my-server-url.local:8085 viam.component.camera.v1.CameraService/GetImage


$ grpcurl -max-msg-sz 20485760 -plaintext -d '{ "name": "MyCamera" }' -protoset <(buf build -o -) my-server-url.local:8085 viam.component.camera.v1.CameraService/GetPointCloud
```

### Handling the responses from the servers

You will get JSON objects as responses from the servers.  Image and PointClouds requests will return the bytes encoded in base64. 

You can use the program [jq](https://stedolan.github.io/jq/) to extract the relevant bytes and info you need from the response fields, and then decode them as needed. You can extract and  see the image by doing something like the following:

```
$ cd api

$ grpcurl -max-msg-sz 10485760 -plaintext -d '{ "name": "color", "mimeType": "image/jpeg" }' -protoset <(buf build -o -) my-server-url.local:8085 viam.component.camera.v1.CameraService/GetImage | jq -r ".image" | base64 --decode >> output_image.jpeg

$ open output_image.jpeg
```

## Adding a GRPC camera as a remote to your robot

### Start the server on robot start up

On app.viam.com, go to Config -> Processes, and put in the following:
```
[ 
  { 
    "id": "intel", 
    "log": true, 
    "name": "/usr/local/bin/intelgrpcserver",
    "args": [port_number, image_width, image_height] // Put the actual numbers you want here. If the realsense does not support the request height and width it will error and fail to start
 
  } 
]
```
If you just want the defaults, dont include the “args” field. If you dont put in anything, this will set up the GRPC server running on port 8085 of your pi with the default resolution.

### Add the GRPC server as a remote

Then go to Config -> Remote, and add the following 
```
[
 {
   "name": "intel",
   "address": "ip-address-of-your-pi:8085", // or whatever port number you set
   "insecure": true
 }
]
```

This will add the two cameras to your robot. They will have the names `intel:color` and `intel:depth`.

### Create camera to display point clouds

Go to Config -> Components, and add the following camera model, `align_color_depth`. 
```
 {
        "stream": "color",
        "width_px": 1280, // whatever the actual height and width of your images are
       "height_px": 720,
    // you can get intrinsics by calling GetProperties on the intel GRPC camera server, too
        "intrinsic_parameters": { 
            "height_px": 720,
            "width_px": 1280,
            "ppx": 648.1280,
            "ppy": 367.736,
            "fx": 900.538,
            "fy": 900.818
        },
        "color_camera_name": "intel:color",
        "depth_camera_name": "intel:depth"
      }
}
"depends_on": [
   "intel:color",
   "intel:depth"
]
```
 
Now in the *Control tab*, you can see both the individual 2D camera streams, as well as the pointcloud camera of the combined color and depth image that you created with `align_color_depth`.

