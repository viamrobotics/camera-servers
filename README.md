# Camera Servers

## Quick Install 

For Linux Distros, the simplest way of getting the camera servers is by downloading the binaries from our package servers depending on your computer's architecture.

For example: 
```
sudo curl -o /usr/local/bin/intelrealgrpcserver http://packages.viam.com/apps/camera-servers/intelrealgrpcserver-latest-aarch64.AppImage

sudo chmod a+rx /usr/local/bin/intelrealgrpcserver
```
### URLs

- Intel Realsense HTTP server x86\_64
  - http://packages.viam.com/apps/camera-servers/intelrealserver-latest-x86_64.AppImage
- Intel Realsense HTTP server aarch64
  - http://packages.viam.com/apps/camera-servers/intelrealserver-latest-aarch64.AppImage
- Intel Realsense GRPC server x86\_64
  - http://packages.viam.com/apps/camera-servers/intelrealgrpcserver-latest-x86_64.AppImage
- Intel Realsense GRPC server aarch64
  - http://packages.viam.com/apps/camera-servers/intelrealgrpcserver-latest-aarch64.AppImage
- PMDTEC Pico Flexx HTTP server x86\_64
  - http://packages.viam.com/apps/camera-servers/royaleserver-latest-x86_64.AppImage
- PMDTEC Pico Flexx HTTP server aarch64
  - http://packages.viam.com/apps/camera-servers/royaleserver-latest-aarch64.AppImage


## Building from Source

### Dependencies
* [libhttpserver](https://github.com/etr/libhttpserver)
  * Needed for all camera servers
* [librealsense](https://github.com/IntelRealSense/librealsense)
  * Needed only for Intel RealSense cameras
  * https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
* [libroyale](https://pmdtec.com/picofamily/software/)
  * Needed only for PMDTEC cameras
* [libopencv](https://opencv.org/releases/)
  * In most apt repos `sudo apt install libopencv-dev`
  * Needed for all servers in order to create depth png endpoint.
* [The Viam API](https://github.com/viamrobotics/api)
  * Needed for gRPC servers
* [openssl](https://www.openssl.org/)
  * Needed for gRPC servers
* [protobuf](https://developers.google.com/protocol-buffers/docs/downloads)
  * Needed for gRPC servers
  * can try `make bufsetup` to install dependencies.

### macOS dependency install
Run `make setupmacos`

### Linux dependency install
**If on Raspberry Pi (Debian):** `sudo apt install xorg-dev`

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

run either `make intelrealserver`, `make intelrealgrpcserver`, or `make royaleserver` depending on which binary you would like to make.

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

You can use the program jq to extract the relevant bytes and info you need from the response fields, and then decode them as needed. You can extract and  see the image by doing something like the following:

```
$ grpcurl -max-msg-sz 10485760 -plaintext -d '{ "name": "MyCamera", "mimeType": "image/jpeg" }' -protoset <(buf build -o -) my-server-url.local:8085 viam.component.camera.v1.CameraService/GetImage | jq -r ".image" > raw_image.txt

$ python decode.py raw_image.txt output.jpeg

$ open output.jpeg
```

The code for decoding base64 using python:
```
# decode.py
import base64, sys
infilename = sys.argv[1]
outfilename = sys.argv[2]
if infilename == "" or outfilename == "":
    print("in file or out file cannot be empty")
    exit(1)
with open(infilename, 'rb') as file:
    data = file.read()
    with open(outfilename, 'wb') as outfile:
        outfile.write(base64.decodebytes(data))
```

