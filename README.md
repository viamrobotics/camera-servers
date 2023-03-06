# Camera Servers

## Quick Install 

For Linux Distros, the simplest way of getting the camera servers is by downloading the binaries from our package servers depending on your computer's architecture.

For example: 
```
sudo curl -o /usr/local/bin/intelrealgrpcserver http://packages.viam.com/apps/camera-servers/intelrealgrpcserver-latest-aarch64.AppImage

sudo chmod a+rx /usr/local/bin/intelrealgrpcserver
```

For MacOS (M1 macs), you can use our homebrew formula from the Viam tap:
```
brew install viamrobotics/brews/intel-real-grpc-server
```

### URLs

- Intel Realsense gRPC server x86\_64
  - http://packages.viam.com/apps/camera-servers/intelrealgrpcserver-latest-x86_64.AppImage
- Intel Realsense gRPC server aarch64
  - http://packages.viam.com/apps/camera-servers/intelrealgrpcserver-latest-aarch64.AppImage

### Troubleshooting

If you get an error like "failed to set power state", or "Permission denied", you may need to install the udev rules for when the USB plugs in. 

```
$ wget https://raw.githubusercontent.com/IntelRealSense/librealsense/7a7c2bcfbc03d45154ad63fa76b221b2bb9d228f/config/99-realsense-libusb.rules
$ sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/ 
$ sudo udevadm control --reload-rules 
$ sudo udevadm trigger
```

You can also look at the official RealSense troubleshooting guide [here](https://github.com/IntelRealSense/librealsense/wiki/Troubleshooting-Q%26A#q-i-ran-the-udev-rules-script-but-linux-still-get-permission-denied).

The intelrealgrpcserver takes advantage of faster USB ports. Use the (blue) USB 3.0 port on the Raspberry Pi for faster streaming and access to more resolution options.
 
## Building gRPC server from Source

### Dependencies
* [librealsense](https://github.com/IntelRealSense/librealsense) (tested on v2.53.1)
  * https://github.com/IntelRealSense/librealsense#download-and-install
  * macOS Troubleshooting
    * We suggest using the following cmake replacement command `cmake .. -DBUILD_EXAMPLES=false -DBUILD_WITH_OPENMP=false -DHWM_OVER_XU=false -DBUILD_TOOLS=false`.
    * If you get a `-latomic` linking error, you can try removing `-latomic` references from `CMake/unix_config.cmake`
* [libjpegturbo](https://github.com/libjpeg-turbo/libjpeg-turbo)
  * Needed for JPEG compression.
  * can try `make setup`.
* [The Viam API](https://github.com/viamrobotics/api)
* [openssl](https://www.openssl.org/)
* [protobuf](https://developers.google.com/protocol-buffers/docs/downloads)

### Dependency install
Run `make setup`

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
    
#### If none of that works, try this:

https://github.com/IntelRealSense/librealsense/blob/master/doc/libuvc_installation.md

### Build the binary

run `make intelrealgrpcserver`

You can then export an AppImage of the binary using
```
cd packaging/appimages && appimage-builder --recipe intelrealgrpcserver-`uname -m`.yml
```

## Adding a gRPC camera as a remote to your robot

### Start the server on robot start up

On app.viam.com, go to Config -> Processes, and put in the following:

**Executable**: /usr/local/bin/intelrealgrpcserver
**Working Directory**: Leave it blank
**Arguments**: All arguments are optional, and positional

* 1st argument: port number, `int` (default is 8085)
* 2nd: color width, `int`
* 3rd: color height, `int`
* 4th: depth width, `int`
* 5th: depth height, `int`

If you just want the defaults, leave “args” field blank. If you dont put in anything, this will set up the gRPC server running on port 8085 of your pi with the default resolution.

The JSON of the process will look like the following:
```
[ 
  { 
    "id": "intel", 
    "log": true, 
    "name": "/usr/local/bin/intelrealgrpcserver",
    "args": [port_number, color_width, color_height, depth_width, depth_height] // Put the actual numbers you want here. If the realsense does not support the requested height and width it will error and fail to start
 
  } 
]
```
Additionally, after `depth_height` you can use `--disable-depth` `--disable-color` to disable streaming of depth or color.

### Add the gRPC server as a remote

Then go to Config -> Remotes, and add the following 

**Address**: 127.0.0.1:8085 // if you changed the port number in the arguments, it won't be 8085.
**TLS**: Off

The JSON of the process will look like the following:
```
[
 {
   "name": "intel",
   "address": "127.0.0.1:8085",
   "insecure": true
 }
]
```

This will add the two cameras to your robot. They will have the names `intel:color` and `intel:depth`.

### Create camera to display point clouds

Go to Config -> Components, and add the following camera model, `join_color_depth`. 
```
 {
        "output_image_type": "color",
    // you can get intrinsics by calling GetProperties on the intel gRPC camera server, too
        "intrinsic_parameters": {  // intrinsic parameters depend on the resolution of the camera
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

## Query a VIAM gRPC camera server with grpcURL

If you have a gRPC camera server running and would like to directly query the camera, here are the instructions of how to do so.

Make sure you have `buf` installed.

### Know what address and port the server is running on

e.g. `127.0.0.1:8085`

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

### Using gRPCurl to query the camera

You can use grpcurl like curl, but for gRPC servers, rather than HTTP servers

The available VIAM camera methods are 
- viam.robot.v1.RobotService/ResourceNames
- viam.component.camera.v1.CameraService/GetImage
- viam.component.camera.v1.CameraService/GetProperties

From within the API directory, you can run commands like

```
$ grpcurl -plaintext -protoset <(buf build -o - buf.build/viamrobotics/api) 127.0.0.1:8085 viam.robot.v1.RobotService/ResourceNames


$ grpcurl -plaintext -d '{ "name": "color" }' -protoset <(buf build -o - buf.build/viamrobotics/api) 127.0.0.1:8085 viam.component.camera.v1.CameraService/GetProperties


$ grpcurl -max-msg-sz 10485760 -plaintext -d '{ "name": "color", "mimeType": "image/jpeg" }' -protoset <(buf build -o - buf.build/viamrobotics/api) 127.0.0.1:8085 viam.component.camera.v1.CameraService/GetImage
```

### Handling the responses from the servers

You will get JSON objects as responses from the servers.  Image requests will return the bytes encoded in base64. 

You can use the program [jq](https://stedolan.github.io/jq/) to extract the relevant bytes and info you need from the response fields, and then decode them as needed. You can extract and  see the image by doing something like the following:

```
$ cd api

$ grpcurl -max-msg-sz 10485760 -plaintext -d '{ "name": "color", "mimeType": "image/jpeg" }' -protoset <(buf build -o - buf.build/viamrobotics/api) 127.0.0.1:8085 viam.component.camera.v1.CameraService/GetImage | jq -r ".image" | base64 --decode >> output_image.jpeg

$ open output_image.jpeg
```


