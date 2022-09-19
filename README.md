# Camera Servers

## Dependencies
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
* [gRPC] (https://github.com/viamrobotics/rdk/tree/main/grpc/cpp)
  * Needed for gRPC servers
* [openssl]
  * Needed for gRPC servers
## Installation Instructions
The simplest way to create the binaries are to compile using the Docker canon image for the desired architecture.
[Internal docs for docker canon.](https://viam.atlassian.net/wiki/spaces/ENG/pages/23527432/Canon+Docker+Images+and+Tools)

You can then export an AppImage of the binary using
```
cd packaging/appimages && appimage-builder --recipe mycameraserver-`uname -m`.yml
```

### macOS dependency install
Run `make setupmacos`

### Linux
**If on Raspberry Pi (Debian):** `sudo apt install xorg-dev`

### Installing `librealsense` from source
```bash
sudo apt install libglfw3-dev libusb-1.0-0-dev libgl1-mesa-dev libglu1-mesa-dev
git clone git@github.com:IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build
cmake ..
make -j 4
sudo make install
```
    
### Installing `libhttpserver` from source
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

### If none of that works, try this:
https://github.com/IntelRealSense/librealsense/blob/master/doc/libuvc_installation.md

Lastly, to use a local copy of RDK see https://github.com/viamrobotics/rdk/blob/main/grpc/cpp/README.md for setting up required grpc components and after doing the `make setup buf` in grpc/cpp inside RDK, set `RDK_SOURCE_DIR` appropriately and build.
* `RDK_SOURCE_DIR=~/source/rdk make cubeeyegrpc`
