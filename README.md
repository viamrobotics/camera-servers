# Camera Servers

## Dependencies
* [libhttpserver](https://github.com/etr/libhttpserver)
  * Needed for all camera servers
* [librealsense](https://github.com/IntelRealSense/librealsense)
  * Needed only for Intel RealSense cameras
  * https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
* [libroyale](https://pmdtec.com/picofamily/software/)
  * Needed only for PMDTEC cameras
* [libCubeEye](http://cube-eye.co.kr/en/#/support/main.asp?sub=download)
  * Needed only for CubeEye cameras
* [libopencv](https://opencv.org/releases/)
  * In most apt repos `sudo apt install libopencv-dev`
  * Needed only for OpenCV server
* [gRPC] (https://github.com/viamrobotics/rdk/tree/main/grpc/cpp)
  * Needed for gRPC servers
* [openssl]
  * Needed for gRPC servers
## Installation Instructions
Run `make` after dependencies are installed. Generate a debian package for the default cameras with `make deb`

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

### CubeEye gRPC C++ Dependencies(Linux)
sudo apt-get install libssl-dev
* Run `make setupgrpc`
* copy over gen files from a working gRPC C++ setup
* Run `make cubeeyegrpc`
