UNAME := $(shell uname)
ENTRYCMD = usermod --uid $(shell id -u) testbot && groupmod --gid $(shell id -g) testbot && sudo -u testbot

ifeq ($(UNAME), Linux)
   special = -lpthread
endif

GREPPED = $(shell grep -sao jetson /proc/device-tree/compatible)
ifneq ("$(strip $(GREPPED))", "")
   $(info Nvidia Jetson Detected)
   SERVER_DEB_PLATFORM = jetson
else ifneq ("$(wildcard /etc/rpi-issue)","")
   $(info Raspberry Pi Detected)
   SERVER_DEB_PLATFORM = pi
else
   SERVER_DEB_PLATFORM = generic
endif
SERVER_DEB_VER = 0.1

LIB_FILES = cameraserver.cpp

# Enter your path to the rdk directory here
RDK_SOURCE_DIR = ../rdk
SRCDIR = $(RDK_SOURCE_DIR)/grpc/cpp/gen
IFLAGS = -I$(SRCDIR)
LDFLAGS = -L/usr/local/lib
GRPCFLAGS = `pkg-config --cflags grpc --libs protobuf grpc++`
OTHER = -pthread -Wl,-lgrpc++_reflection -Wl,-ldl
SOURCES = $(SRCDIR)/proto/api/v1/robot.grpc.pb.cc $(SRCDIR)/proto/api/v1/robot.pb.cc
SOURCES += $(SRCDIR)/proto/api/service/v1/metadata.grpc.pb.cc $(SRCDIR)/proto/api/service/v1/metadata.pb.cc
SOURCES += $(SRCDIR)/proto/api/common/v1/common.grpc.pb.cc $(SRCDIR)/proto/api/common/v1/common.pb.cc
SOURCES += $(SRCDIR)/proto/api/component/v1/camera.grpc.pb.cc $(SRCDIR)/proto/api/component/v1/camera.pb.cc
SOURCES += $(SRCDIR)/google/api/annotations.pb.cc $(SRCDIR)/google/api/httpbody.pb.cc
SOURCES += $(SRCDIR)/google/api/http.pb.cc

default: cubeeyeserver intelrealserver royaleserver

format: *.h *.cpp
	clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4}" *.cpp *.h

all: default opencv

clean:
	rm -rf cubeeyeserver intelrealserver royaleserver opencvserver

clean-all: clean
	rm -rf packaging/appimages/deploy packaging/appimages/appimage-builder-cache packaging/appimages/AppDir packaging/debian/work

setupmacos: macos.sh
	./macos.sh
	
setupgrpc:
	bash etc/setup.sh

cubeeyegrpc: cubeeyeGRPC.cpp $(LIB_FILES)
	g++ -g -std=c++17 cubeeyeGRPC.cpp $(LIB_FILES) `pkg-config --cflags --libs libhttpserver cubeeye` $(SOURCES) $(IFLAGS) $(LDFLAGS) $(GRPCFLAGS) $(OTHER) -o cubeeyegrpcserver

cubeeyeserver: cubeeyeserver.cpp $(LIB_FILES)
	g++ -g -std=c++17 cubeeyeserver.cpp $(LIB_FILES) `pkg-config --cflags --libs libhttpserver cubeeye` $(special) -o cubeeyeserver

intelrealserver: intelrealserver.cpp $(LIB_FILES)
	g++ -g -std=c++17 intelrealserver.cpp $(LIB_FILES) `pkg-config --cflags --libs realsense2 libhttpserver` $(special) -o intelrealserver

royaleserver: royaleserver.cpp $(LIB_FILES)
	g++ -g -std=c++17 royaleserver.cpp $(LIB_FILES) `pkg-config --cflags --libs libhttpserver royale` $(special) -o royaleserver

opencvserver: opencvserver.cpp $(LIB_FILES)
	g++ -g -std=c++17 opencvserver.cpp $(LIB_FILES) `pkg-config --cflags --libs opencv4 libhttpserver` $(special) -o opencvserver

deb: clean default
	rm -rf packaging/debian/work/ && mkdir packaging/debian/work/
	cp -r packaging/debian/viam-camera-servers-$(SERVER_DEB_VER)/ packaging/debian/work/viam-camera-servers-$(SERVER_DEB_PLATFORM)-$(SERVER_DEB_VER)/
	install -D cubeeyeserver packaging/debian/work/viam-camera-servers-$(SERVER_DEB_PLATFORM)-$(SERVER_DEB_VER)/usr/bin/cubeeyeserver
	install -D intelrealserver packaging/debian/work/viam-camera-servers-$(SERVER_DEB_PLATFORM)-$(SERVER_DEB_VER)/usr/bin/intelrealserver
	install -D royaleserver packaging/debian/work/viam-camera-servers-$(SERVER_DEB_PLATFORM)-$(SERVER_DEB_VER)/usr/bin/royaleserver
	cd packaging/debian/work/viam-camera-servers-$(SERVER_DEB_PLATFORM)-$(SERVER_DEB_VER)/ \
	&& sed -i "s/viam-camera-servers/viam-camera-servers-$(SERVER_DEB_PLATFORM)/g" debian/control debian/changelog \
	&& dch --force-distribution -D viam -v $(SERVER_DEB_VER)+`date -u '+%Y%m%d%H%M'` "Auto-build from commit `git log --pretty=format:'%h' -n 1`" \
	&& dpkg-buildpackage -us -uc -b \

appimages: clean default
	cd packaging/appimages && appimage-builder --recipe cubeeyeserver-`uname -m`.yml
	cd packaging/appimages && appimage-builder --recipe intelrealserver-`uname -m`.yml
	cd packaging/appimages && appimage-builder --recipe royaleserver-`uname -m`.yml
	mkdir -p packaging/appimages/deploy/
	mv packaging/appimages/*.AppImage* packaging/appimages/deploy/
	chmod 755 packaging/appimages/deploy/*.AppImage

# This sets up multi-arch emulation under linux. Run before using multi-arch targets.
docker-emulation:
	docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

appimages-multiarch: appimages-amd64 appimages-arm64

appimages-amd64:
	docker run --platform linux/amd64 -v`pwd`:/host --workdir /host --rm ghcr.io/viamrobotics/appimage:latest "$(ENTRYCMD) make appimages"

appimages-arm64:
	docker run --platform linux/arm64 -v`pwd`:/host --workdir /host --rm ghcr.io/viamrobotics/appimage:latest "$(ENTRYCMD) make appimages"

appimages-deploy:
	gsutil -m -h "Cache-Control: no-cache" cp packaging/appimages/deploy/* gs://packages.viam.com/apps/camera-servers/
