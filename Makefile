UNAME := $(shell uname)

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

format: *.h *.cpp
	clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4}" *.cpp *.h


default: cubeeyeserver intelrealserver royaleserver

all: default opencv

setupmacos: macos.sh
	./macos.sh

cubeeyeserver: cubeeyeserver.cpp $(LIB_FILES)
	g++ -g -std=c++11 cubeeyeserver.cpp $(LIB_FILES) `pkg-config --cflags --libs libhttpserver cubeeye` $(special) -o cubeeyeserver

intelrealserver: intelrealserver.cpp $(LIB_FILES)
	g++ -g -std=c++17 intelrealserver.cpp $(LIB_FILES) `pkg-config --cflags --libs realsense2 libhttpserver` $(special) -o intelrealserver

royaleserver: royaleserver.cpp $(LIB_FILES)
	g++ -g -std=c++17 royaleserver.cpp $(LIB_FILES) `pkg-config --cflags --libs libhttpserver royale` $(special) -o royaleserver

opencvserver: opencvserver.cpp $(LIB_FILES)
	g++ -g -std=c++17 opencvserver.cpp $(LIB_FILES) `pkg-config --cflags --libs opencv4 libhttpserver` $(special) -o opencvserver

deb: default
	rm -rf packaging/work/ && mkdir packaging/work/
	cp -r packaging/viam-camera-servers-$(SERVER_DEB_VER)/ packaging/work/viam-camera-servers-$(SERVER_DEB_PLATFORM)-$(SERVER_DEB_VER)/
	install -D cubeeyeserver packaging/work/viam-camera-servers-$(SERVER_DEB_PLATFORM)-$(SERVER_DEB_VER)/usr/bin/cubeeyeserver
	install -D intelrealserver packaging/work/viam-camera-servers-$(SERVER_DEB_PLATFORM)-$(SERVER_DEB_VER)/usr/bin/intelrealserver
	install -D royaleserver packaging/work/viam-camera-servers-$(SERVER_DEB_PLATFORM)-$(SERVER_DEB_VER)/usr/bin/royaleserver
	cd packaging/work/viam-camera-servers-$(SERVER_DEB_PLATFORM)-$(SERVER_DEB_VER)/ \
	&& sed -i "s/viam-camera-servers/viam-camera-servers-$(SERVER_DEB_PLATFORM)/g" debian/control debian/changelog \
	&& dch --force-distribution -D viam -v $(SERVER_DEB_VER)+`date -u '+%Y%m%d%H%M'` "Auto-build from commit `git log --pretty=format:'%h' -n 1`" \
	&& dpkg-buildpackage -us -uc -b \

clean:
	rm -f cubeeyeserver intelrealserver royaleserver opencvserver
