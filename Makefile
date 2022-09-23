UNAME := $(shell uname)

ifeq ($(UNAME), Linux)
   special = -lpthread
endif

LIB_FILES = cameraserver.cpp

default: intelrealserver royaleserver 

format: *.h *.cpp
	clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4, ColumnLimit: 100}" *.cpp *.h

all: default opencv

clean:
	rm -rf intelrealserver royaleserver opencvserver

clean-all: clean
	git clean -fxd

setupmacos: macos.sh
	./macos.sh

intelrealserver: intelrealserver.cpp $(LIB_FILES)
	g++ -g -std=c++17 intelrealserver.cpp $(LIB_FILES) `pkg-config --cflags --libs realsense2 opencv4 libhttpserver` $(special) -o intelrealserver

royaleserver: royaleserver.cpp $(LIB_FILES)
	g++ -g -std=c++17 royaleserver.cpp $(LIB_FILES) `pkg-config --cflags --libs libhttpserver opencv4 royale` $(special) -o royaleserver

opencvserver: opencvserver.cpp $(LIB_FILES)
	g++ -g -std=c++17 opencvserver.cpp $(LIB_FILES) `pkg-config --cflags --libs opencv4 libhttpserver` $(special) -o opencvserver

appimages: default
	cd packaging/appimages && appimage-builder --recipe intelrealserver-`uname -m`.yml
	cd packaging/appimages && appimage-builder --recipe royaleserver-`uname -m`.yml
	mkdir -p packaging/appimages/deploy/
	mv packaging/appimages/*.AppImage* packaging/appimages/deploy/
	chmod 755 packaging/appimages/deploy/*.AppImage

appimages-deploy:
	gsutil -m -h "Cache-Control: no-cache" cp packaging/appimages/deploy/* gs://packages.viam.com/apps/camera-servers/

include *.make
