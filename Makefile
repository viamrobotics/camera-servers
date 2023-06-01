BIN_OUTPUT_PATH = bin/$(shell uname -s)-$(shell uname -m)

TOOL_BIN = bin/tools/$(shell uname -s)-$(shell uname -m)

PATH_WITH_TOOLS="`pwd`/$(TOOL_BIN):${PATH}"

UNAME := $(shell uname)

ifeq ($(UNAME), Darwin)
ifneq ($(shell which brew), )
   PKG_CONFIG_PATH_EXTRA=$(PKG_CONFIG_PATH):/usr/local/lib/pkgconfig:$(shell find $(shell which brew > /dev/null && brew --prefix) -name openssl.pc | head -n1 | xargs dirname)
endif
endif

LIB_FLAGS = $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH):$(PKG_CONFIG_PATH_EXTRA) pkg-config --cflags grpc++ realsense2 --libs protobuf grpc++ libturbojpeg realsense2)
GCC_FLAGS = -pthread -Wl,-ldl

ifeq ($(UNAME), Darwin)
	# There's a strange bug in the realsense pc file that adds this when it's not necessary at all
	# when brew is in use. This may break something in the future.
   LIB_FLAGS := $(LIB_FLAGS:-L/usr/local/lib/x86_64-linux-gnu=)
endif

GRPC_DIR = ./

ifdef API_SOURCE_DIR
GRPC_DIR=$(API_SOURCE_DIR)/grpc/cpp/gen
else
ifneq ($(wildcard ./grpc/cpp/gen),)
GRPC_DIR=./grpc/cpp/gen
else
GRPC_DIR=./grpc/cpp/gen
endif
endif

ifeq ($(shell arch), x86_64)
   GCC_FLAGS += -mpclmul -msse2 -msse4.2
endif

GEN_SOURCES = $(GRPC_DIR)/robot/v1/robot.grpc.pb.cc $(GRPC_DIR)/robot/v1/robot.pb.cc
GEN_SOURCES += $(GRPC_DIR)/common/v1/common.grpc.pb.cc $(GRPC_DIR)/common/v1/common.pb.cc
GEN_SOURCES += $(GRPC_DIR)/component/camera/v1/camera.grpc.pb.cc $(GRPC_DIR)/component/camera/v1/camera.pb.cc
GEN_SOURCES += $(GRPC_DIR)/google/api/annotations.pb.cc $(GRPC_DIR)/google/api/httpbody.pb.cc
GEN_SOURCES += $(GRPC_DIR)/google/api/http.pb.cc
THIRD_PARTY_SOURCES = third_party/fpng.cpp third_party/lodepng.cpp

default: intelrealgrpcserver-release-opt

format: *.cpp
	clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4, ColumnLimit: 100}" *.cpp

all: default

clean:
	rm -rf intelrealgrpcserver

clean-all: clean
	git clean -fxd

setup:
	sudo apt install -y libturbojpeg-dev protobuf-compiler-grpc libgrpc-dev libgrpc++-dev || brew install jpeg-turbo grpc openssl --quiet

$(GEN_SOURCES): $(TOOL_BIN)/buf $(TOOL_BIN)/protoc-gen-grpc-cpp
	PATH=$(PATH_WITH_TOOLS) buf generate --template ./grpc/buf.gen.yaml buf.build/viamrobotics/api
	PATH=$(PATH_WITH_TOOLS) buf generate --template ./grpc/buf.google.gen.yaml buf.build/googleapis/googleapis

$(TOOL_BIN)/buf:
	mkdir -p $(TOOL_BIN)
	GOBIN=`pwd`/$(TOOL_BIN) go install github.com/bufbuild/buf/cmd/buf@v1.13.1

$(TOOL_BIN)/protoc-gen-grpc-cpp:
	ln -sf `which grpc_cpp_plugin` $(TOOL_BIN)/protoc-gen-grpc-cpp

SERVER_SOURCES = $(THIRD_PARTY_SOURCES) $(GEN_SOURCES) intel_realsense_grpc.cpp
CPP_COMPILER = g++
CPP_FLAGS = -std=c++17 -I$(GRPC_DIR) $(LIB_FLAGS) $(GCC_FLAGS)

intelrealgrpcserver: $(SERVER_SOURCES)
	$(CPP_COMPILER) -o intelrealgrpcserver $(SERVER_SOURCES) $(CPP_FLAGS) $(CPP_FLAGS_EXTRA) 

intelrealgrpcserver-debug: CPP_FLAGS_EXTRA = -pg
intelrealgrpcserver-debug: intelrealgrpcserver

intelrealgrpcserver-release: intelrealgrpcserver

intelrealgrpcserver-release-opt: CPP_FLAGS_EXTRA = -O3
intelrealgrpcserver-release-opt: intelrealgrpcserver

appimages: clean default
	cd packaging/appimages && appimage-builder --recipe intelrealgrpcserver-`uname -m`.yml
	mkdir -p packaging/appimages/deploy/
	mv packaging/appimages/*.AppImage* packaging/appimages/deploy/
	chmod 755 packaging/appimages/deploy/*.AppImage

appimages-deploy:
	gsutil -m -h "Cache-Control: no-cache" cp packaging/appimages/deploy/* gs://packages.viam.com/apps/camera-servers/
