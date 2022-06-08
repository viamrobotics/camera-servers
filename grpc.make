GRPCDIR = ./

ifdef RDK_SOURCE_DIR
GRPCDIR=$(RDK_SOURCE_DIR)/grpc/cpp/gen
else
ifneq ($(wildcard ./grpc/cpp/gen),)
GRPCDIR=./grpc/cpp/gen
else
GRPCDIR=./rdk-minimal/grpc/cpp/gen
endif
endif

GRPCFLAGS = `pkg-config --cflags grpc --libs protobuf grpc++` -pthread -Wl,-lgrpc++_reflection -Wl,-ldl

SOURCES = $(GRPCDIR)/proto/api/robot/v1/robot.grpc.pb.cc $(GRPCDIR)/proto/api/robot/v1/robot.pb.cc
SOURCES += $(GRPCDIR)/proto/api/common/v1/common.grpc.pb.cc $(GRPCDIR)/proto/api/common/v1/common.pb.cc
SOURCES += $(GRPCDIR)/proto/api/component/camera/v1/camera.grpc.pb.cc $(GRPCDIR)/proto/api/component/camera/v1/camera.pb.cc
SOURCES += $(GRPCDIR)/google/api/annotations.pb.cc $(GRPCDIR)/google/api/httpbody.pb.cc
SOURCES += $(GRPCDIR)/google/api/http.pb.cc

buf: bufsetup
	PATH="${PATH}:`pwd`/grpc/bin" buf generate --template ./grpc/buf.gen.yaml buf.build/viamrobotics/rdk
	PATH="${PATH}:`pwd`/grpc/bin" buf generate --template ./grpc/buf.google.gen.yaml buf.build/googleapis/googleapis

bufsetup:
	sudo apt-get install -y protobuf-compiler-grpc libgrpc-dev libgrpc++-dev || brew install grpc openssl --quiet
	GOBIN=`pwd`/grpc/bin go install github.com/bufbuild/buf/cmd/buf@latest
	ln -sf `which grpc_cpp_plugin` grpc/bin/protoc-gen-grpc-cpp
	pkg-config openssl || export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:`find \`which brew > /dev/null && brew --prefix\` -name openssl.pc | head -n1 | xargs dirname`

pull-rdk: pull-rdk.sh
	./pull-rdk.sh

cubeeyegrpc: cubeeyeGRPC.cpp $(LIB_FILES) $(SOURCES)
	g++ -g -std=c++17 cubeeyeGRPC.cpp $(LIB_FILES) `pkg-config --cflags --libs libhttpserver cubeeye` $(SOURCES) -I$(GRPCDIR) $(GRPCFLAGS) -o cubeeyegrpcserver