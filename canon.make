DOCKER_CMD = docker run $(DOCKER_SSH_AGENT) $(DOCKER_NETRC_RUN) -v$(HOME)/.ssh:/home/testbot/.ssh:ro -v$(shell pwd):/host --workdir /host --rm -ti $(DOCKER_PLATFORM) ghcr.io/viamrobotics/canon:$(DOCKER_TAG) --testbot-uid $(shell id -u) --testbot-gid $(shell id -g)

ifeq ("aarch64", "$(shell uname -m)")
	DOCKER_NATIVE_PLATFORM = --platform linux/arm64
	DOCKER_NATIVE_TAG = arm64
	DOCKER_NATIVE_TAG_CACHE = arm64-cache
else ifeq ("arm64", "$(shell uname -m)")
	DOCKER_NATIVE_PLATFORM = --platform linux/arm64
	DOCKER_NATIVE_TAG = arm64
	DOCKER_NATIVE_TAG_CACHE = arm64-cache
else ifeq ("x86_64", "$(shell uname -m)")
	DOCKER_NATIVE_PLATFORM = --platform linux/amd64
	DOCKER_NATIVE_TAG = amd64
	DOCKER_NATIVE_TAG_CACHE = amd64-cache
else
	DOCKER_NATIVE_TAG = latest
	DOCKER_NATIVE_TAG_CACHE = latest
endif

# If there's a netrc file, use it.
ifeq ($(shell grep -qs github.com ~/.netrc && `which echo` -n yes), yes)
	DOCKER_NETRC_BUILD = --secret id=netrc,src=$(HOME)/.netrc
	DOCKER_NETRC_RUN = -v$(HOME)/.netrc:/home/testbot/.netrc:ro
endif

ifeq ("Darwin", "$(shell uname -s)")
	# Docker has magic paths for OSX
	DOCKER_SSH_AGENT = -v /run/host-services/ssh-auth.sock:/run/host-services/ssh-auth.sock -e SSH_AUTH_SOCK="/run/host-services/ssh-auth.sock"
else ifneq ("$(SSH_AUTH_SOCK)x", "x")
	DOCKER_SSH_AGENT = -v "$(SSH_AUTH_SOCK):$(SSH_AUTH_SOCK)" -e SSH_AUTH_SOCK="$(SSH_AUTH_SOCK)"
endif

DOCKER_PLATFORM = $(DOCKER_NATIVE_PLATFORM)
DOCKER_TAG = $(DOCKER_NATIVE_TAG)

# This sets up multi-arch emulation under linux. Run before using multi-arch targets.
canon-emulation:
	docker run --rm --privileged multiarch/qemu-user-static --reset -c yes -p yes

# Canon shells use the raw (non-cached) canon docker image
canon-shell:
	$(DOCKER_CMD) bash

canon-shell-amd64: DOCKER_PLATFORM = --platform linux/amd64
canon-shell-amd64: DOCKER_TAG = amd64
canon-shell-amd64:
	$(DOCKER_CMD) bash

canon-shell-arm64: DOCKER_PLATFORM = --platform linux/arm64
canon-shell-arm64: DOCKER_TAG = arm64
canon-shell-arm64:
	$(DOCKER_CMD) bash

# AppImage packaging targets run in canon docker
appimages-multiarch: appimages-amd64 appimages-arm64

appimages-amd64: DOCKER_PLATFORM = --platform linux/amd64
appimages-amd64: DOCKER_TAG = amd64-cache
appimages-amd64:
	$(DOCKER_CMD) make appimages

appimages-arm64: DOCKER_PLATFORM = --platform linux/arm64
appimages-arm64: DOCKER_TAG = arm64-cache
appimages-arm64:
	$(DOCKER_CMD) make appimages
