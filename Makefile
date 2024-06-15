.PHONY: all list-projects

DOCKER_IMAGE ?= aabadie/dotbot:latest
DOCKER_TARGETS ?= all
SEGGER_DIR ?= /opt/segger
BUILD_CONFIG ?= Debug
SEGGER_PROJECT ?= aiot_play_fw.emProject

all:
	@echo "\e[1mBuilding all projects\e[0m"
	"$(SEGGER_DIR)/bin/emBuild" $(SEGGER_PROJECT) -config $(BUILD_CONFIG) -rebuild -verbose
	@echo "\e[1mDone\e[0m\n"

clean:
	"$(SEGGER_DIR)/bin/emBuild" $(SEGGER_PROJECT) -config $(BUILD_CONFIG) -clean

docker:
	docker run --rm -i \
		-e BUILD_CONFIG="$(BUILD_CONFIG)" \
		-e PACKAGES_DIR_OPT="-packagesdir $(SEGGER_DIR)/packages" \
		-e SEGGER_DIR="$(SEGGER_DIR)" \
		-v $(PWD):/dotbot $(DOCKER_IMAGE) \
		make $(DOCKER_TARGETS)
