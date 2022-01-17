CMAKE = cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
ROOT_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
MKDIR = mkdir -p
RMDIR = rm -rf

.PHONY: all
all: LASlib

.ONESHELL:
.PHONY: LASlib
LASlib:
	cd LAStools
	$(MKDIR) build && cd build
	$(CMAKE) -DCMAKE_INSTALL_PREFIX=../install ..
	$(MAKE)
	$(MAKE) install

.PHONY: clean
clean:
	$(RMDIR) LAStools/build LAStools/install
