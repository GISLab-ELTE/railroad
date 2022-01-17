CMAKE = cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
ROOT_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
MKDIR = mkdir -p
RMDIR = rm -rf

.PHONY: all
all: LASlib LASzip Potree

.ONESHELL:
.PHONY: LASlib
LASlib:
	cd LAStools
	$(MKDIR) build && cd build
	$(CMAKE) -DCMAKE_INSTALL_PREFIX=../install ..
	$(MAKE)
	$(MAKE) install

.PHONY: LASzip
LASzip:
	mkdir -p LASzip/build
	cd LASzip/build; \
	$(CMAKE) ..; \
	$(MAKE)

.PHONY: Potree
Potree: LASzip
	mkdir -p PotreeConverter/build
	cd PotreeConverter/build; \
	$(CMAKE) .. \
		-DLASZIP_INCLUDE_DIRS=$(ROOT_DIR)/LASzip/include/laszip \
		-DLASZIP_LIBRARY=$(ROOT_DIR)/LASzip/build/lib/liblaszip.so; \
	$(MAKE)

.PHONY: clean
clean:
	$(RMDIR) LAStools/build LAStools/install
	$(RMDIR) LASzip/build
	$(RMDIR) PotreeConverter/build
