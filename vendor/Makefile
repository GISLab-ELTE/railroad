CMAKE = cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
ROOT_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

.PHONY: all
all: LASlib LASzip Potree

.PHONY: LASlib
LASlib:
	$(MAKE) -C LAStools/LASlib

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
	$(MAKE) -C LAStools/LASlib clean
	rm -rf LASzip/build
	rm -rf PotreeConverter/build
