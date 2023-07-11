Build instructions
=========================================

This document summarizes how to build and install the project and its dependencies.

Operating system
---------------

The project is built under Linux. It is recommended to use a 64-bit operating system.  
Ubuntu 20.04 LTS was used during development.

Dependencies
--------------

The project uses some third-party dependencies.
These can be installed from the official repository of the given Linux distribution.
*  [GNU Make](https://www.gnu.org/software/make/)
*  [Boost](https://www.boost.org/), version >= 1.71
*  [CMake](https://cmake.org/), version >= 3.16
*  [PCL](http://pointclouds.org/), version >= 1.10
*  [OpenCV](https://opencv.org/), version >= 4.0

The following command installs the packages:
```bash
sudo apt-get update
sudo apt-get install build-essential make cmake libpcl-dev libproj-dev libopencv-dev
```

Compilation
--------------

### Submodules

The project depends on some locally built tools:
* [LAStools](https://github.com/LAStools/LAStools) (LASlib and LASzip)

Check out the repository, then initialize the submodules to download the locally built dependencies:
```bash
git submodule init
git submodule update
```

Compile the dependencies:
```bash
cd vendor
make
```

*Note:* rerun when tools change in the vendor directory.

### Build the project

Configure the project:
```bash
mkdir build
cd build
cmake ../src
```

Compile the project:
```bash
make
```

*Note:* you may add the `-j<N>` flag to compile on multiple threads (where `<N>` is the number of threads).

## Install the program

If you would like to deploy the final compiled binaries to a distinct folder, you may install the project.
```bash
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=<install_path>
make
make install
```
