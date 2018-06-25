Robust Railroad Cable Detection Framework
=========================================

Operating system
---------------

The project is built under Linux. It is recommended to use a 64-bit operating system.  
Ubuntu 16.04 LTS was used during development.

Dependencies
--------------

The project uses some third-party dependencies.
These can be installed from the official repository of the given Linux distribution.
In Ubuntu 16.04 LTS the following packages are necessary for building:
*  [GNU Make](https://www.gnu.org/software/make/)
*  [CMake](https://cmake.org/)
*  [PCL](http://pointclouds.org/)
*  [OpenCV](https://opencv.org/)

The following command installs the packages:
```bash
sudo apt-get update
sudo apt-get install binutils make cmake libpcl-dev libproj-dev libopencv-dev
```

How to build
--------------

### Submodules

The project depends on some locally built tools:
* [LASzip](https://github.com/LASzip/LASzip)
* [LASlib](https://github.com/LAStools/LAStools/tree/master/LASlib)
* [PotreeConverter](https://github.com/potree/PotreeConverter)

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

How to use
--------------

After successful compilation the `railroad` executable can be used for cable detection algorithms.
Sample execution:
```bash
railroad --input cloud.laz --verify cable.laz
```

### Allowed options
| Option | Description |
|--------|-------------|
| `--input <path>` | input file path |
| `--verify <path>` | verifier file path |
| `--size <N>` | maximum size of point cloud to process |
| `--algorithm <alg1> ... <algN>` | specify the algorithm pipes to execute (default: all) |
| `--boundaries <minX>, <minY>, <maxX>, <maxY>` | boundaries to process |
| `--usePCDAsCache` | create and use PCD file of LAZ as cache |
| `--loglevel <level>` | log level (trace, debug, info, warning, error, fatal; default: info)
| `--help` | produce help message |

### Implemented algorithms
| Name | Algorithm pipe |
|------|----------------|
| Voronoi | CutFilter(FROM_ABOVE_VORONOI) |
| Skeleton | CutFilter(FROM_ABOVE_SKELETON) |
| Angle | CutFilter(FROM_ABOVE_ANGLE) |
| Ground | GroundFilter |
| Above | AboveFilter |
| Density | DensityFilter |
| AngleGround | LimiterFilter <br> CutFilter(FROM_ABOVE_ANGLE) <br> GroundFilter |
| AngleGroundAbove | LimiterFilter <br> CutFilter(FROM_ABOVE_ANGLE) <br> GroundFilter <br> AboveFilter |
| AngleGroundAboveCylinder | LimiterFilter <br> CutFilter(FROM_ABOVE_ANGLE) <br> GroundFilter <br> AboveFilter <br> CylinderFilter |
| AngleGroundCylinder | LimiterFilter <br> CutFilter(FROM_ABOVE_ANGLE) <br> GroundFilter <br> CylinderFilter |
| AngleAbove | LimiterFilter <br> CutFilter(FROM_ABOVE_ANGLE) <br> AboveFilter |
| AngleAboveCylinder | LimiterFilter <br> CutFilter(FROM_ABOVE_ANGLE) <br> AboveFilter <br> CylinderFilter |
| VoronoiGround | LimiterFilter <br> CutFilter(FROM_ABOVE_VORONOI) <br> GroundFilter |
| VoronoiGroundAbove | LimiterFilter <br> CutFilter(FROM_ABOVE_VORONOI) <br> GroundFilter <br> AboveFilter |
| VoronoiGroundAboveCylinder | LimiterFilter <br> CutFilter(FROM_ABOVE_VORONOI) <br> GroundFilter <br> AboveFilter <br> CylinderFilter |
| VoronoiGroundCylinder | LimiterFilter <br> CutFilter(FROM_ABOVE_VORONOI) <br> GroundFilter <br> CylinderFilter |
| VoronoiAbove | LimiterFilter <br> CutFilter(FROM_ABOVE_VORONOI) <br> AboveFilter |
| VoronoiAboveCylinder | LimiterFilter <br> CutFilter(FROM_ABOVE_VORONOI) <br> AboveFilter <br> CylinderFilter |
| GroundDensity | GroundFilter <br> DensityFilter |
| GroundDensityAbove | GroundFilter <br> DensityFilter <br> AboveFilter |        
| GroundDensityAboveCylinder | GroundFilter <br> DensityFilter <br> AboveFilter <br> CylinderFilter |
| GroundDensityCylinder | GroundFilter <br> DensityFilter <br> CylinderFilter |

Contributing
------------

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on coding conventions.

License
------------

This project is licensed under the BSD 3-Clause License - see the [LICENSE](LICENSE) file for details.
