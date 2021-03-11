Robust Railroad Infrastructure Detection Framework
=========================================

This software library and tool provides a fast and robust solution to extract various railroad infrastructure from dense (MLS) LiDAR point clouds. Primary focus is given to cable and railtrack detection.


Operating system
---------------

The project is built under Linux. It is recommended to use a 64-bit operating system.  
Ubuntu 16.04 LTS and 18.04 LTS was used during development.

Dependencies
--------------

The project uses some third-party dependencies.
These can be installed from the official repository of the given Linux distribution.
*  [GNU Make](https://www.gnu.org/software/make/)
*  [CMake](https://cmake.org/)
*  [PCL](http://pointclouds.org/)
*  [OpenCV](https://opencv.org/)

The following command installs the packages:
```bash
sudo apt-get update
sudo apt-get install build-essential make cmake libpcl-dev libproj-dev libopencv-dev
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

After successful compilation the `railroad` executable can be used for the railroad infrastructure detection algorithms.
Sample execution:
```bash
railroad --input cloud.laz
```

### Allowed options
| Option | Description | Mandatory |
|--------|-------------|-----------|
| `--input <path>` | input file path | YES |
| `--seed <path>` | seed file path | |
| `--verify <path>` | verifier file path | |
| `--size <N>` | maximum size of point cloud to process | |
| `--algorithm <alg1> ... <algN>` | specify the algorithm pipes to execute (default: all) | |
| `--boundaries <minX>, <minY>, <maxX>, <maxY>` | boundaries to process | |
| `--usePCDAsCache` | create and use PCD file of LAZ as cache | |
| `--loglevel <level>` | log level (trace, debug, info, warning, error, fatal; default: info) | |
| `--help` | produce help message | |

### Implemented algorithms
| Name | Algorithm pipe | Infrastructure |
|------|----------------|----------------|
| Voronoi | CutFilter(FROM_ABOVE_VORONOI) | cable |
| Skeleton | CutFilter(FROM_ABOVE_SKELETON) | cable |
| Angle | CutFilter(FROM_ABOVE_ANGLE) | cable |
| Ground | GroundFilter | cable |
| Above | AboveFilter | cable |
| Density | DensityFilter | cable |
| AngleGround | CutFilter(FROM_ABOVE_ANGLE) <br> GroundFilter | cable |
| AngleGroundAbove | CutFilter(FROM_ABOVE_ANGLE) <br> GroundFilter <br> AboveFilter | cable |
| AngleGroundAboveCylinder | CutFilter(FROM_ABOVE_ANGLE) <br> GroundFilter <br> AboveFilter <br> CylinderFilter | cable |
| AngleGroundCylinder | CutFilter(FROM_ABOVE_ANGLE) <br> GroundFilter <br> CylinderFilter | cable |
| AngleAbove | CutFilter(FROM_ABOVE_ANGLE) <br> AboveFilter | cable |
| AngleAboveCylinder | CutFilter(FROM_ABOVE_ANGLE) <br> AboveFilter <br> CylinderFilter | cable |
| VoronoiGround | CutFilter(FROM_ABOVE_VORONOI) <br> GroundFilter | cable |
| VoronoiGroundAbove | CutFilter(FROM_ABOVE_VORONOI) <br> GroundFilter <br> AboveFilter | cable | cable |
| VoronoiGroundAboveCylinder | CutFilter(FROM_ABOVE_VORONOI) <br> GroundFilter <br> AboveFilter <br> CylinderFilter | cable |
| VoronoiGroundCylinder | CutFilter(FROM_ABOVE_VORONOI) <br> GroundFilter <br> CylinderFilter | cable |
| VoronoiAbove | CutFilter(FROM_ABOVE_VORONOI) <br> AboveFilter | cable |
| VoronoiAboveCylinder | CutFilter(FROM_ABOVE_VORONOI) <br> AboveFilter <br> CylinderFilter | cable | cable |
| GroundDensity | GroundFilter <br> DensityFilter | cable |
| GroundDensityAbove | GroundFilter <br> DensityFilter <br> AboveFilter | cable |
| GroundDensityAboveCylinder | GroundFilter <br> DensityFilter <br> AboveFilter <br> CylinderFilter | cable |
| GroundDensityCylinder | GroundFilter <br> DensityFilter <br> CylinderFilter | cable |
| HeightWidthRansac | HeightFilter <br> WidthFilter <br> RansacFilter | cable |
| HeightWidthHough3D | HeightFilter <br> WidthFilter <br> Hough3dFilter | cable |
| HeightGrowth | HeightFilter <br> GrowthFilter | cable |
| RailTrack | RailTrackFilter | rail track |

Sample results
------------
*Above* pipeline result:  
![Above pipeline result](figs/result_above.png)

*HeightGrowth* pipeline result:  
![HeightGrowth pipeline result](figs/result_heightgrowth.png)

*RailTrack* pipeline result:  
![RailTrack pipeline result](figs/result_railtrack.png)



Publications
------------
 * Máté Cserép, Péter Hudoba, Zoltán Vincellér: *Robust Railroad Cable Detection in Rural Areas from MLS Point Clouds*, In Proceedings of Free and Open Source Software for Geospatial (FOSS4G) Conference, Vol. 18 , Article 2, 2018, [DOI: 10.7275/z46z-xh51](https://doi.org/10.7275/z46z-xh51)
 * Friderika Mayer: *Powerline tracking and extraction from dense LiDAR point clouds*, MSc thesis, Eötvös Loránd University, 2020, [PDF](https://gis.inf.elte.hu/wordpress/wp-content/uploads/2020/07/mayer_friderika_msc_compressed.pdf)
 * Adalbert Demján: *Object extraction of rail track from VLS LiDAR data*, MSc thesis, Eötvös Loránd University, 2020, [PDF](https://gis.inf.elte.hu/wordpress/wp-content/uploads/2020/07/demjan_adalbert_msc_compressed.pdf)

Contributing
------------

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on coding conventions.

License
------------

This project is licensed under the BSD 3-Clause License - see the [LICENSE](LICENSE) file for details.
