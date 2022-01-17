Usage instructions
=========================================

This document describes how to used the compiled executables of the project.
Read the [Build instructions](BUILD.md) first in case you have not compiled the project yet.

Algorithm testing and benchmarking
--------------

After successful compilation the `railroad_benchmark` executable can be used to evaluate and benchmark a single or multiple railroad infrastructure detection algorithm.
Sample execution:
```bash
railroad_benchmark --input cloud.laz
```

### Allowed options
| Option | Description | Mandatory |
|--------|-------------|-----------|
| `--input <path>` | input file path | YES |
| `--seed <path>` | seed file path | |
| `--verify <path>` | verifier file path | |
| `--output <path>` | output directory path (default: ./) | |
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
| RailTrackWithSeed | WidthFilter <br> RailTrackFilter | rail track |


Combined detection
--------------

The `railroad_combined` executable can be used perform combined railroad infrastructure detection (both cable and railtrack) with a given algorithm pair.
Sample execution:
```bash
railroad_combined --input cloud.laz
```

### Allowed options
| Option | Description | Mandatory |
|--------|-------------|-----------|
| `--input <path>` | input file path | YES |
| `--seed <path>` | seed file path for cable detection | |
| `--output <path>` | output directory path (default: `./`) | |
| `--algorithm-cable <alg>` | specify the algorithm pipe to execute for cable detection (default: `AngleAbove`) | |
| `--algorithm-rail <alg>` | specify the algorithm pipe to execute for rail track detection (default: `RailTrackWithSeed`) | |
| `--size <N>` | maximum size of point cloud to process | |
| `--boundaries <minX>, <minY>, <maxX>, <maxY>` | boundaries to process | |
| `--usePCDAsCache` | create and use PCD file of LAZ as cache | |
| `--loglevel <level>` | log level (trace, debug, info, warning, error, fatal; default: info) | |
| `--help` | produce help message | |

*Remark*: the result of the cable detection will be automatically used as the seed for rail track detection.