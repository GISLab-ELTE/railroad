
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
| `--seedpaths <path1> ... <pathN>` | seed file paths | |
| `--seedtypes <type1> ... <typeN>` | seed types for seed file paths (rail, pole, cable, ties) | |
| `--verify <path>` | verifier file path | |
| `--output <path>` | output directory path (default: ./) | |
| `--size <N>` | maximum size of point cloud to process | |
| `--algorithm <alg1> ... <algN>` | specify the algorithm pipes to execute (default: all) | |
| `--shift` | shift point cloud to origo during processing to avoid precision loss (default: no) | |
| `--boundaries <minX>, <minY>, <maxX>, <maxY>` | boundaries to process | |
| `--usePCDAsCache` | create and use PCD file of LAZ as cache | |
| `--loglevel <level>` | log level (trace, debug, info, warning, error, fatal; default: info) | |
| `--help` | produce help message | |

### Implemented algorithms
| Name | Algorithm pipe | Infrastructure | Required seeds |
|------|----------------|----------------|----------------|
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
| VoronoiGroundAbove | CutFilter(FROM_ABOVE_VORONOI) <br> GroundFilter <br> AboveFilter | cable |
| VoronoiGroundAboveCylinder | CutFilter(FROM_ABOVE_VORONOI) <br> GroundFilter <br> AboveFilter <br> CylinderFilter | cable |
| VoronoiGroundCylinder | CutFilter(FROM_ABOVE_VORONOI) <br> GroundFilter <br> CylinderFilter | cable |
| VoronoiAbove | CutFilter(FROM_ABOVE_VORONOI) <br> AboveFilter | cable |
| VoronoiAboveCylinder | CutFilter(FROM_ABOVE_VORONOI) <br> AboveFilter <br> CylinderFilter | cable |
| GroundDensity | GroundFilter <br> DensityFilter | cable |
| GroundDensityAbove | GroundFilter <br> DensityFilter <br> AboveFilter | cable |
| GroundDensityAboveCylinder | GroundFilter <br> DensityFilter <br> AboveFilter <br> CylinderFilter | cable |
| GroundDensityCylinder | GroundFilter <br> DensityFilter <br> CylinderFilter | cable |
| HeightWidthRansac | HeightFilter <br> WidthFilter <br> RansacFilter | cable | CABLE (initial section) |
| HeightWidthHough3D | HeightFilter <br> WidthFilter <br> Hough3dFilter | cable | CABLE (initial section) |
| HeightGrowth | HeightFilter <br> GrowthFilter | cable | CABLE (initial section) |
| RailTrack | RailTrackFilter | rail track |
| RailTrackWithSeed | WidthFilter <br> RailTrackFilter | rail track | CABLE |
| PoleDetection | WidthFilter <br> BandPassFilter <br> OutlierFilter <br> BandPassFilter <br> MinDistanceClusterFilter <br> CorrigateCentroidsFilter <br> RansacCylinderFilter | mast | RAIL |
| CantileverDetection | WidthFilter <br> HeightFilter <br> CantileverFilter | cantilever |  CABLE <br> POLE |

Sample execution:
```bash
railroad_benchmark -a PoleDetection -i cloud.laz --seedpaths railtrack.laz --seedtypes RAIL
```

### Error detection algorithms
| Name | Algorithm pipe | Infrastructure | Required seeds |
|------|----------------|----------------|----------------|
| StructureGaugeDetection | StructureGaugeFilter | low vegetation | RAIL |
| CableErrorDetection | CableDetectionFilter | cable | RAIL <br> CABLE |
| GroundErrorDetection | VegetationDetectionFilter | ground | RAIL |
| CableStaggerCheckingFirstClass | MinHeightFilter <br> RansacFilter <br> StaggerFilter | cable |
| CableStaggerCheckingLowerClass | MinHeightFilter <br> RansacFilter <br> StaggerFilter | cable |


Sample execution:

```bash
railroad_benchmark -a CableErrorDetection -i cloud.laz --seedpaths railtrack.laz cable.laz --seedtypes RAIL CABLE
```


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
| `--output <path>` | output directory path (default: `./`) | |
| `--algorithm-cable <alg>` | specify the algorithm pipe to execute for cable detection (default: `AngleAbove`) | |
| `--algorithm-rail <alg>` | specify the algorithm pipe to execute for rail track detection (default: `RailTrackWithSeed`) | |
| `--size <N>` | maximum size of point cloud to process | |
| `--boundaries <minX>, <minY>, <maxX>, <maxY>` | boundaries to process | |
| `--usePCDAsCache` | create and use PCD file of LAZ as cache | |
| `--loglevel <level>` | log level (trace, debug, info, warning, error, fatal; default: info) | |
| `--help` | produce help message | |

*Remark*: the result of the cable detection will be automatically used as the seed for rail track detection.
