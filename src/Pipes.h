/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_PIPES_H
#define RAILROAD_PIPES_H

#include "filters/DensityFilter.h"
#include "filters/AboveFilter.h"
#include "filters/CylinderFilter.h"
#include "filters/GroundFilter.h"
#include "filters/OutlierFilter.h"
#include "filters/CutFilter.h"
#include "filters/LimiterFilter.h"
#include "filters/RansacFilter.h"
#include "filters/Hough3dFilter.h"
#include "filters/GrowthFilter.h"
#include "filters/RailTrackFilter.h"
#include "filters/HeightFilter.h"
#include "filters/WidthFilter.h"
#include "filters/CableDistanceFilter.h"
#include "filters/StructureGaugeFilter.h"
#include "filters/VegetationDetectionFilter.h"
#include "filters/BandPassFilter.h"
#include "filters/EuclideanClusterFilter.h"
#include "filters/MinDistanceClusterFilter.h"
#include "filters/RansacCylinderFilter.h"
#include "filters/CantileverFilter.h"
#include "filters/MaxHeightFilter.h"
#include "filters/MinHeightFilter.h"
#include "filters/StaggerFilter.h"
#include "filters/CorrigateCentroidsFilter.h"

#include "piping/ProcessorPipeBunch.h"
#include "piping/ProcessorPipe.h"
#include <vector>
namespace railroad
{

ProcessorPipeBunch *generateTestPipes()
{
    ProcessorPipeBunch *pipesHolder = (new ProcessorPipeBunch())

        ->add("Voronoi", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
        )
        ->add("Skeleton", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::SKELETON))
        )
        ->add("Angle", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::ANGLE))
        )
        ->add("Ground", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new GroundFilter())
        )
        ->add("Above", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new AboveFilter())
        )
        ->add("Density", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new DensityFilter())
        )
        ->add("AngleGround", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::ANGLE))
                  ->add(new GroundFilter())
        )
        ->add("AngleGroundAbove", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::ANGLE))
                  ->add(new GroundFilter())
                  ->add(new AboveFilter())
        )
        ->add("AngleGroundAboveCylinder", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::ANGLE))
                  ->add(new GroundFilter())
                  ->add(new AboveFilter())
                  ->add(new CylinderFilter())
        )
        ->add("AngleGroundCylinder", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::ANGLE))
                  ->add(new GroundFilter())
                  ->add(new CylinderFilter())
        )
        ->add("AngleAbove", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::ANGLE))
                  ->add(new AboveFilter())
        )
        ->add("AngleAboveCylinder", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::ANGLE))
                  ->add(new AboveFilter())
                  ->add(new CylinderFilter())
        )
        ->add("VoronoiGround", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
                  ->add(new GroundFilter())
        )
        ->add("VoronoiGroundAbove", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
                  ->add(new GroundFilter())
                  ->add(new AboveFilter())
        )
        ->add("VoronoiGroundAboveCylinder", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
                  ->add(new GroundFilter())
                  ->add(new AboveFilter())
                  ->add(new CylinderFilter())
        )
        ->add("VoronoiGroundCylinder", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
                  ->add(new GroundFilter())
                  ->add(new CylinderFilter())
        )
        ->add("VoronoiAbove", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
                  ->add(new AboveFilter())
        )
        ->add("VoronoiAboveCylinder", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
                  ->add(new AboveFilter())
                  ->add(new CylinderFilter())
        )
        ->add("GroundDensity", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new GroundFilter())
                  ->add(new DensityFilter())
        )
        ->add("GroundDensityAbove", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new GroundFilter())
                  ->add(new DensityFilter())
                  ->add(new AboveFilter())
        )
        ->add("GroundDensityAboveCylinder", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new GroundFilter())
                  ->add(new DensityFilter())
                  ->add(new AboveFilter())
                  ->add(new CylinderFilter())
        )
        ->add("GroundDensityCylinder", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new GroundFilter())
                  ->add(new DensityFilter())
                  ->add(new CylinderFilter())
        )

        // Author: Friderika Mayer
        ->add("Ransac", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new HeightFilter(SeedHelper::CABLE))
                  ->add(new WidthFilter(SeedHelper::CABLE))
                  ->add(new RansacFilter())
        )
        ->add("Hough", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new HeightFilter(SeedHelper::CABLE))
                  ->add(new Hough3dFilter())
        )
        ->add("Growth", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new HeightFilter(SeedHelper::CABLE))
                  ->add(new GrowthFilter(SeedHelper::CABLE))
        )

        // Author: Adalbert Demján
        ->add("RailTrack", LASClass::RAIL,
              (new ProcessorPipe())
                  ->add(new RailTrackFilter())
        )
        ->add("RailTrackWithSeed", LASClass::RAIL,
              (new ProcessorPipe())
                  ->add(new WidthFilter(SeedHelper::CABLE))
                  ->add(new RailTrackFilter())
        )

        // Author: Milán Horváth
        ->add("CableErrorDetection", LASClass::CABLE,
            std::vector<ProcessorPipe*>()= {
            (new ProcessorPipe())
                ->add(new CableDistanceFilter())
            ,(new ProcessorPipe())
                  ->add(new WidthFilter(SeedHelper::CABLE, 2))
                  ->add(new RailTrackFilter())
            ,(new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
                  ->add(new AboveFilter())
            }, 0

        )
        ->add("StructureGaugeDetection", LASClass::LOW_VEGETATION,
            std::vector<ProcessorPipe*>()= {
            (new ProcessorPipe())
                  ->add(new StructureGaugeFilter())
            ,(new ProcessorPipe())
                   ->add(new WidthFilter(SeedHelper::CABLE, 2))
                   ->add(new RailTrackFilter())
            ,(new ProcessorPipe())
                   ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
                   ->add(new AboveFilter())
            }

        )
        ->add("GroundErrorDetection", LASClass::GROUND,
            std::vector<ProcessorPipe*>()= {
            (new ProcessorPipe())
                  ->add(new VegetationDetectionFilter())
            ,(new ProcessorPipe())
                   ->add(new WidthFilter(SeedHelper::CABLE, 2))
                   ->add(new RailTrackFilter())
            ,(new ProcessorPipe())
                   ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
                   ->add(new AboveFilter())
            }
        )

        // Author: Attila Láber
        ->add("PoleDetection", LASClass::POLE,
              (new ProcessorPipe())
                  ->add(new WidthFilter(SeedHelper::RAIL, 2.2F))
                  ->add(new BandPassFilter(SeedHelper::RAIL, 1.0, 3))
                  ->add(new OutlierFilter(0.16, 150))
                  ->add(new BandPassFilter(0.2))
                  ->add(new MinDistanceClusterFilter(5.0))
                  ->add(new CorrigateCentroidsFilter(0.5))
                  ->add(new RansacCylinderFilter(0.22))
        )
        ->add("CantileverDetection", LASClass::CANTILEVER,
              (new ProcessorPipe())
                  ->add(new WidthFilter(SeedHelper::CABLE, 0.5))
                  ->add(new HeightFilter(SeedHelper::CABLE))
                  ->add(new CantileverFilter())
        )
        ->add("CableStaggerCheckingFirstClass", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new MinHeightFilter(0.18, false))
                  ->add(new RansacFilter(1000, 0.08))
                  ->add(new StaggerFilter(0.4, 0.01, 0.15))
        )
        ->add("CableStaggerCheckingLowerClass", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new MinHeightFilter(0.18, false))
                  ->add(new RansacFilter(1000, 0.08))
                  ->add(new StaggerFilter(0.4, 0.03, 0.15))
        )
        ->add("CableStaggerCheckingDemo", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new MinHeightFilter(0.18, false))
                  ->add(new RansacFilter(1000, 0.08))
                  ->add(new StaggerFilter(0.2, 0.01, 0.35))
        );

    return pipesHolder;
}

} // railroad

#endif //RAILROAD_PIPES_H
