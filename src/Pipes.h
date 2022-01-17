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

#include "piping/ProcessorPipeBunch.h"
#include "piping/ProcessorPipe.h"

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
                  ->add(new HeightFilter())
                  ->add(new WidthFilter())
                  ->add(new RansacFilter())
        )
        ->add("Hough", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new HeightFilter())
                  ->add(new Hough3dFilter())
        )
        ->add("Growth", LASClass::CABLE,
              (new ProcessorPipe())
                  ->add(new HeightFilter())
                  ->add(new GrowthFilter())
        )

        // Author: Adalbert Demján
        ->add("RailTrack", LASClass::RAIL,
              (new ProcessorPipe())
                  ->add(new RailTrackFilter())
        )
        ->add("RailTrackWithSeed", LASClass::RAIL,
              (new ProcessorPipe())
                  ->add(new WidthFilter())
                  ->add(new RailTrackFilter())
        );

    return pipesHolder;
}

} // railroad

#endif //RAILROAD_PIPES_H
