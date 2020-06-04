/*
 * BSD 3-Clause License
 * Copyright (c) 2018-2020, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <iostream>
#include <string>
#include <climits>
#include <vector>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <lasdefinitions.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "helpers/LogHelper.h"
#include "helpers/LASHelper.h"
#include "helpers/PCLHelper.h"

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

#include "Results.h"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace logging = boost::log;

using namespace railroad;

enum ExitCodes
{
    // Success
    Success = 0,
    UserAbort = -1,
    NoResult = -2,

    // Errors
    InvalidInput = 1,
    UnexcpectedError = 2,
    Unsupported = 3,
};

ProcessorPipeBunch *generateTestPipes();

int main(int argc, char *argv[])
{
    std::string inputFile;
    std::string seedFile;
    std::string verifierFile;
    unsigned long maxSize;
    std::vector<long> boundaries;
    std::vector<std::string> pipes;
    logging::trivial::severity_level logLevel;

    // Read console arguments
    po::options_description desc("Allowed options");
    desc.add_options()
        ("input,i", po::value<std::string>(&inputFile),
         "input file path")
        ("seed", po::value<std::string>(&seedFile)->default_value(std::string()),
         "seed file path (used by some algorithms)")
        ("verify", po::value<std::string>(&verifierFile)->default_value(std::string()),
         "verifier file path")
        ("size", po::value<unsigned long>(&maxSize)->default_value(ULONG_MAX),
         "maximum size of point cloud to process")
        ("algorithm,a", po::value<std::vector<std::string>>(&pipes)->multitoken(),
         "specify the algorithm pipes to execute (default: all)")
        ("boundaries", po::value<std::vector<long>>(&boundaries)->multitoken(),
         "minX, minY, maxX, maxY boundaries to process")
        ("usePCDAsCache", "create and use PCD file of LAZ as cache")
        ("loglevel", po::value<logging::trivial::severity_level>(&logLevel)->default_value(logging::trivial::info),
         "log level (trace, debug, info, warning, error, fatal)")
        ("help,h", "produce help message");


    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // Initialize the logger
    initLogger(logLevel);

    // Argument validation
    if (vm.count("help")) {
        std::cout << "BSD 3-Clause License" << std::endl
                  << "Copyright (c) 2018-2020, Máté Cserép & Péter Hudoba" << std::endl
                  << "All rights reserved." << std::endl
                  << std::endl
                  << "You may obtain a copy of the License at" << std::endl
                  << "https://opensource.org/licenses/BSD-3-Clause" << std::endl
                  << std::endl;

        std::cout << "Executes railroad cable detection algorithms." << std::endl;
        std::cout << desc << std::endl;
        return Success;
    }

    bool argumentError = false;
    if (!vm.count("input")) {
        std::cerr << "Input file is mandatory." << std::endl;
        argumentError = true;

        if (!fs::exists(inputFile)) {
            std::cerr << "The input file does not exist." << std::endl;
            argumentError = true;
        }
    }

    if (seedFile.length() > 0 && !fs::exists(seedFile)) {
        std::cerr << "The seed file does not exist." << std::endl;
        argumentError = true;
    }

    if (verifierFile.length() > 0 && !fs::exists(verifierFile)) {
        std::cerr << "The verifier file does not exist." << std::endl;
        argumentError = true;
    }

    if (vm.count("boundaries") && boundaries.size() != 4) {
        std::cerr << "Give 4 boundary coordinates: minX, minY, maxX, maxY." << std::endl;
        argumentError = true;
    }

    if (argumentError) {
        std::cerr << "Use the --help option for description." << std::endl;
        return InvalidInput;
    }

    // Input handling
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr seed;
    pcl::PointCloud<pcl::PointXYZ>::Ptr verifier;
    LASheader outputHeader;

    // Read input file
    if (vm.count("usePCDAsCache")) {
        LOG(debug) << "PDA Cache is active";
        if (!fs::exists(inputFile + ".pcd")) {
            LOG(debug) << "PDA Cache not found... generating";
            if (!vm.count("boundaries"))
                cloud = readLAS(inputFile, outputHeader, maxSize);
            else
                cloud = readLAS(inputFile, outputHeader,
                                boundaries[0], boundaries[2], boundaries[1], boundaries[3], maxSize);
            pcl::io::savePCDFileBinary(inputFile + ".pcd", *cloud);
        } else {
            LOG(debug) << "PDA Cache has been found";
            outputHeader = readLASHeader(inputFile);
            cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPCDFile<pcl::PointXYZ>(inputFile + ".pcd", *cloud);
        }
    } else {
        if (!vm.count("boundaries"))
            cloud = readLAS(inputFile, outputHeader, maxSize);
        else
            cloud = readLAS(inputFile, outputHeader,
                            boundaries[0], boundaries[2], boundaries[1], boundaries[3], maxSize);
    }
    LOG(info) << "Input file loaded, size: " << cloud->size();

    // Read seed file
    if (seedFile.length() > 0) {
        seed = readLAS(seedFile);
        LOG(info) << "Seed file loaded, size: " << seed->size();
    }

    // Read verifier file
    if (verifierFile.length() > 0) {
        verifier = readLAS(verifierFile);
        LOG(info) << "Verifier file loaded, size: " << verifier->size();
    }

    // Processing algorithms
    CloudProcessor *algorithm;
    std::string outputFile;
    pcl::PointCloud<pcl::PointXYZ>::Ptr result;
    pcl::PointCloud<pcl::PointXYZI>::Ptr visual;

    {
        ProcessorPipeBunch *pipesHolder = generateTestPipes();

        if (vm.count("algorithm")) {
            for (auto algorithm : pipes)
                pipesHolder->addToRunOnlyList(algorithm);
        }

        Results<double> results;
        Results<int> pointReductionResults;
        for (auto pipeElement : pipesHolder->getPipes()) {
            LOG(info) << "Starting " << pipeElement.name << " case";
            algorithm = pipeElement.pipe;
            algorithm->setInputCloud(cloud);
            if (seedFile.length() > 0)
                algorithm->setSeedCloud(seed);
            mkdir(pipeElement.name.c_str(), 0777);
            chdir(pipeElement.name.c_str());
            result = algorithm->execute();
            chdir("..");
            auto pipeResults = pipeElement.pipe->getTimeResults();
            outputFile = pipeElement.name + ".laz";

            LOG(debug) << "Calculating visuals";
            visual = mergePointCloudsVisual(cloud, result, 1);
            LOG(debug) << "Writing results";
            writeLAS(outputFile, outputHeader, visual);
            LOG(info) << "Finished writing results: " << outputFile;

            if (verifierFile.length() > 0) {
                LOG(debug) << "Result size: " << result->size()
                           << ", Verifier size: " << verifier->size();

                int falsePositives = diffPointClouds(result, verifier, 3)->size();
                double percentage = 100.0 * falsePositives / result->size();
                pipeResults["accuracy_false_positive"] = percentage;
                LOG(info) << "False positives: " << falsePositives
                          << " (" << percentage << "%)";

                int falseNegatives = diffPointClouds(verifier, result, 3)->size();
                percentage = 100.0 * falseNegatives / verifier->size();
                pipeResults["accuracy_false_negative"] = percentage;
                LOG(info) << "False negatives: " << falseNegatives
                          << " (" << percentage << "%)";
            }

            results.add(pipeElement.name, pipeResults);
            LOG(info) << "Time results:";
            for (auto const &x : pipeResults) {
                LOG(info) << x.first << ":" << x.second;
            }
            pointReductionResults.add(pipeElement.name, pipeElement.pipe->getFilteredPointsResults());
            delete algorithm;
        }
        std::cout << results;
        std::cout << pointReductionResults;

        delete pipesHolder;
    }
    return Success;
}

ProcessorPipeBunch *generateTestPipes()
{
    ProcessorPipeBunch *pipesHolder = (new ProcessorPipeBunch())

        ->add("Voronoi",
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
        )
        ->add("Skeleton",
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::SKELETON))
        )
        ->add("Angle",
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::ANGLE))
        )
        ->add("Ground",
              (new ProcessorPipe())
                  ->add(new GroundFilter())
        )
        ->add("Above",
              (new ProcessorPipe())
                  ->add(new AboveFilter())
        )
        ->add("Density",
              (new ProcessorPipe())
                  ->add(new DensityFilter())
        )
        ->add("AngleGround",
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::ANGLE))
                  ->add(new GroundFilter())
        )
        ->add("AngleGroundAbove",
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::ANGLE))
                  ->add(new GroundFilter())
                  ->add(new AboveFilter())
        )
        ->add("AngleGroundAboveCylinder",
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::ANGLE))
                  ->add(new GroundFilter())
                  ->add(new AboveFilter())
                  ->add(new CylinderFilter())
        )
        ->add("AngleGroundCylinder",
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::ANGLE))
                  ->add(new GroundFilter())
                  ->add(new CylinderFilter())
        )
        ->add("AngleAbove",
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::ANGLE))
                  ->add(new AboveFilter())
        )
        ->add("AngleAboveCylinder",
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::ANGLE))
                  ->add(new AboveFilter())
                  ->add(new CylinderFilter())
        )
        ->add("VoronoiGround",
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
                  ->add(new GroundFilter())
        )
        ->add("VoronoiGroundAbove",
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
                  ->add(new GroundFilter())
                  ->add(new AboveFilter())
        )
        ->add("VoronoiGroundAboveCylinder",
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
                  ->add(new GroundFilter())
                  ->add(new AboveFilter())
                  ->add(new CylinderFilter())
        )
        ->add("VoronoiGroundCylinder",
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
                  ->add(new GroundFilter())
                  ->add(new CylinderFilter())
        )
        ->add("VoronoiAbove",
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
                  ->add(new AboveFilter())
        )
        ->add("VoronoiAboveCylinder",
              (new ProcessorPipe())
                  ->add(new CutFilter(ImportantPartFinderProcessor::VORONOI))
                  ->add(new AboveFilter())
                  ->add(new CylinderFilter())
        )
        ->add("GroundDensity",
              (new ProcessorPipe())
                  ->add(new GroundFilter())
                  ->add(new DensityFilter())
        )
        ->add("GroundDensityAbove",
              (new ProcessorPipe())
                  ->add(new GroundFilter())
                  ->add(new DensityFilter())
                  ->add(new AboveFilter())
        )
        ->add("GroundDensityAboveCylinder",
              (new ProcessorPipe())
                  ->add(new GroundFilter())
                  ->add(new DensityFilter())
                  ->add(new AboveFilter())
                  ->add(new CylinderFilter())
        )
        ->add("GroundDensityCylinder",
              (new ProcessorPipe())
                  ->add(new GroundFilter())
                  ->add(new DensityFilter())
                  ->add(new CylinderFilter())
        )
        
        // Author: Friderika Mayer
        ->add("Ransac",
              (new ProcessorPipe())
                  ->add(new HeightFilter())
                  ->add(new WidthFilter())
                  ->add(new RansacFilter())
        )
        ->add("Hough",
              (new ProcessorPipe())
                  ->add(new HeightFilter())
                  ->add(new Hough3dFilter())
        )
        ->add("Growth",
              (new ProcessorPipe())
                  ->add(new HeightFilter())
                  ->add(new GrowthFilter())
        )
        
        // Author: Adalbert Demján
        ->add("RailTrack",
             (new ProcessorPipe())
                  ->add(new RailTrackFilter())
        );

    return pipesHolder;
}