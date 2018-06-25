/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <iostream>
#include <string>
#include <algorithm>
#include <functional>
#include <climits>
#include <vector>
#include <map>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <lasreader.hpp>
#include <laswriter.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

#include "helpers/LogHelper.h"
#include "helpers/LASHelper.h"

#include "filters/DensityFilter.h"
#include "filters/AboveFilter.h"
#include "filters/CylinderFilter.h"
#include "filters/GroundFilter.h"
#include "filters/OutlierFilter.h"
#include "filters/CutFilter.h"
#include "filters/LimiterFilter.h"

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
                  << "Copyright (c) 2018, Máté Cserép & Péter Hudoba" << std::endl
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr verifier;
    LASheader outputHeader;

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
                verifier = readLAS(verifierFile);
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
    }
    return Success;
}

ProcessorPipeBunch *generateTestPipes()
{
    ProcessorPipeBunch *pipesHolder = (new ProcessorPipeBunch())

        ->add("Voronoi",
              (new ProcessorPipe())
                  ->add(new CutFilter(FROM_ABOVE_VORONOI))
        )
        ->add("Skeleton",
              (new ProcessorPipe())
                  ->add(new CutFilter(FROM_ABOVE_SKELETON))
        )
        ->add("Angle",
              (new ProcessorPipe())
                  ->add(new CutFilter(FROM_ABOVE_ANGLE))
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
                  ->add(new LimiterFilter())
                  ->add(new CutFilter(FROM_ABOVE_ANGLE))
                  ->add(new GroundFilter())
        )
        ->add("AngleGroundAbove",
              (new ProcessorPipe())
                  ->add(new LimiterFilter())
                  ->add(new CutFilter(FROM_ABOVE_ANGLE))
                  ->add(new GroundFilter())
                  ->add(new AboveFilter())
        )
        ->add("AngleGroundAboveCylinder",
              (new ProcessorPipe())
                  ->add(new LimiterFilter())
                  ->add(new CutFilter(FROM_ABOVE_ANGLE))
                  ->add(new GroundFilter())
                  ->add(new AboveFilter())
                  ->add(new CylinderFilter())
        )
        ->add("AngleGroundCylinder",
              (new ProcessorPipe())
                  ->add(new LimiterFilter())
                  ->add(new CutFilter(FROM_ABOVE_ANGLE))
                  ->add(new GroundFilter())
                  ->add(new CylinderFilter())
        )
        ->add("AngleAbove",
              (new ProcessorPipe())
                  ->add(new LimiterFilter())
                  ->add(new CutFilter(FROM_ABOVE_ANGLE))
                  ->add(new AboveFilter())
        )
        ->add("AngleAboveCylinder",
              (new ProcessorPipe())
                  ->add(new LimiterFilter())
                  ->add(new CutFilter(FROM_ABOVE_ANGLE))
                  ->add(new AboveFilter())
                  ->add(new CylinderFilter())
        )
        ->add("VoronoiGround",
              (new ProcessorPipe())
                  ->add(new LimiterFilter())
                  ->add(new CutFilter(FROM_ABOVE_VORONOI))
                  ->add(new GroundFilter())
        )
        ->add("VoronoiGroundAbove",
              (new ProcessorPipe())
                  ->add(new LimiterFilter())
                  ->add(new CutFilter(FROM_ABOVE_VORONOI))
                  ->add(new GroundFilter())
                  ->add(new AboveFilter())
        )
        ->add("VoronoiGroundAboveCylinder",
              (new ProcessorPipe())
                  ->add(new LimiterFilter())
                  ->add(new CutFilter(FROM_ABOVE_VORONOI))
                  ->add(new GroundFilter())
                  ->add(new AboveFilter())
                  ->add(new CylinderFilter())
        )
        ->add("VoronoiGroundCylinder",
              (new ProcessorPipe())
                  ->add(new LimiterFilter())
                  ->add(new CutFilter(FROM_ABOVE_VORONOI))
                  ->add(new GroundFilter())
                  ->add(new CylinderFilter())
        )
        ->add("VoronoiAbove",
              (new ProcessorPipe())
                  ->add(new LimiterFilter())
                  ->add(new CutFilter(FROM_ABOVE_VORONOI))
                  ->add(new AboveFilter())
        )
        ->add("VoronoiAboveCylinder",
              (new ProcessorPipe())
                  ->add(new LimiterFilter())
                  ->add(new CutFilter(FROM_ABOVE_VORONOI))
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
        );

    return pipesHolder;
}
