/*
 * BSD 3-Clause License
 * Copyright (c) 2022, Máté Cserép
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

#include "piping/ProcessorPipeBunch.h"
#include "piping/ProcessorPipe.h"

#include "Pipes.h"

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

int main(int argc, char *argv[])
{
    std::string inputFile;
    std::string seedFile;
    std::string outputDirectory = fs::current_path().string();
    std::string algorithmCable;
    std::string algorithmRail;
    unsigned long maxSize;
    std::vector<long> boundaries;
    logging::trivial::severity_level logLevel;

    // Read console arguments
    po::options_description desc("Allowed options");
    desc.add_options()
        ("input,i", po::value<std::string>(&inputFile),
         "input file path")
        ("seed", po::value<std::string>(&seedFile)->default_value(std::string()),
         "seed file path for cable detection (used by some algorithms)")
        ("output,o", po::value<std::string>(&outputDirectory)->default_value(outputDirectory),
         "output directory path (default: ./)")
        ("algorithm-cable", po::value<std::string>(&algorithmCable)->default_value("AngleAbove"),
         "specify the algorithm pipe to execute for cable detection")
        ("algorithm-rail", po::value<std::string>(&algorithmRail)->default_value("RailTrackWithSeed"),
         "specify the algorithm pipe to execute for rail track detection")
        ("size", po::value<unsigned long>(&maxSize)->default_value(ULONG_MAX),
         "maximum size of point cloud to process")
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
                  << "Copyright (c) 2018-2022, Máté Cserép & Péter Hudoba" << std::endl
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
        std::cerr << "The cable seed file does not exist." << std::endl;
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

    // Read cable seed file
    if (seedFile.length() > 0) {
        seed = readLAS(seedFile);
        LOG(info) << "Cable seed file loaded, size: " << seed->size();
    }

    // Generate processing algorithms
    ProcessorPipeBunch *pipesHolder = generateTestPipes();
    pipesHolder->addToRunOnlyList(algorithmCable);
    auto pipes = pipesHolder->getPipes();
    if (pipes.size() != 1 || pipes[0].classification != LASClass::CABLE) {
        std::cerr << "Cable detection algorithm not found.";
        return InvalidInput;
    }

    pipesHolder->addToRunOnlyList(algorithmRail);
    pipes = pipesHolder->getPipes();
    if (pipes.size() != 2 || pipes[1].classification != LASClass::RAIL) {
        std::cerr << "Rail track detection algorithm not found.";
        return InvalidInput;
    }

    // Create and change to output directory
    fs::create_directories(outputDirectory);
    fs::current_path(outputDirectory);

    // Define result variables
    CloudProcessor *algorithm;
    pcl::PointCloud<pcl::PointXYZ>::Ptr resultCable, resultRail;
    pcl::PointCloud<pcl::PointXYZL>::Ptr visual;

    // Executing selected cable detection pipe
    auto pipeElement = pipes[0];
    algorithm = pipeElement.pipe;
    algorithm->setInputCloud(cloud);
    LOG(info) << "Starting " << pipeElement.name << " case";
    if (seedFile.length() > 0)
        algorithm->setSeedCloud(seed);
    mkdir(pipeElement.name.c_str(), 0777);
    chdir(pipeElement.name.c_str());
    resultCable = algorithm->execute();
    chdir("..");
    LOG(debug) << "Calculating visuals";
    visual = mergePointCloudsVisual(cloud, resultCable, pipeElement.classification);
    delete algorithm;

    // Executing selected rail track detection pipe
    pipeElement = pipes[1];
    algorithm = pipeElement.pipe;
    algorithm->setInputCloud(cloud);
    LOG(info) << "Starting " << pipeElement.name << " case";
    algorithm->setSeedCloud(resultCable); // use cable result as seed for rail track detection
    mkdir(pipeElement.name.c_str(), 0777);
    chdir(pipeElement.name.c_str());
    resultRail = algorithm->execute();
    chdir("..");
    LOG(debug) << "Calculating visuals";
    visual = mergePointCloudsVisual(visual, resultRail, pipeElement.classification);
    delete algorithm;

    // Write results
    LOG(debug) << "Writing results";
    std::string outputFile = pipes[0].name + "-" + pipes[1].name + ".laz";
    writeLAS(outputFile, outputHeader, visual);
    LOG(info) << "Finished writing results: " << outputFile;

    delete pipesHolder;
    return Success;
}
