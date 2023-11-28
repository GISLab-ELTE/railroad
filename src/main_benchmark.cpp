/*
 * BSD 3-Clause License
 * Copyright (c) 2018-2022, Máté Cserép & Péter Hudoba
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

#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "helpers/LogHelper.h"
#include "helpers/LASHelper.h"
#include "helpers/PCLHelper.h"
#include "helpers/BenchmarkHelper.h"

#include "piping/ProcessorPipeBunch.h"
#include "piping/ProcessorPipe.h"

#include "Results.h"
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
    std::vector<std::string> seedPaths;
    std::vector<std::string> seedTypes;
    std::string verifierFile;
    std::string outputDirectory = fs::current_path().string();
    unsigned long maxSize;
    bool autoShift;
    std::vector<long> boundaries;
    std::vector<std::string> pipes;
    logging::trivial::severity_level logLevel;

    // Read console arguments
    po::options_description desc("Allowed options");
    desc.add_options()
        ("input,i", po::value<std::string>(&inputFile),
         "input file path")
        ("seedpaths,sp", po::value<std::vector<std::string>>(&seedPaths)->multitoken(),
        "list of seed file paths (used by some algorithms)")
        ("seedtypes,st", po::value<std::vector<std::string>>(&seedTypes)->multitoken(),
        "list of seed types for seed file paths (rail, pole, cable, ties)")
        ("verify", po::value<std::string>(&verifierFile)->default_value(std::string()),
         "verifier file path")
        ("output,o", po::value<std::string>(&outputDirectory)->default_value(outputDirectory),
         "output directory path (default: ./)")
        ("size", po::value<unsigned long>(&maxSize)->default_value(ULONG_MAX),
         "maximum size of point cloud to process")
        ("algorithm,a", po::value<std::vector<std::string>>(&pipes)->multitoken(),
         "specify the algorithm pipes to execute (default: all)")
        ("shift,s", po::bool_switch(&autoShift),
         "shift point cloud origo during processing to avoid precision loss")
        ("boundaries", po::value<std::vector<long>>(&boundaries)->multitoken(),
         "minX, minY, maxX, maxY boundaries to process")
        ("usePCDAsCache", "create and use PCD file of LAZ as cache")
        ("loglevel", po::value<logging::trivial::severity_level>(&logLevel)->default_value(logging::trivial::info),
         "log level (trace, debug, info, warning, error, fatal)")
        ("help,h", "produce help message");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    LASAutoShift = autoShift; // set point cloud shift

    // Initialize the logger
    initLogger(logLevel);

    // Initialize seedHelper
    SeedHelper seedHelper(seedPaths, seedTypes);

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

    if(!seedHelper.seedsAreValid()){
        std::cerr << "Problem with supplied seeds, see above message for details." << std::endl;
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr verifier;
    LASheader outputHeader;
    LASheader verifierHeader;

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

    // Read new style seed file list
    if (seedHelper.getSeedFileCount() > 0) {
        seedHelper.loadSeedFiles();
    }

    // Read verifier file
    if (verifierFile.length() > 0) {
        verifier = readLAS(verifierFile, verifierHeader);
        LOG(info) << "Verifier file loaded, size: " << verifier->size();
    }

    // Generate processing algorithms
    ProcessorPipeBunch *pipesHolder = generateTestPipes();
    if (vm.count("algorithm")) {
        for (auto algorithm : pipes) {
            pipesHolder->addToRunOnlyList(algorithm);
        }
    }

    // Create and change to output directory
    fs::create_directories(outputDirectory);
    fs::current_path(outputDirectory);

    // Define result variables
    SingleResultCloudProcessor *algorithm;
    BenchmarkHelper benchmark;
    std::string resultFile;
    std::string visualFile;
    std::string csvFile;
    pcl::PointCloud<pcl::PointXYZ>::Ptr result;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> results;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> seedResult;
    pcl::PointCloud<pcl::PointXYZL>::Ptr visual;
    Results<double> timeAccuracyResults;
    Results<int> pointReductionResults;

    // Executing selected algorithm pipes
    for (auto pipeElement : pipesHolder->getPipes()) {
        LOG(info) << "Starting " << pipeElement.name << " case";
        mkdir(pipeElement.name.c_str(), 0777);
        chdir(pipeElement.name.c_str());

        int length = pipeElement.pipeVector.size() - 1;
        for (int i = 0; i < length+1; i++) {
            //Initalizing actual algorithm from pipeVector
            algorithm = pipeElement.pipeVector.at(length - i);

            //Always set base cloud
            algorithm->setBaseCloud(cloud);
            //Set seedHelper
            algorithm->setSeedHelper(seedHelper);

            //Chosing working cloud, in this condition the algorithm can choose pre-calculated result of "cloudIndex"th step to working cloud
            //We aren't using precalculated cloud in main algorithm OR this step is one of pre-calculation fases OR seed is given (no need to pre-calculate)
            if(pipeElement.cloudIndex == -1 || i != length || seedPaths.size() > 0)
                algorithm->setInputCloud(cloud);
            else
                algorithm->setInputCloud(seedResult.at(pipeElement.cloudIndex));

            if (i != 0 && seedHelper.getSeedFileCount() == 0) {
                seedHelper.addArgumentSeed(seedResult.at(i - 1));
            }

            //Algorithm running with checking (main algorithm or precalculation)
            if (length == i)
                result = algorithm->execute();
            else
                seedResult.push_back(algorithm->execute());
        }
        chdir("..");
        results=benchmark.noiseFilter(result);
        resultFile = pipeElement.name + ".laz";
        visualFile = pipeElement.name + "_visual.laz";
        csvFile = pipeElement.name + ".csv";
        writeLAS(resultFile, outputHeader, results.at(0));
        LOG(debug) << "Calculating visuals";
        visual = mergePointCloudsVisual(cloud, results.at(1), LASClass::LOW_POINT);
        visual = mergePointCloudsVisual(visual, results.at(0), pipeElement.classification);
        LOG(debug) << "Writing merged visual results";
        writeLAS(visualFile, outputHeader, visual);
        LOG(info) << "Finished writing results: " << resultFile << ", " << visualFile;

        LOG(debug) << "Writing error positions";
        if(length>0&&results.at(0)->size()>0)
            benchmark.writeCSV(csvFile, benchmark.errorPositions(results.at(0)));

        //Calculate time results of main algorithm
        auto pipeResults = pipeElement.pipeVector.at(0)->getTimeResults();
        if (verifierFile.length() > 0) {

            std::string falsePosFile = pipeElement.name + "_falsepos.laz";
            std::string falseNegFile = pipeElement.name + "_falseneg.laz";

            LOG(debug) << "Result size: " << result->size()
                       << ", Verifier size: " << verifier->size();
            pcl::PointCloud<pcl::PointXYZ>::Ptr falsePos = diffPointClouds(result, verifier, 3);
            int falsePositives = falsePos->size();
            double percentage = 100.0 * falsePositives / result->size();
            pipeResults["accuracy_false_positive"] = percentage;
            LOG(info) << "False positives: " << falsePositives
                      << " (" << percentage << "%)";

            writeLAS(falsePosFile, outputHeader, falsePos);

            pcl::PointCloud<pcl::PointXYZ>::Ptr falseNeg = diffPointClouds(verifier, result, 3);
            int falseNegatives = falseNeg->size();
            percentage = 100.0 * falseNegatives / verifier->size();
            pipeResults["accuracy_false_negative"] = percentage;
            LOG(info) << "False negatives: " << falseNegatives
                      << " (" << percentage << "%)";

            writeLAS(falseNegFile, outputHeader, falseNeg);
        }

        timeAccuracyResults.add(pipeElement.name, pipeResults);
        LOG(info) << "Time results:";
        for (auto const &x : pipeResults) {
            LOG(info) << x.first << ":" << x.second;
        }

        pointReductionResults.add(pipeElement.name, pipeElement.pipeVector.at(0)->getFilteredPointsResults());

        for(auto pipeVectorElement : pipeElement.pipeVector)
            delete pipeVectorElement;
    }

    // Save benchmark results
    LOG(info) << "time_accuracy_results.tsv: saving time and accuracy results";
    std::ofstream timeAccuracyResultsFile("time_accuracy_results.tsv");
    if (timeAccuracyResultsFile.is_open()) {
        timeAccuracyResultsFile << timeAccuracyResults;
        timeAccuracyResultsFile.close();
    }
    else {
        LOG(error) << "time_accuracy_results.tsv: failed to save";
    }

    LOG(info) << "point_reduction_results.tsv: saving point reduction results";
    std::ofstream pointReductionResultsFile("point_reduction_results.tsv");
    if (pointReductionResultsFile.is_open()) {
        pointReductionResultsFile << pointReductionResults;
        pointReductionResultsFile.close();
    }
    else {
        LOG(error) << "point_reduction_results.tsv: failed to save";
    }
    delete pipesHolder;
    return Success;
}
