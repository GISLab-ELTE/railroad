/*
* BSD 3-Clause License
* Copyright (c) 2021-2023, Máté Cserép & Balázs Tábori
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
#include <boost/algorithm/string.hpp>

#include <lasdefinitions.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "helpers/LogHelper.h"
#include "helpers/LASHelper.h"

#include "filters/FragmentationFilter.h"

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
    std::string outputDirectory = fs::current_path().string();
    std::string algorithm;
    unsigned long maxSize;
    std::vector<long> boundaries;
    logging::trivial::severity_level logLevel;
    int fragmentationAngle;

    // Read console arguments
    po::options_description desc("Allowed options");
    desc.add_options()
        ("input,i", po::value<std::string>(&inputFile),
         "input file path")
        ("output,o", po::value<std::string>(&outputDirectory)->default_value(outputDirectory),
         "output directory path (default: ./)")
        ("size", po::value<unsigned long>(&maxSize)->default_value(ULONG_MAX),
         "maximum size of point cloud to process")
        ("algorithm,a", po::value<std::string>(&algorithm)->default_value("ThresholdContour"),
         "specify the filter to execute for fragmentation (ThresholdContour, CannyHough, GeneralizedHough)")
        ("angle", po::value<int>(&fragmentationAngle)->default_value(10),
         "fragmentation angle")
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
                  << "Copyright (c) 2018-2023, Máté Cserép & Péter Hudoba" << std::endl
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

    if (vm.count("boundaries") && boundaries.size() != 4) {
        std::cerr << "Give 4 boundary coordinates: minX, minY, maxX, maxY." << std::endl;
        argumentError = true;
    }

    if (fragmentationAngle <= 0) {
        std::cerr << "The fragmentation angle must be more than 0." << std::endl;
        argumentError = true;
    }

    // Define the algorithm
    FragmentationFilter *filter = nullptr;
    if (boost::algorithm::to_upper_copy(algorithm) ==
        boost::algorithm::to_upper_copy(std::string("ThresholdContour"))) {
        filter = new FragmentationFilter(THRESHOLD_CONTOUR);
    } else if (boost::algorithm::to_upper_copy(algorithm) ==
               boost::algorithm::to_upper_copy(std::string("CannyHough"))) {
        filter = new FragmentationFilter(CANNY_HOUGH);
    } else if (boost::algorithm::to_upper_copy(algorithm) ==
               boost::algorithm::to_upper_copy(std::string("GeneralizedHough"))) {
        filter = new FragmentationFilter(GENERALIZED_HOUGH);
    } else {
        std::cerr << "Unsupported fragmentation algorithm." << std::endl;
        argumentError = true;
    }

    if (argumentError) {
        std::cerr << "Use the --help option for description." << std::endl;
        return InvalidInput;
    }

    // Input handling
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
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

    // Create and change to output directory
    fs::create_directories(outputDirectory);
    fs::current_path(outputDirectory);

    // Processing
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> result;
    std::string name = filter->name();

    LOG(info) << "Starting " << name << " case";
    filter->setInputCloud(cloud);
    filter->setFragmentationAngle(fragmentationAngle);
    result = filter->execute();
    chdir("..");

    LOG(debug) << "Writing results";
    std::string outputFile;
    for (unsigned int i = 0; i < result.size(); i++) {
        outputFile = name + std::to_string(i) + ".laz";
        writeLAS(outputFile, outputHeader, result[i]);
    }
    LOG(info) << "Finished writing results";

    delete filter;

    return Success;
}
