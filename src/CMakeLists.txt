cmake_minimum_required(VERSION 3.5)
project(railroad)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O2 -pedantic -Wall -Wno-unused-result -Wno-strict-aliasing")

# Thread
find_package(Threads REQUIRED)
link_libraries(pthread)

# Boost
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC ON)
find_package(Boost REQUIRED COMPONENTS filesystem log program_options)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${Boost_LIBRARIES})

# LAStools
include_directories(../vendor/LAStools/LASzip/src)
include_directories(../vendor/LAStools/LASlib/inc)
link_directories(../vendor/LAStools/LASlib/lib)
link_libraries(las)

# PCL
find_package(PCL 1.7 REQUIRED COMPONENTS common filters segmentation visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
link_libraries(${PCL_LIBRARIES})

# OpenCV
find_package(OpenCV REQUIRED)
link_libraries(${OpenCV_LIBS})

add_executable(railroad
    main.cpp
    filters/DensityFilter.cpp filters/DensityFilter.h
    filters/AboveFilter.cpp filters/AboveFilter.h
    filters/CylinderFilter.cpp filters/CylinderFilter.h
    filters/GroundFilter.cpp filters/GroundFilter.h
    filters/OutlierFilter.cpp filters/OutlierFilter.h
    filters/CutFilter.cpp filters/CutFilter.h
    filters/LimiterFilter.cpp filters/LimiterFilter.h
    filters/RailTrackFilter.cpp filters/RailTrackFilter.h
    helpers/PCLHelper.cpp helpers/PCLHelper.h
    helpers/LogHelper.cpp helpers/LogHelper.h
    helpers/LASHelper.cpp helpers/LASHelper.h
    dim2/ImportantPartFinderProcessor.cpp dim2/ImportantPartFinderProcessor.h
    dim2/Dim2Helper.cpp dim2/Dim2Helper.h
    dim2/TrackFinderProcessor.cpp dim2/TrackFinderProcessor.h
    dim2/Projection.cpp dim2/Projection.h
    piping/ProcessorPipe.cpp piping/ProcessorPipe.h
    piping/ProcessorPipeBunch.cpp piping/ProcessorPipeBunch.h
    base/CloudProcessor.h
    Results.h)
