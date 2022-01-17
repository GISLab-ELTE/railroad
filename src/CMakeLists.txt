cmake_minimum_required(VERSION 3.5)
project(railroad)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-pedantic -Wall -Wno-unused-result -Wno-strict-aliasing")
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX})

# Thread
find_package(Threads REQUIRED)
link_libraries(pthread)

# Boost
find_package(Boost REQUIRED COMPONENTS filesystem log program_options)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${Boost_LIBRARIES})

# LAStools
find_package(LASlib PATHS ../vendor/LAStools/install)
if(LASlib_FOUND)
    set(LASlib_LIBRARY_DIRS ${LASlib_INCLUDE_DIRS}/../../lib/LASlib/)
    set(LASlib_LIBRARIES LASlib)
else()
    message(SEND_ERROR "LASlib not found")
endif()
include_directories(${LASlib_INCLUDE_DIRS})
link_directories(${LASlib_LIBRARY_DIRS})
link_libraries(${LASlib_LIBRARIES})

# PCL
find_package(PCL 1.7 REQUIRED COMPONENTS common io filters segmentation visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_libraries(${PCL_LIBRARIES})

# OpenCV
find_package(OpenCV 4 REQUIRED)
link_libraries(${OpenCV_LIBS})

add_library(railroad SHARED
    base/CloudProcessor.cpp base/CloudProcessor.h
    filters/DensityFilter.cpp filters/DensityFilter.h
    filters/AboveFilter.cpp filters/AboveFilter.h
    filters/CylinderFilter.cpp filters/CylinderFilter.h
    filters/GroundFilter.cpp filters/GroundFilter.h
    filters/OutlierFilter.cpp filters/OutlierFilter.h
    filters/CutFilter.cpp filters/CutFilter.h
    filters/LimiterFilter.cpp filters/LimiterFilter.h
    filters/RansacFilter.cpp filters/RansacFilter.h
    filters/Hough3dFilter.cpp filters/Hough3dFilter.h
    filters/GrowthFilter.cpp filters/GrowthFilter.h
    filters/HeightFilter.cpp filters/HeightFilter.h
    filters/WidthFilter.cpp filters/WidthFilter.h
    filters/RailTrackFilter.cpp filters/RailTrackFilter.h
    dim2/ImportantPartFinderProcessor.cpp dim2/ImportantPartFinderProcessor.h
    dim2/Dim2Helper.cpp dim2/Dim2Helper.h
    dim2/TrackFinderProcessor.cpp dim2/TrackFinderProcessor.h
    dim2/Projection.cpp dim2/Projection.h
    helpers/PCLHelper.cpp helpers/PCLHelper.h
    helpers/LogHelper.cpp helpers/LogHelper.h
    helpers/LASHelper.cpp helpers/LASHelper.h helpers/LASClass.h
    helpers/HoughHelper.cpp helpers/HoughHelper.h
    piping/ProcessorPipe.cpp piping/ProcessorPipe.h
    piping/ProcessorPipeBunch.cpp piping/ProcessorPipeBunch.h)
link_libraries(railroad)

add_executable(railroad_benchmark
    main_benchmark.cpp
    Results.h Pipes.h)

add_executable(railroad_combined
    main_combined.cpp
    Results.h Pipes.h)

install(TARGETS railroad_benchmark railroad_combined railroad
    DESTINATION ${CMAKE_INSTALL_PREFIX})
