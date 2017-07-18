# We want a recent CMake file
cmake_minimum_required(VERSION 3.0)

# The project
project(SleepStageAlgo)

# Create variables with files to be compiled
set(SRCS
    main.cpp
    SleepStageAlgorithm.cpp
    Features.cpp
    )
    
set(HEADERS
	SleepStageAlgorithm.hpp
	Features.hpp
	DateHelper.hpp
	DataSample.hpp
    )


# Include JsonCPP library
set(JSONCPP_LIB_PATH /home/lecoucl/Projects/Sommeil/SleepStageAlgorithm/jsoncpp)
include_directories(${JSONCPP_LIB_PATH}/include)

# Include Boost library 
FIND_PACKAGE( Boost 1.40 COMPONENTS date_time REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

# Include FFTW3 library
FIND_LIBRARY(FFTW3_LIBRARIES NAMES fftw3 libfftw3 PATHS /usr/lib REQUIRED)

add_executable(sleep_stage_algo ${SRCS} ${HEADERS})
target_link_libraries(sleep_stage_algo ${JSONCPP_LIB_PATH}/src/lib_json/libjsoncpp.so ${Boost_LIBRARIES} ${FFTW3_LIBRARIES})
