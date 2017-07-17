# Nous voulons un cmake "récent" pour utiliser les dernières fonctionnalités
cmake_minimum_required(VERSION 3.0)

# Notre projet est étiqueté hello
project(SleepStageAlgo)

# Crée des variables avec les fichiers à compiler
set(SRCS
    main.cpp
    )
    
set(HEADERS
    )


# Include JsonCPP library
set(JSONCPP_LIB_PATH /home/lecoucl/Projects/Sommeil/SleepStageAlgorithm/jsoncpp)

include_directories(${JSONCPP_LIB_PATH}/include)

# Include Boost library 
FIND_PACKAGE( Boost 1.40 COMPONENTS date_time REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_executable(sleep_stage_algo ${SRCS} ${HEADERS})
target_link_libraries(sleep_stage_algo ${JSONCPP_LIB_PATH}/src/lib_json/libjsoncpp.so ${Boost_LIBRARIES})
