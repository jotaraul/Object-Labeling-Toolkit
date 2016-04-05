
#---------------------------------------------------------------------------#
#                         Object Labeling Toolkit                           #
#            A set of software components for the management and            #
#                      labeling of RGB-D datasets                           #
#                                                                           #
#            Copyright (C) 2015-2016 Jose Raul Ruiz Sarmiento               #
#                 University of Malaga <jotaraul@uma.es>                    #
#             MAPIR Group: <http://http://mapir.isa.uma.es/>                #
#                                                                           #
#   This program is free software: you can redistribute it and/or modify    #
#   it under the terms of the GNU General Public License as published by    #
#   the Free Software Foundation, either version 3 of the License, or       #
#   (at your option) any later version.                                     #
#                                                                           #
#   This program is distributed in the hope that it will be useful,         #
#   but WITHOUT ANY WARRANTY; without even the implied warranty of          #
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the            #
#   GNU General Public License for more details.                            #
#   <http://www.gnu.org/licenses/>                                          #
#                                                                           #
#---------------------------------------------------------------------------#

# Project name	
PROJECT(OLT)

# Required commands in newer CMake versions:
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Loads the current version number (e.g "0.5.1")
FILE(READ "${CMAKE_SOURCE_DIR}/version_prefix.txt" VERSION_NUMBER)

STRING(SUBSTRING "${VERSION_NUMBER}" 0 1 VERSION_NUMBER_MAJOR)
STRING(SUBSTRING "${VERSION_NUMBER}" 2 1 VERSION_NUMBER_MINOR)
STRING(SUBSTRING "${VERSION_NUMBER}" 4 1 VERSION_NUMBER_PATCH)

#------------------------------------------------------------------------------#
#                                 DEPENDENCIES
#------------------------------------------------------------------------------#

# --------------------------------------------
# MRPT library:
# --------------------------------------------

FIND_PACKAGE( MRPT REQUIRED slam;gui;hwdrivers;gui;vision;topography)

# --------------------------------------------
# PCL library:
# --------------------------------------------

find_package(PCL 1.7 REQUIRED)
IF (PCL_FOUND)
	INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
ENDIF(PCL_FOUND)

#message(PCL_LIBS: ${PCL_LIBRARIES})

# --------------------------------------------
# OpenCV
# --------------------------------------------

set(OLT_USING_OPENCV "FALSE" CACHE BOOL
  "Check if you want to use OpenCV at different parts of OLT (no mandatory).")

IF (OLT_USING_OPENCV)
	FIND_PACKAGE( OpenCV REQUIRED )
	add_definitions(-DUSING_OPENCV)
ENDIF (OLT_USING_OPENCV)

# --------------------------------------------
# Third party
# --------------------------------------------

# Difodo multi sensor
SET( DIFODO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/difodo_multi/ )

FILE(GLOB difodo_sources ${DIFODO_DIR}*.cpp)
FILE(GLOB difodo_headers ${DIFODO_DIR}*.h)

INCLUDE_DIRECTORIES(${DIFODO_DIR})	

ADD_LIBRARY(DIFODO ${difodo_sources} ${difodo_headers} )

# Tell CMake that the linker language is C++
SET_TARGET_PROPERTIES(DIFODO PROPERTIES LINKER_LANGUAGE CXX)

#------------------------------------------------------------------------------#
#                                  TARGET
#------------------------------------------------------------------------------#

# Create UPGM++ libraries and add include directories to the examples
SET( OLT_LIBRARIES "core;processing;mapping;labeling" )

SET(INC_DIR "")
SET(LIBRARIES "")

FOREACH( LIBRARY ${OLT_LIBRARIES} )
	ADD_SUBDIRECTORY(libs/${LIBRARY})
        INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/libs/${LIBRARY})
	SET(INC_DIR ${INC_DIR}${CMAKE_SOURCE_DIR}/libs/${LIBRARY}\;)
	SET(LIBRARIES ${LIBRARIES}${PROJECT_BINARY_DIR}/libs/libOLT-${LIBRARY}.so\;)
	set_target_properties(${LIBRARY} PROPERTIES PREFIX "libOLT-")
ENDFOREACH( LIBRARY ${OLT_LIBRARIES} )

# --------------------------------------------
# Compilation flags
# --------------------------------------------

set(OLT_USING_OMPENMP "FALSE" CACHE BOOL
  "Check if you want to parallelize some parts of the code using OpenMP.")

IF (OLT_USING_OMPENMP)
	IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -O3 -mtune=native -march=native ")
	ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
ELSE (OLT_USING_OMPENMP)
	IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native -march=native ")
	ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
ENDIF (OLT_USING_OMPENMP)


# The debug post-fix of .dll /.so libs
# ------------------------------------------
set(CMAKE_DEBUG_POSTFIX  "-dbg")

#------------------------------------------------------------------------------#
#                         Enable GCC profiling (GCC only)
#------------------------------------------------------------------------------#

#IF(CMAKE_COMPILER_IS_GNUCXX)
#	SET(ENABLE_PROFILING OFF CACHE BOOL "Enable profiling in the GCC compiler (Add flags: -g -pg)")
#ENDIF(CMAKE_COMPILER_IS_GNUCXX)

#IF(ENABLE_PROFILING)
#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -pg")
#ENDIF(ENABLE_PROFILING)

#IF(UNIX)
#	LINK_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}")
#ENDIF(UNIX)

#------------------------------------------------------------------------------#
#                     Using CLAMS intrinsic calibration?
#------------------------------------------------------------------------------#

set(OLT_USING_CLAMS_INTRINSIC_CALIBRATION "FALSE" CACHE BOOL
  "Check if an intrinsic calibration by CLAMS of the RGB-D sensors within the dataset is available.")

IF (OLT_USING_CLAMS_INTRINSIC_CALIBRATION)
	MESSAGE("Using CLAMS intrinsitc calibration of RGB-D sensors")
	add_definitions(-DUSING_CLAMS_INTRINSIC_CALIBRATION)

	#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/third_party/CLAMS/include)

	set(CLAMS_INCLUDE_DIR "" CACHE PATH
	  "Path to the CLAMS include directory")

	INCLUDE_DIRECTORIES(${CLAMS_INCLUDE_DIR})	

	set(CLAMS_DISCRETE_DEPTH_DISTORTION_MODEL_HEADER "" CACHE FILEPATH
	  "File called discrete_depth_distortion_model.h")

	set(CLAMS_DISCRETE_DEPTH_DISTORTION_MODEL_SOURCE "" CACHE FILEPATH
	  "File called discrete_depth_distortion_model.cpp")

	ADD_LIBRARY(Undistort 
		${CLAMS_DISCRETE_DEPTH_DISTORTION_MODEL_HEADER}
		${CLAMS_DISCRETE_DEPTH_DISTORTION_MODEL_SOURCE}
	)

	TARGET_LINK_LIBRARIES(Undistort libboost_system.so libboost_thread.so)

	# Tell CMake that the linker language is C++
	SET_TARGET_PROPERTIES(Undistort PROPERTIES LINKER_LANGUAGE CXX)

ELSE (OLT_USING_CLAMS_INTRINSIC_CALIBRATION)
	MESSAGE("Non using CLAMS intrisic calibration of RGB-D sensors")
ENDIF(OLT_USING_CLAMS_INTRINSIC_CALIBRATION)


#------------------------------------------------------------------------------#
#                           Toolkit-executables
#------------------------------------------------------------------------------#

ADD_EXECUTABLE(Process_rawlog ${CMAKE_SOURCE_DIR}/apps/process_rawlog.cpp)

IF (OLT_USING_CLAMS_INTRINSIC_CALIBRATION)
        TARGET_LINK_LIBRARIES(Process_rawlog ${PCL_LIBRARIES} ${MRPT_LIBS} Undistort core processing)
ELSE (OLT_USING_CLAMS_INTRINSIC_CALIBRATION)
        TARGET_LINK_LIBRARIES(Process_rawlog ${PCL_LIBRARIES} ${MRPT_LIBS} core processing)
ENDIF(OLT_USING_CLAMS_INTRINSIC_CALIBRATION)

ADD_EXECUTABLE(Mapping ${CMAKE_SOURCE_DIR}/apps/mapping.cpp)
TARGET_LINK_LIBRARIES(Mapping ${PCL_LIBRARIES} ${MRPT_LIBS} DIFODO )

ADD_EXECUTABLE(Visualize_reconstruction ${CMAKE_SOURCE_DIR}/apps/visualize_reconstruction.cpp)
TARGET_LINK_LIBRARIES(Visualize_reconstruction ${PCL_LIBRARIES} ${MRPT_LIBS} processing)

ADD_EXECUTABLE(Label_scene ${CMAKE_SOURCE_DIR}/apps/label_scene.cpp)
TARGET_LINK_LIBRARIES(Label_scene ${PCL_LIBRARIES} ${MRPT_LIBS} )

ADD_EXECUTABLE(Label_rawlog ${CMAKE_SOURCE_DIR}/apps/label_rawlog.cpp)
TARGET_LINK_LIBRARIES(Label_rawlog ${PCL_LIBRARIES} ${MRPT_LIBS} )

ADD_EXECUTABLE(Segmentation ${CMAKE_SOURCE_DIR}/apps/segmentation.cpp)
TARGET_LINK_LIBRARIES(Segmentation ${PCL_LIBRARIES} ${MRPT_LIBS} )

ADD_EXECUTABLE(Create_video ${CMAKE_SOURCE_DIR}/apps/create_video.cpp)
TARGET_LINK_LIBRARIES(Create_video ${PCL_LIBRARIES} ${MRPT_LIBS} )

ADD_EXECUTABLE(Dataset_statistics ${CMAKE_SOURCE_DIR}/apps/dataset_statistics.cpp)
TARGET_LINK_LIBRARIES(Dataset_statistics ${PCL_LIBRARIES} ${MRPT_LIBS} processing)

ADD_EXECUTABLE(Benchmark ${CMAKE_SOURCE_DIR}/apps/benchmark.cpp)
TARGET_LINK_LIBRARIES(Benchmark ${PCL_LIBRARIES} ${MRPT_LIBS} )

ADD_EXECUTABLE(Calibrate ${CMAKE_SOURCE_DIR}/apps/calibrate.cpp)
TARGET_LINK_LIBRARIES(Calibrate ${PCL_LIBRARIES} ${MRPT_LIBS} )


#------------------------------------------------------------------------------#
#                          Status messages
#------------------------------------------------------------------------------#

IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	MESSAGE(STATUS "Compiler flags: " ${CMAKE_CXX_FLAGS_RELEASE})
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")

