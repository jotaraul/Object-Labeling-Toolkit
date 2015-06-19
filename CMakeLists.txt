
#---------------------------------------------------------------------------#
#                         Object Labeling Toolkit                           #
#            A set of software components for the management and            #
#                      labeling of RGB-D datasets                           #
#                                                                           #
#              Copyright (C) 2015 Jose Raul Ruiz Sarmiento                  #
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
PROJECT(HOMe-toolkit)

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

# --------------------------------------------
# MRPT library:
# --------------------------------------------

FIND_PACKAGE( MRPT REQUIRED slam;gui;hwdrivers;gui;vision;topography)

# --------------------------------------------
# PCL library:
# --------------------------------------------

find_package(PCL 1.7 REQUIRED)
if (PCL_FOUND)
	INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
endif(PCL_FOUND)

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
#                           Toolkit-executables
#------------------------------------------------------------------------------#

ADD_EXECUTABLE(Process_rawlog ${CMAKE_SOURCE_DIR}/src/process_rawlog.cpp)
TARGET_LINK_LIBRARIES(Process_rawlog ${PCL_LIBRARIES} ${MRPT_LIBS} )

ADD_EXECUTABLE(Localize_sensors ${CMAKE_SOURCE_DIR}/src/localization.cpp)
TARGET_LINK_LIBRARIES(Localize_sensors ${PCL_LIBRARIES} ${MRPT_LIBS} )

message(PCL_LIBS: ${PCL_LIBRARIES})

ADD_EXECUTABLE(Sequential_visualization ${CMAKE_SOURCE_DIR}/src/sequential_visualization.cpp)
TARGET_LINK_LIBRARIES(Sequential_visualization ${PCL_LIBRARIES} ${MRPT_LIBS} )

ADD_EXECUTABLE(Label_scene ${CMAKE_SOURCE_DIR}/src/label_scene.cpp)
TARGET_LINK_LIBRARIES(Label_scene ${PCL_LIBRARIES} ${MRPT_LIBS} )

ADD_EXECUTABLE(Label_rawlog ${CMAKE_SOURCE_DIR}/src/label_rawlog.cpp)
TARGET_LINK_LIBRARIES(Label_rawlog ${PCL_LIBRARIES} ${MRPT_LIBS} )

ADD_EXECUTABLE(Segmentation ${CMAKE_SOURCE_DIR}/src/segmentation.cpp)
TARGET_LINK_LIBRARIES(Segmentation ${PCL_LIBRARIES} ${MRPT_LIBS} )

ADD_EXECUTABLE(Create_video ${CMAKE_SOURCE_DIR}/src/create_video.cpp)
TARGET_LINK_LIBRARIES(Create_video ${PCL_LIBRARIES} ${MRPT_LIBS} )

ADD_EXECUTABLE(Dataset_statistics ${CMAKE_SOURCE_DIR}/src/dataset_statistics.cpp)
TARGET_LINK_LIBRARIES(Dataset_statistics ${PCL_LIBRARIES} ${MRPT_LIBS} )

#ADD_EXECUTABLE(Calibrate ${CMAKE_SOURCE_DIR}/src/calibrate.cpp)
#TARGET_LINK_LIBRARIES(Calibrate ${PCL_LIBRARIES} ${MRPT_LIBS} )
