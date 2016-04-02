
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


SET ( PROCESSING_LIB_NAME processing )

# TARGET:
FILE(GLOB aux_srcs1 ${CMAKE_SOURCE_DIR}/libs/${PROCESSING_LIB_NAME}/*.cpp)
FILE(GLOB aux_srcs2 ${CMAKE_SOURCE_DIR}/libs/${PROCESSING_LIB_NAME}/*.hpp)

SET(SRCS ${aux_srcs1} ${aux_srcs2})

ADD_LIBRARY(${PROCESSING_LIB_NAME} SHARED ${SRCS})

SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/libs)


TARGET_LINK_LIBRARIES(${PROCESSING_LIB_NAME} core ${MRPT_LIBS} ${OpenCV_LIBS})

install(TARGETS ${PROCESSING_LIB_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(FILES ${aux_srcs2} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/${PROCESSING_LIB_NAME}  )

