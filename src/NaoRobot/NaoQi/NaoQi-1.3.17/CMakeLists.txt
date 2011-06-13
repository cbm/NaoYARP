message ( STATUS "" )
message ( STATUS " _..::: Configuring for NaoQi RoboCup v1.3.17 :::.._ " )
message ( STATUS "" )



########################### BOOST INCLUDE
INCLUDE_DIRECTORIES(  ${BOOST_INCLUDE_DIR} )


############################ ENVIRONMENT VARIABLE AL_DIR
# Check that AL_DIR is set.
# Otherwise, try to set it - first by running findroot.cmake script; else retrieving the AL_DIR environment variable.
IF ( "x${AL_DIR}x" STREQUAL "xx"  )
  IF (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../findroot.cmake  )
        MESSAGE( STATUS "Could not find findroot.cmake in parent folder")
        IF( "x$ENV{AL_DIR}x" STREQUAL "xx" )
          MESSAGE( FATAL_ERROR "Please move your project to naoqi/modules/src directory or set the environment variable AL_DIR to the naoqi directory!" )
        ELSE( "x$ENV{AL_DIR}x" STREQUAL "xx" )
          SET( AL_DIR "$ENV{AL_DIR}" )
          MESSAGE( STATUS "Retrieve AL_DIR environment variable to set Naoqi folder")
        ENDIF ( "x$ENV{AL_DIR}x" STREQUAL "xx" )
  ELSE (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../findroot.cmake  )
        MESSAGE( STATUS "Run findroot.cmake to find Naoqi folder")
        INCLUDE ( ../findroot.cmake )
  ENDIF (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../findroot.cmake  )
  MESSAGE( STATUS "Naoqi folder is now set to ${AL_DIR}")
ENDIF("x${AL_DIR}x" STREQUAL "xx" )



############################ CMAKE PACKAGE DIRECTORY
# Set cmakeModules folder
SET( CMAKE_MODULE_PATH ${AL_DIR}/cmakemodules )




############################ ALDEBARAN CMAKE FILE
# Include aldebaran config file
# (platform management)

INCLUDE( "${CMAKE_MODULE_PATH}/aldebaran.cmake" )


############################ FIND PACKAGE
# Libraries specifics. Define here cmake packages
# usefull to your project

FIND_PACKAGE( ALCOMMON REQUIRED )
FIND_PACKAGE( TINYXML REQUIRED )
FIND_PACKAGE( PTHREAD REQUIRED )
FIND_PACKAGE( BOOST REQUIRED )





ADD_DEFINITIONS( ${ALCOMMON_DEFINITIONS} ${PTHREAD_DEFINITIONS} ${TINYXML_DEFINITIONS} ${BOOST_DEFINITIONS})

INCLUDE_DIRECTORIES( ${PROXIES_INCLUDE_DIR} ${ALCOMMON_INCLUDE_DIR} ${PTHREAD_INCLUDE_DIR}  ${TINYXML_INCLUDE_DIR} ${BOOST_INCLUDE_DIR})



add_library( LibNaoQi_1_3_17 SHARED NaoJoint.cpp NaoJointChain.cpp )

#use_lib(LibNaoRoboCup ALCOMMON)



TARGET_LINK_LIBRARIES( LibNaoRoboCup ${ALCOMMON_LIBRARIES} ${PTHREAD_LIBRARIES} ${NEWMAT_LIBRARIES}  ${TINYXML_LIBRARIES} ${BOOST_LIBRARIES})

add_subdirectory(naoyarp)
