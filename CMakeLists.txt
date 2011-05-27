project(nao)

cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

message ( STATUS "" )
message ( STATUS " _..::: Configuring YARP interface for Nao robot :::.._ " )
message ( STATUS "" )



# find YARP
find_package(YARP REQUIRED)

# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS})



add_subdirectory(src)
