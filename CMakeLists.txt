cmake_minimum_required(VERSION 2.8)
project(NaoYARP)
set(NaoQiVersion "1.12" CACHE STRING "NaoQi Version is 1.12")
option(NaoYARP_IS_REMOTE "NaoYARP_IS_REMOTE" ON)

# You need this to find the QiBuild CMake framework
find_package(qibuild REQUIRED)
find_package(YARP REQUIRED)
include_directories(${YARP_INCLUDE_DIRS})
add_subdirectory(src)
include_directories(./src/)
option(NaoYARP_IS_REMOTE
    "module is compiled as a remote module (ON or OFF)"
    ON)
 
if(NaoYARP_IS_REMOTE)
    add_definitions( " -DNaoYARP_IS_REMOTE")
    qi_create_bin(naoyarpw "main.cpp" )
else()
    qi_create_lib(naoyarpw SHARED "main.cpp" )
endif()
 
target_link_libraries ( naoyarpw LibYarpDev LibNaoQi_1_12)
# LibYarpDev MUST before LibNaoQi_1_12!!
qi_use_lib(naoyarpw ALCOMMON BOOST)


