cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap.cmake")
use(NAOQI-PLUGINS-TOOLS)

create_module( naoyarp )

SET ( NAOYARP_SRCS
  src/naoyarpmain.cpp
  src/naoyarpmodule.h
  src/naoyarpmodule.cpp

)

configure_src_module( naoyarp ${NAOYARP_SRCS} )

use_lib(naoyarp ALCOMMON)

target_link_libraries(naoyarp LibNaoRoboCup LibGeneric)

