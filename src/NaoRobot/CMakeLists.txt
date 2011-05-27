
include_directories(./)

add_subdirectory(Generic)


set ( NaoEdition RoboCup CACHE STRING "The edition of the robot. Academic or Robocup" )

if ( "x${NaoEdition}x" STREQUAL "xx" )
  
    message ( FATAL_ERROR "NaoEdition can't be empty! Select an available platform: Academic or RoboCup" )
    
    
elseif ( "${NaoEdition}" STREQUAL "Academic" )


    add_subdirectory(Academic)

elseif ( "${NaoEdition}" STREQUAL "RoboCup" )
  
  
    add_subdirectory(RoboCup)


else ( "x${NaoEdition}x" STREQUAL "xx" )
  
    message ( FATAL_ERROR "Please select a valid platform: Academic or RoboCup"  )
    
endif ( "x${NaoEdition}x" STREQUAL "xx" )
