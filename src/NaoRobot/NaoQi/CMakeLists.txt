message ( STATUS "" )
message ( STATUS " _..::: Configuring for Nao RoboCup Edition :::.._ " )
message ( STATUS "" )


set ( NaoQiVersion xx CACHE STRING "Version of underlying NaoQi" )

if ( "x${NaoQiVersion}x" STREQUAL "xx" )

    message ( FATAL_ERROR "NaoQiVersion must be set" )

else ( "x${NaoQiVersion}x" STREQUAL "xx" )


    add_subdirectory ( "NaoQi-${NaoQiVersion}" )


endif ( "x${NaoQiVersion}x" STREQUAL "xx" )
