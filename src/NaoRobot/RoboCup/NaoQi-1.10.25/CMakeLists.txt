message ( STATUS "" )
message ( STATUS " _..::: Configuring for NaoQi RoboCup v1.10.25 :::.._ " )
message ( STATUS "" )

include( "./naoyarp/bootstrap.cmake" )


add_library( LibNaoRoboCup SHARED NaoJoint.cpp
				  NaoJointChain.cpp
				  NaoInertial.cpp
				  NaoCam.cpp
)

use_lib(LibNaoRoboCup ALCOMMON)

add_subdirectory(naoyarp)
