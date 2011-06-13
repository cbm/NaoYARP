message ( STATUS "" )
message ( STATUS " _..::: Configuring for NaoQi RoboCup v1.10.25 :::.._ " )
message ( STATUS "" )

include( "./naoyarp/bootstrap.cmake" )


add_library( LibNaoQi_1_10_25 SHARED NaoJoint.cpp
				     NaoJointChain.cpp
				     NaoInertial.cpp
				     NaoCam.cpp
)

use_lib(LibNaoQi_1_10_25 ALCOMMON)

add_subdirectory(naoyarp)
