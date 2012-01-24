message ( STATUS "" )
message ( STATUS " _..::: Configuring for NaoQi RoboCup v1.12:::.._ " )
message ( STATUS "" )


add_library( LibNaoQi_1_12 SHARED NaoJoint.cpp
				     NaoJointChain.cpp
				     NaoInertial.cpp
				     NaoCam.cpp
)

qi_use_lib(LibNaoQi_1_12 ALCOMMON)

