
add_library ( LibGeneric SHARED YarpJointDev.cpp
				YarpAcc.cpp
				YarpGyro.cpp
				YarpFSR.cpp
				YarpSonar.cpp
				YarpCam.cpp
				DeviceManager.cpp
)

target_link_libraries( LibGeneric ${YARP_LIBRARIES} )
