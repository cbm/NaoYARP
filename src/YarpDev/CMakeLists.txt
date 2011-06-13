
add_library ( LibYarpDev SHARED YarpJointDev.cpp
				YarpAcc.cpp
				YarpGyro.cpp
				YarpFSR.cpp
				YarpSonar.cpp
				YarpCam.cpp
				DeviceManager.cpp
)

target_link_libraries( LibYarpDev ${YARP_LIBRARIES} )
