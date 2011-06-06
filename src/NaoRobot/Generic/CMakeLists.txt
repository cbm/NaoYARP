
add_library ( LibGeneric SHARED YarpJointDev.cpp
				YarpAcc.cpp
				YarpGyro.cpp
				YarpFSR.cpp
				YarpSonar.cpp
				YarpCam.cpp
)

target_link_libraries( LibGeneric ${YARP_LIBRARIES} )
