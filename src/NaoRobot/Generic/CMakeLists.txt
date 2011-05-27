
add_library ( LibGeneric SHARED YarpJointDev.cpp)

target_link_libraries( LibGeneric ${YARP_LIBRARIES} )
