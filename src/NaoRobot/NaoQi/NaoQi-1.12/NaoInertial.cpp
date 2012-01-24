/*
    NaoYARP: a YARP interface for Aldebaran's Nao robot.
    Copyright (C) 2011  Alexandros Paraschos <Alexandros.Paraschos@newport.ac.uk>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "NaoRobot/NaoInertial.h"

#include "NaoRobot/ALBrokerWrapperClass.h"

#include <alcommon/albroker.h>
#include <alcommon/alproxy.h>
#include <alproxies/almemoryproxy.h>

#include <iostream>

#include "Tools/logger.h"



NaoInertial::NaoInertial() {

    try {
        _memoryProxy =  ALBrokerWrapper::Instance().GetBroker()->getMemoryProxy();
    }
    catch ( AL::ALError& err ) {
        Logger::Instance().WriteMsg ( "NaoInertial",
                                      "Error in getting Memory proxy", Logger::FatalError );
        Logger::Instance().WriteMsg ( "NaoInertial", err.toString(), Logger::FatalError );
    }

    _accDevNameList.push_back ( "Device/SubDeviceList/InertialSensor/AccX/Sensor/Value" );

    _accDevNameList.push_back ( "Device/SubDeviceList/InertialSensor/AccY/Sensor/Value" );
    _accDevNameList.push_back ( "Device/SubDeviceList/InertialSensor/AccZ/Sensor/Value" );

    _gyrDevNameList.push_back ( "Device/SubDeviceList/InertialSensor/GyrX/Sensor/Value" );
    _gyrDevNameList.push_back ( "Device/SubDeviceList/InertialSensor/GyrY/Sensor/Value" );
    _gyrDevNameList.push_back ( "Device/SubDeviceList/InertialSensor/GyrRef/Sensor/Value" );

    _FSRDevNameList.push_back ( "Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value" );
    _FSRDevNameList.push_back ( "Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value" );
    _FSRDevNameList.push_back ( "Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value" );
    _FSRDevNameList.push_back ( "Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value" );

    _FSRDevNameList.push_back ( "Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value" );
    _FSRDevNameList.push_back ( "Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value" );
    _FSRDevNameList.push_back ( "Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value" );
    _FSRDevNameList.push_back ( "Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value" );

    _ultraSoundDevNameList.push_back ( "Device/SubDeviceList/US/Left/Sensor/Value" );
    _ultraSoundDevNameList.push_back ( "Device/SubDeviceList/US/Right/Sensor/Value" );

}


NaoInertial::~NaoInertial() {
    ;
}



std::vector< float > NaoInertial::GetAccValues() {

    static std::string msg = "Accelerometers";
    return GetDataWithCheck ( _accDevNameList, msg, _accDevNameList.size() );

}


std::vector< float > NaoInertial::GetFSRValues() {

    static std::string msg = "FSRs";
    return GetDataWithCheck ( _FSRDevNameList, msg, _FSRDevNameList.size() );

}

std::vector< float > NaoInertial::GetGyrValues() {

    static std::string msg = "Gyros";
    return GetDataWithCheck ( _gyrDevNameList, msg, _gyrDevNameList.size() );

}

std::vector< float > NaoInertial::GetUltraSoundValues() {

    static std::string msg = "Sonars";
    return GetDataWithCheck ( _ultraSoundDevNameList, msg, _ultraSoundDevNameList.size() );

}


std::vector< float > NaoInertial::GetDataWithCheck ( std::vector<std::string>& devList,
        std::string& msg,
        unsigned int num ) {

    std::vector< float > dat =  _memoryProxy->getListData ( devList );

    if ( dat.size() != num )
        std::cerr << "NaoInertial-" << msg << ": Warning only " << dat.size() <<
                  " out of " << num << " values were returned!" << std::endl;

    return dat;

}

