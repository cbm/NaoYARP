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


#include "DeviceManager.h"

#include "YarpJointDev.h"
#include "YarpCam.h"
#include "YarpAcc.h"
#include "YarpGyro.h"
#include "YarpFSR.h"
#include "YarpSonar.h"

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>


DeviceManager::DeviceManager() {


    _robotName = "/nao/";


    RegisterDevices();


    yarp::os::Network yarp;
    yarp::os::Property prop;


    // Instantiating Motor Devices

    prop.clear();
    prop.put ( "device", "controlboard" );
    prop.put ( "subdevice", "YarpJointDev" );
    prop.put ( "threadrate", 100 );



    prop.put ( "name", ( _robotName + "head" ).c_str() );
    prop.put ( "part", "Head" );
    _Devices.push_back (
        boost::shared_ptr<yarp::dev::PolyDriver> ( new yarp::dev::PolyDriver ( prop ) ) );



    prop.put ( "name", ( _robotName + "left_arm" ).c_str() );
    prop.put ( "part", "LArm" );
    _Devices.push_back (
        boost::shared_ptr<yarp::dev::PolyDriver> ( new yarp::dev::PolyDriver ( prop ) ) );



    prop.put ( "name", ( _robotName + "right_arm" ).c_str() );
    prop.put ( "part", "RArm" );
    _Devices.push_back (
        boost::shared_ptr<yarp::dev::PolyDriver> ( new yarp::dev::PolyDriver ( prop ) ) );



    prop.put ( "name", ( _robotName + "left_leg" ).c_str() );
    prop.put ( "part", "LLeg" );
    _Devices.push_back (
        boost::shared_ptr<yarp::dev::PolyDriver> ( new yarp::dev::PolyDriver ( prop ) ) );



    prop.put ( "name", ( _robotName + "right_leg" ).c_str() );
    prop.put ( "part", "RLeg" );
    _Devices.push_back (
        boost::shared_ptr<yarp::dev::PolyDriver> ( new yarp::dev::PolyDriver ( prop ) ) );






    prop.clear();
    prop.put ( "device", "grabber" );
    prop.put ( "subdevice", "YarpCam" );
    prop.put ( "name", ( _robotName + "cam" ).c_str() );
    _Devices.push_back (
        boost::shared_ptr<yarp::dev::PolyDriver> ( new yarp::dev::PolyDriver ( prop ) ) );

}



DeviceManager::~DeviceManager() {
    ;
}



void DeviceManager::RegisterDevices() {


    yarp::dev::Drivers::factory().add (
        new yarp::dev::DriverCreatorOf<YarpJointDev> ( "YarpJointDev", "controlboard", "YarpJointDev" ) );


    yarp::dev::Drivers::factory().add (
        new yarp::dev::DriverCreatorOf<YarpCam> ( "YarpCam", "grabber", "YarpCam" ) );


    yarp::dev::Drivers::factory().add (
        new yarp::dev::DriverCreatorOf<YarpAcc> ( "YarpAcc", "grabber", "YarpAcc" ) );


    yarp::dev::Drivers::factory().add (
        new yarp::dev::DriverCreatorOf<YarpGyro> ( "YarpGyro", "grabber", "YarpGyro" ) );


    yarp::dev::Drivers::factory().add (
        new yarp::dev::DriverCreatorOf<YarpFSR> ( "YarpFSR", "grabber", "YarpFSR" ) );


    yarp::dev::Drivers::factory().add (
        new yarp::dev::DriverCreatorOf<YarpSonar> ( "YarpSonar", "grabber", "YarpSonar" ) );

}
