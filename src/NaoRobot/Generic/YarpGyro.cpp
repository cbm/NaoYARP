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


#include "YarpGyro.h"

#include "Tools/StdVecToYarpVec.h"



YarpGyro::YarpGyro () : _expValues ( 3 ) {
    ;
}



YarpGyro::~YarpGyro() {
    ;
}



bool YarpGyro::open ( yarp::os::Searchable& config ) {

    _inertial = boost::shared_ptr<NaoInertial> ( new NaoInertial );

    return true;
}



bool YarpGyro::close() {
    return yarp::dev::DeviceDriver::close();
}



bool YarpGyro::getChannels ( int* nc ) {
    *nc = _inertial->GetGyrValues().size();
    return ( ( int ) *nc ) == _expValues;
}



bool YarpGyro::read ( yarp::sig::Vector& out ) {

    out.clear();

    out = Conv::StdVecToYarpVec<> ( _inertial->GetGyrValues() );

    return ( ( int ) out.size() ) == _expValues;

}
