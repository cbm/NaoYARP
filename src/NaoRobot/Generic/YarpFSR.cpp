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


#include "YarpFSR.h"

#include "Tools/StdVecToYarpVec.h"



YarpFSR::YarpFSR () : _expValues ( 8 ) {
    ;
}



YarpFSR::~YarpFSR() {
    ;
}



bool YarpFSR::open ( yarp::os::Searchable& config ) {

    _inertial = boost::shared_ptr<NaoInertial> ( new NaoInertial );

    return true;
}



bool YarpFSR::close() {
    return yarp::dev::DeviceDriver::close();
}



bool YarpFSR::getChannels ( int* nc ) {
    *nc = _inertial->GetFSRValues().size();
    return ( ( int ) *nc ) == _expValues;
}



bool YarpFSR::read ( yarp::sig::Vector& out ) {
    out.clear();

    out = Conv::StdVecToYarpVec<> ( _inertial->GetFSRValues() );

    return ( ( int ) out.size() ) == _expValues;
}
