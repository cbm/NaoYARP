/*
    NaoYARP: a YARP interface for Aldebaran's Nao robot.
    Copyright (C) 2011  Alexandros Paraschos <Alexandros.Paraschos@newport.ac.uk>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "NaoRobot/NaoJoint.h"
#include "NaoRobot/ALBrokerWrapperClass.h"

#include <alcommon/alproxy.h>
#include <alproxies/almotionproxy.h>

#include "Tools/logger.h"


NaoJoint::NaoJoint ( std::string name ) : _jointName ( name ) {

    try {
        _motion = ALBrokerWrapper::Instance().GetBroker()->getMotionProxy();
    }
    catch ( AL::ALError& err ) {
        Logger::Instance().WriteMsg ( "NaoJoint",
                                      "Error in getting Motion proxy", Logger::FatalError );
        Logger::Instance().WriteMsg ( "NaoJoint", err.toString(), Logger::FatalError );
    }

}



NaoJoint::~NaoJoint () {
    ;
}



float NaoJoint::GetCurrentAngle () {
    return _motion->getAngles ( _jointName, true ).at ( 0 );
}



void NaoJoint::GotoAngle ( float angle, float speed ) {

    _motion->post.angleInterpolationWithSpeed ( _jointName, angle, speed );

}



void NaoJoint::GotoAngleBlock ( float angle, float speed ) {

    _motion->angleInterpolationWithSpeed ( _jointName, angle, speed );

}



void NaoJoint::ChangeAngle ( float angle, float speed ) {

    _motion->changeAngles ( _jointName, angle, speed );

}



void NaoJoint::ChangeAngleBlock ( float angle, float speed ) {

    int id = _motion->post.changeAngles ( _jointName, angle, speed );

    _motion->wait ( id, 100 );

}



void NaoJoint::GetRange ( std::vector< float >& res ) {

    AL::ALValue ar = _motion->getLimits ( _jointName );
    res.push_back ( ar[0][0] );
    res.push_back ( ar[0][1] );

}



float NaoJoint::GetStiffness ( ) {
    return _motion->getStiffnesses ( _jointName ).at ( 0 );
}



void NaoJoint::SetStiffness ( float stiffness ) {
    _motion->stiffnessInterpolation ( _jointName, stiffness, 0.1f );
}
