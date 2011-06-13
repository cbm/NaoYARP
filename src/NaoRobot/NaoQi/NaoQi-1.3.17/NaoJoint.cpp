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

#include <alproxy.h>
#include <almotionproxy.h>
#include <../../naoqiclient/interface/almotionproxy_interface.h>



NaoJoint::NaoJoint ( std::string name ) : _jointName ( name ) {
    _motion = ALBrokerWrapper::Instance().GetBroker()->getMotionProxy();
}



NaoJoint::~NaoJoint () {
    ;
}



float NaoJoint::GetCurrentAngle () {
    return _motion->getAngle ( _jointName );
}



void NaoJoint::GotoAngle ( float angle, float speed ) {

    int perSpeed = speed * 100;
    _motion->post.gotoAngleWithSpeed ( _jointName, angle, perSpeed,
                                       AL::ALMotionProxyInterface::INTERPOLATION_LINEAR );

}



void NaoJoint::GotoAngleBlock ( float angle, float speed ) {

    int perSpeed = speed * 100;
    _motion->gotoAngleWithSpeed ( _jointName, angle, perSpeed,
                                  AL::ALMotionProxyInterface::INTERPOLATION_LINEAR );
}



void NaoJoint::ChangeAngle ( float angle, float speed ) {

    float curAngle = GetCurrentAngle();

    GotoAngle(curAngle+angle,speed);

}



void NaoJoint::ChangeAngleBlock ( float angle, float speed ) {

    float curAngle = GetCurrentAngle();

    GotoAngleBlock(curAngle+angle,speed);

}



void NaoJoint::GetRange ( std::vector< float >& res ) {

    std::vector< float > ar = _motion->getJointLimits ( _jointName );
    res.push_back ( ar[0] );
    res.push_back ( ar[1] );

}



float NaoJoint::GetStiffness ( ) {
    return _motion->getJointStiffness ( _jointName );
}



void NaoJoint::SetStiffness ( float stiffness ) {
    _motion->setJointStiffness ( _jointName, stiffness );
}
