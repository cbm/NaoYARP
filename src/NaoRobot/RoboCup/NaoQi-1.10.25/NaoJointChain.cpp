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

#include "NaoRobot/NaoJointChain.h"
#include "NaoRobot/ALBrokerWrapperClass.h"

#include <alcommon/alproxy.h>
#include <alproxies/almotionproxy.h>

#include <libalmotion/almotiondefinitions.h>
#include <almath/almath.h>




NaoJointChain::NaoJointChain ( std::string name ) : _chainName ( name ) {

    _motion = ALBrokerWrapper::Instance().GetBroker()->getMotionProxy();

    _jointNames = _motion->getJointNames ( _chainName );    
    
    std::vector<std::string>::const_iterator it;
    for ( it = _jointNames.begin(); it < _jointNames.end(); it++ )
      _joints.push_back( boost::shared_ptr<NaoJoint> (new NaoJoint (*it) ) );

}



NaoJointChain::~NaoJointChain () {
    ;
}



std::vector<float> NaoJointChain::GetCurrentAngles () {

    return _motion->getAngles ( _chainName, true );

}



unsigned NaoJointChain::GetNumberOfJoints () {
  
    return _jointNames.size();

}



boost::shared_ptr<NaoJoint> NaoJointChain::GetSpecificJoint ( unsigned i ) {
    
    return _joints.at(i);
    
}



void NaoJointChain::GotoAngle ( std::vector<float>& angle, float speed ) {

    _motion->post.angleInterpolationWithSpeed ( _chainName, angle, speed );

}



void NaoJointChain::GotoAngleBlock ( std::vector<float>& angle, float speed ) {

    _motion->angleInterpolationWithSpeed ( _chainName, angle, speed );

}



void NaoJointChain::ChangeAngle ( std::vector<float>& angle, float speed ) {

    _motion->changeAngles ( _chainName, angle, speed );

}



void NaoJointChain::ChangeAngleBlock ( std::vector<float>& angle, float speed ) {

    int id = _motion->post.changeAngles ( _chainName, angle, speed );

    _motion->wait ( id, 100 );

}



std::vector<float> NaoJointChain::GetStiffness ( ) {

    return _motion->getStiffnesses ( _chainName );

}




void NaoJointChain::SetStiffness ( std::vector<float>& stiffness ) {

    _motion->stiffnessInterpolation ( _chainName, stiffness, 0.1f );

}



std::vector<float> NaoJointChain::GetPotision ( ) {

    return _motion->getPosition ( _chainName, AL::Motion::SPACE_TORSO, true );

}



void NaoJointChain::GotoPosition ( std::vector<float>& pos, float t, bool ignoreOrientation ) {

    unsigned int mask = ignoreOrientation ? AL::Math::AXIS_MASK_ALL : AL::Math::AXIS_MASK_ALL;

    _motion->post.positionInterpolation ( _chainName, AL::Motion::SPACE_TORSO, pos, mask, t, true );

}



void NaoJointChain::GotoPositionBlock ( std::vector<float>& pos, float t, bool ignoreOrientation ) {

    unsigned int mask = ignoreOrientation ? AL::Math::AXIS_MASK_ALL : AL::Math::AXIS_MASK_ALL;

    _motion->positionInterpolation ( _chainName, AL::Motion::SPACE_TORSO, pos, mask, t, true );

}



bool NaoJointChain::WaitForPostedMotions ( float timeout ) {
  return false; ///TODO
}



boost::shared_ptr<NaoJoint> NaoJointChain::operator[] ( unsigned i ) {
  
  return this->GetSpecificJoint(i);
  
}

