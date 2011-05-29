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

#include <alproxy.h>
#include <almotionproxy.h>
#include <../../naoqiclient/interface/almotionproxy_interface.h>




NaoJointChain::NaoJointChain ( std::string name ) : _chainName ( name ) {

    _motion = ALBrokerWrapper::Instance().GetBroker()->getMotionProxy();

    std::vector<std::string> allJointNames = _motion->getBodyJointNames ();

    bool isAcademic = allJointNames.size() > 22 ? true : false;

    std::vector<std::string>::const_iterator it = allJointNames.begin();

    /// Joint's order is clockwise, starting from the head

    const unsigned HeadIdx = 0;
    const unsigned HeadSize = 2;

    const unsigned LArmIdx = HeadIdx + HeadSize;
    const unsigned ArmSize = isAcademic ? 6 : 4;

    const unsigned LLegIdx = LArmIdx + ArmSize;
    const unsigned LegSize = 6;

    const unsigned RLegIdx = LLegIdx + LegSize;

    const unsigned RArmIdx = RLegIdx + LegSize;


    if ( _chainName == "Head" )
        _jointNames.assign ( it + HeadIdx, it + HeadIdx + HeadSize );
    else if ( _chainName == "LArm" )
        _jointNames.assign ( it + LArmIdx, it + LArmIdx + ArmSize );
    else if ( _chainName == "LLeg" )
        _jointNames.assign ( it + LLegIdx, it + LLegIdx + LegSize );
    else if ( _chainName == "RLeg" )
        _jointNames.assign ( it + RLegIdx, it + RLegIdx + LegSize );
    else if ( _chainName == "RArm" )
        _jointNames.assign ( it + RArmIdx, it + RArmIdx + ArmSize );
    else
        std::cerr << "Joint Chain " << *it << " not found!" << std::endl;



    for ( it = _jointNames.begin(); it < _jointNames.end(); it++ )
        _joints.push_back ( boost::shared_ptr<NaoJoint> ( new NaoJoint ( *it ) ) );

}



NaoJointChain::~NaoJointChain () {
    ;
}



std::vector<float> NaoJointChain::GetCurrentAngles () {

    return _motion->getChainAngles ( _chainName );

}



unsigned NaoJointChain::GetNumberOfJoints () {

    return _jointNames.size();

}



boost::shared_ptr<NaoJoint> NaoJointChain::GetSpecificJoint ( unsigned i ) {

    return _joints.at ( i );

}



void NaoJointChain::GotoAngle ( std::vector<float>& angle, float speed ) {

    int perSpeed = speed * 100;
    _motion->post.gotoChainAnglesWithSpeed ( _chainName, angle, speed,
            AL::ALMotionProxyInterface::INTERPOLATION_LINEAR );

}



void NaoJointChain::GotoAngleBlock ( std::vector<float>& angle, float speed ) {

    int perSpeed = speed * 100;
    _motion->gotoChainAnglesWithSpeed ( _chainName, angle, speed,
                                        AL::ALMotionProxyInterface::INTERPOLATION_LINEAR );

}




void NaoJointChain::ChangeAngle ( std::vector<float>& angle, float speed ) {

    std::vector<float> curAngles = GetCurrentAngles();

    assert ( curAngles.size() == angle.size() );

    for ( unsigned i = 0; i < curAngles.size(); ++i )
        curAngles[i] += angle[i];

    GotoAngle ( curAngles, speed );

}



void NaoJointChain::ChangeAngleBlock ( std::vector<float>& angle, float speed ) {

    std::vector<float> curAngles = GetCurrentAngles();

    assert ( curAngles.size() == angle.size() );

    for ( unsigned i = 0; i < curAngles.size(); ++i )
        curAngles[i] += angle[i];

    GotoAngleBlock ( curAngles, speed );

}

//
//
//
// std::vector<float> NaoJointChain::GetStiffness ( ) {
//
//     return _motion->getStiffnesses ( _chainName );
//
// }
//
//
//
//
// void NaoJointChain::SetStiffness ( std::vector<float>& stiffness ) {
//
//     _motion->stiffnessInterpolation ( _chainName, stiffness, 0.1f );
//
// }



std::vector<float> NaoJointChain::GetPotision ( ) {

    return _motion->getPosition ( _chainName,
                                  AL::ALMotionProxyInterface::SPACE_BODY );

}



void NaoJointChain::GotoPosition ( std::vector<float>& pos, float t, bool ignoreOrientation ) {

    unsigned int mask = ignoreOrientation ?
                        AL::ALMotionProxyInterface::AXIS_MASK_ALL : AL::ALMotionProxyInterface::AXIS_MASK_ALL;

    _motion->post.gotoPosition ( _chainName,
                                 AL::ALMotionProxyInterface::SPACE_BODY, pos, mask, t,
                                 AL::ALMotionProxyInterface::INTERPOLATION_LINEAR );

}



void NaoJointChain::GotoPositionBlock ( std::vector<float>& pos, float t, bool ignoreOrientation ) {

    unsigned int mask = ignoreOrientation ?
                        AL::ALMotionProxyInterface::AXIS_MASK_ALL : AL::ALMotionProxyInterface::AXIS_MASK_ALL;

    _motion->gotoPosition ( _chainName,
                            AL::ALMotionProxyInterface::SPACE_BODY, pos, mask, t,
                            AL::ALMotionProxyInterface::INTERPOLATION_LINEAR );

}



bool NaoJointChain::WaitForPostedMotions ( float timeout ) {
    return false; ///TODO
}



boost::shared_ptr<NaoJoint> NaoJointChain::operator[] ( unsigned i ) {

    return this->GetSpecificJoint ( i );

}

