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


#include "YarpJointDev.h"

#include <iostream>

#include <cmath>



YarpJointDev::YarpJointDev ()  {
    ;
}



YarpJointDev::~YarpJointDev () {
    ;
}



bool YarpJointDev::open ( yarp::os::Searchable& config ) {

    yarp::os::Value partName = config.find ( "part" );

    if ( partName.isNull() || !partName.isString() )
        return false;

    _chain = boost::shared_ptr<NaoJointChain> (
                 new NaoJointChain ( std::string ( partName.asString() ) ) );

    /// Initializing trajectory speeds to max speed.
    for ( unsigned i = 0; i < _chain->GetNumberOfJoints(); ++i )
        _refSpeeds.push_back ( 1.0f );

    _maxRefSpeed = 1.0f;

    return true;
}


bool YarpJointDev::close() {
    return true;
}


//from position

bool YarpJointDev::getAxes ( int* ax ) {

    *ax = _chain->GetNumberOfJoints();

    return true;
}



bool YarpJointDev::getRefSpeed ( int j, double* ref ) {

    try {
        *ref = _refSpeeds.at ( j );
    }
    catch ( ... ) {
        return false;
    }

    return true;
}



bool YarpJointDev::getRefSpeeds ( double* spds ) {

    std::vector<double>::const_iterator i;

    for ( i = _refSpeeds.begin(); i < _refSpeeds.end(); i++ )
        * ( spds++ ) = *i;

    return true;

}



bool YarpJointDev::positionMove ( int j, double ref ) {

    if ( j < 0 || ( unsigned int ) j >= _chain->GetNumberOfJoints() )
        return false;

    boost::shared_ptr<NaoJoint> jnt = _chain->GetSpecificJoint ( j );

    jnt->GotoAngleBlock ( ref, _refSpeeds.at ( j ) );

    return true;
}



bool YarpJointDev::positionMove ( const double* refs ) {

    std::vector<float> angles;

    for ( unsigned i = 0; i < _chain->GetNumberOfJoints(); ++i )
        angles.push_back ( * ( refs++ ) );

    _chain->GotoAngleBlock ( angles, _maxRefSpeed );

    return true;
}



bool YarpJointDev::relativeMove ( int j, double delta ) {

    if ( j < 0 || ( unsigned int ) j >= _chain->GetNumberOfJoints() )
        return false;

    boost::shared_ptr<NaoJoint> jnt = _chain->GetSpecificJoint ( j );

    jnt->ChangeAngleBlock ( delta, _refSpeeds.at ( j ) );

    return true;
}



bool YarpJointDev::relativeMove ( const double* deltas ) {

    std::vector<float> angles;

    for ( unsigned i = 0; i < _chain->GetNumberOfJoints(); ++i )
        angles.push_back ( * ( deltas++ ) );

    _chain->ChangeAngleBlock ( angles, _maxRefSpeed );

    return true;
}



inline double myMax ( double i, double j ) {
    return i > j ? i : j;
}



bool YarpJointDev::setRefSpeed ( int j, double sp ) {

    try {
        _refSpeeds.at ( j ) = sp;
    }
    catch ( ... ) {
        return false;
    }

    _maxRefSpeed = *std::max_element ( _refSpeeds.begin(), _refSpeeds.end(), myMax );

    return true;
}



bool YarpJointDev::setRefSpeeds ( const double* spds ) {
    std::vector<double>::iterator it;

    for ( it = _refSpeeds.begin(); it < _refSpeeds.end(); it++ )
        *it = * ( spds++ );

    return true;
}



//from control limits


bool YarpJointDev::getLimits ( int axis, double* min, double* max ) {

    if ( axis < 0 || ( unsigned int ) axis >= _chain->GetNumberOfJoints() )
        return false;

    std::vector<float> range;

    _chain->GetSpecificJoint ( axis )->GetRange ( range );

    *min = range.at ( 0 );

    *max = range.at ( 1 );

    return true;
}




//from encoders

bool YarpJointDev::getEncoder ( int j, double* v ) {

    if ( j < 0 || ( unsigned int ) j >= _chain->GetNumberOfJoints() )
        return false;

    *v = _chain->GetSpecificJoint ( j )->GetCurrentAngle();

    return true;
}



bool YarpJointDev::getEncoders ( double* encs ) {

    std::vector<float> angles = _chain->GetCurrentAngles();

    assert ( angles.size() == _chain->GetNumberOfJoints() );

    std::vector<float>::const_iterator it;

    for ( it = angles.begin(); it < angles.end(); it++ )
        * ( encs++ ) = *it;

    return true;
}




//from impendance

bool YarpJointDev::getCurrentImpedanceLimit ( int j, double* min_stiff, double* max_stiff, 
    										double* min_damp, double* max_damp ) {
    										
    *min_stiff = 0.0f;
    *max_stiff = 1.0f;
    *min_damp = 0.0f;
    *max_damp = 0.0f;
    
    return true;
    										
}



bool YarpJointDev::getImpedance ( int j, double* stiffness, double* damping  ) {

    if ( j < 0 || ( unsigned int ) j >= _chain->GetNumberOfJoints() )
        return false;

    *stiffness = _chain->GetSpecificJoint ( j )->GetStiffness();

    *damping = 0.0f; ///Not implemented - just set to zero;

    return true;
}



bool YarpJointDev::setImpedance ( int j, double stiffness, double  ) {

    if ( j < 0 || ( unsigned int ) j >= _chain->GetNumberOfJoints() )
        return false;

    _chain->GetSpecificJoint ( j )->SetStiffness ( stiffness );

    return true;
}




//from cartesian


bool YarpJointDev::getDOF ( yarp::sig::Vector& curDof ) {

    curDof.clear();

    for ( unsigned i = 0; i < _chain->GetNumberOfJoints(); ++i )
        curDof.push_back ( 1 );

    return true;

}



bool YarpJointDev::getPose ( yarp::sig::Vector& x, yarp::sig::Vector& od ) {

    std::vector<float> pos_or =  _chain->GetPotision();

    PosEulerSingle_To_PosAxisAngle ( pos_or, x, od );

    return true;
}



bool YarpJointDev::getTrackingMode ( bool* f ) {
    *f = true;
    return true;
}



bool YarpJointDev::getTrajTime ( double* t ) {
    *t = _trajTime;
    return true;
}



bool YarpJointDev::goToPose ( const yarp::sig::Vector& xd, const yarp::sig::Vector& od, const double t ) {

    std::vector<float> pos_or;
    PosAxisAngle_To_PosEulerSingle ( xd, od, pos_or );

    _chain->GotoPosition ( pos_or, t <= 0 ? _trajTime : t );

    return true;
}



bool YarpJointDev::goToPoseSync ( const yarp::sig::Vector& xd, const yarp::sig::Vector& od, const double t ) {

    std::vector<float> pos_or;
    PosAxisAngle_To_PosEulerSingle ( xd, od, pos_or );

    _chain->GotoPositionBlock ( pos_or, t <= 0 ? _trajTime : t );

    return true;
}



bool YarpJointDev::goToPosition ( const yarp::sig::Vector& xd, const double t ) {

    yarp::sig::Vector od;
    std::vector<float> pos_or;
    PosAxisAngle_To_PosEulerSingle ( xd, od, pos_or );

    _chain->GotoPosition ( pos_or, t <= 0 ? _trajTime : t, true );

    return true;

}



bool YarpJointDev::goToPositionSync ( const yarp::sig::Vector& xd, const double t ) {

    yarp::sig::Vector od;
    std::vector<float> pos_or;
    PosAxisAngle_To_PosEulerSingle ( xd, od, pos_or );

    _chain->GotoPositionBlock ( pos_or, t <= 0 ? _trajTime : t, true );

    return true;
}



bool YarpJointDev::restoreContext ( const int id ) {
    return true;
}



bool YarpJointDev::setPositionMode() {
    return true;
}



bool YarpJointDev::setTrajTime ( const double t ) {
    _trajTime = t;
    return true;
}



bool YarpJointDev::stopControl() {
    return true;
}



bool YarpJointDev::storeContext ( int* id ) {
    return true;
}



bool YarpJointDev::waitMotionDone ( const double period, const double timeout ) {
    return !_chain->WaitForPostedMotions ( timeout );
}



void YarpJointDev::AxisAngle_To_Euler ( const float& x, const float& y, const float& z, const float& theta,
                                        float& wx, float& wy, float& wz ) {

    /**
     * @note Source:
     * http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToEuler/index.htm
     */

    double heading;
    double attitude;
    double bank;

    double s = sin ( theta );
    double c = cos ( theta );
    double t = 1 - c;

    const double PI = 3.14159;

    //  if axis is not already normalised then uncomment this
    // double magnitude = Math.sqrt(x*x + y*y + z*z);
    // if (magnitude==0) throw error;
    // x /= magnitude;
    // y /= magnitude;
    // z /= magnitude;

    if ( ( x*y*t + z*s ) > 0.998 ) { // north pole singularity detected
        heading = 2 * atan2 ( x * sin ( theta / 2 ), cos ( theta / 2 ) );
        attitude = PI / 2;
        bank = 0;
        return;
    }

    if ( ( x*y*t + z*s ) < -0.998 ) { // south pole singularity detected
        heading = -2 * atan2 ( x * sin ( theta / 2 ), cos ( theta / 2 ) );
        attitude = -PI / 2;
        bank = 0;
        return;
    }

    heading = atan2 ( y * s - x * z * t , 1 - ( y * y + z * z ) * t );

    attitude = asin ( x * y * t + z * s ) ;
    bank = atan2 ( x * s - y * z * t , 1 - ( x * x + z * z ) * t );


    //TODO: validate
    wx = heading;
    wy = attitude;
    wz = bank;

}


void YarpJointDev::Euler_To_AxisAngle ( const float& wx, const float& wy, const float& wz,
                                        float& xa, float& ya, float& za, float& theta ) {


    /**
     * @note Source:
     * http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToAngle/index.htm
     */



    //TODO: validate
    double heading = wx;
    double attitude = wy;
    double bank = wz;



    double c1 = cos ( heading / 2 );
    double s1 = sin ( heading / 2 );
    double c2 = cos ( attitude / 2 );
    double s2 = sin ( attitude / 2 );
    double c3 = cos ( bank / 2 );
    double s3 = sin ( bank / 2 );
    double c1c2 = c1 * c2;
    double s1s2 = s1 * s2;
    double w = c1c2 * c3 - s1s2 * s3;
    xa = c1c2 * s3 + s1s2 * c3;
    ya = s1 * c2 * c3 + c1 * s2 * s3;
    za = c1 * s2 * c3 - s1 * c2 * s3;
    theta = 2 * acos ( w );
    double norm = xa * xa + ya * ya + za * za;

    if ( norm < 0.001 ) { // when all euler angles are zero angle =0 so
        // we can set axis to anything to avoid divide by zero
        xa = 1;
        ya = za = 0;
    }
    else {
        norm = sqrt ( norm );
        xa /= norm;
        ya /= norm;
        za /= norm;
    }

}



void YarpJointDev::PosAxisAngle_To_PosEulerSingle ( const yarp::sig::Vector& xd,
        const yarp::sig::Vector& od,
        std::vector< float >& res ) {


    assert ( xd.length() == 3 );
    assert ( od.length() == 4 );

    float wx, wy, wz;

    AxisAngle_To_Euler ( od[0], od[1], od[2], od[3], wx, wy, wz );

    res.clear();

    res.push_back ( xd[0] );
    res.push_back ( xd[1] );
    res.push_back ( xd[2] );

    res.push_back ( wx );
    res.push_back ( wy );
    res.push_back ( wz );


}



void YarpJointDev::PosEulerSingle_To_PosAxisAngle ( const std::vector< float >& pos,
        yarp::sig::Vector& xd,
        yarp::sig::Vector& od ) {

    assert ( pos.size() == 6 );

    xd.clear();
    od.clear();

    xd.push_back ( pos[0] );
    xd.push_back ( pos[1] );
    xd.push_back ( pos[2] );

    float xa, ya, za, theta;

    Euler_To_AxisAngle ( pos[3], pos[4], pos[5], xa, ya, za, theta );

    od.push_back ( xa );
    od.push_back ( ya );
    od.push_back ( za );
    od.push_back ( theta );

}
