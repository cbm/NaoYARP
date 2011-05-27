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


#ifndef NAOJOINT_H
#define NAOJOINT_H

#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

namespace AL {

class ALMotionProxy;
}

class NaoJoint {

public:

    NaoJoint ( std::string name );

    virtual ~NaoJoint ();



    /**
     * Get the current angle of the joint,
     * using the value provided by the encoders.
     *
     * @return the angle of the joint in radians.
     *
     */

    float GetCurrentAngle ();



    /**
     * Set the joint to the desired angle.
     *
     * @param angle Joint angle in radians.
     * @param speed Fraction of max speed.
     *
     * @note Non-blocking call.
     *
     */

    void GotoAngle ( float angle, float speed );



    /**
     * Set the joint to the desired angle.
     *
     * @param angle Joint angle in radians.
     * @param speed Fraction of max speed.
     *
     * @note Blocking alternative.
     *
     */

    void GotoAngleBlock ( float angle, float speed );



    /**
     * Changes the joint angle from the current position.
     *
     * @param angle Joint angle in radians.
     * @param speed Fraction of max speed.
     *
     * @note Non-blocking call.
     *
     */

    void ChangeAngle ( float angle, float speed );




    /**
     * Changes the joint angle from the current position.
     *
     * @param angle Joint angle in radians.
     * @param speed Fraction of max speed.
     *
     * @note Blocking alternative.
     *
     */

    void ChangeAngleBlock ( float angle, float speed );



    /**
     * Returns the joint's min and max angles.
     *
     * @param res Vector for storing the returned values.
     * Values are in radians.
     *
     */

    void GetRange ( std::vector< float >& res );



    /**
     * Returns the joint's stiffness.
     *
     * @return The stiffness, with 1.0 indicating the max and
     * 0.0 the min.
     *
     */

    float GetStiffness ( );



    /**
     * Sets the stiffness to the joint.
     *
     * @param stiffness Stiffness, with 1.0 indicating max and 0.0 min.
     *
     */

    void SetStiffness ( float stiffness );



private:

    std::string _jointName;
    boost::shared_ptr<AL::ALMotionProxy> _motion;

};

#endif // NAOJOINT_H
