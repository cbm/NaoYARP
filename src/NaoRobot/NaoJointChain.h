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


#ifndef NAOJOINTCHAIN_H
#define NAOJOINTCHAIN_H

#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include "NaoJoint.h"

namespace AL {
  class ALMotionProxy;
}

class NaoJointChain {

public:

    NaoJointChain ( std::string name );

    virtual ~NaoJointChain ();
    
    
    /** 
     * Get the current angles of the joints in the chain, 
     * using the values provided by the encoders.
     *
     * @return A vector of angles in radians.
     * 
     */
    
    std::vector<float> GetCurrentAngles ();
    
    
    
    /**
     * Get number of joints participating in the chain.
     * 
     * @return Number of joints.
     * 
     */
    
    unsigned GetNumberOfJoints ();
    
    
    
    /**
     * Access a specific joint in the chain.
     * 
     * @param i The number of the joint.
     * @return An instance of NaoJoint.
     * 
     */
    
    boost::shared_ptr<NaoJoint> GetSpecificJoint ( unsigned i );
    
    
    
    /**
     * Set the joints in the chain to the desired angles.
     * 
     * @param angle Vector of Joint angle in radians.
     * @param speed Fraction of max speed.
     * 
     * @note Non-blocking call.
     * 
     */

    void GotoAngle ( std::vector<float>& angle, float speed );
    
    
    
    /**
     * Set the joints in the chain to the desired angles.
     * 
     * @param angle Vector of Joint angle in radians.
     * @param speed Fraction of max speed.
     * 
     * @note Blocking alternative.
     * 
     */

    void GotoAngleBlock ( std::vector<float>& angle, float speed );
    
    
    
    /**
     * Changes the joints angles from the current positions.
     * 
     * @param angle Vector of Joint angle in radians.
     * @param speed Fraction of max speed.
     * 
     * @note Non-blocking call.
     * 
     */
    
    void ChangeAngle ( std::vector<float>& angle, float speed );
    
    
    
    
    /**
     * Changes the joints angles from the current positions.
     * 
     * @param angle Vector of Joint angle in radians.
     * @param speed Fraction of max speed.
     * 
     * @note Blocking alternative.
     * 
     */
    
    void ChangeAngleBlock ( std::vector<float>& angle, float speed );
    
    
    
//     /**
//      * Returns the joint's min and max angles.
//      * 
//      * @param res Vector for storing the returned values.
//      * 
//      */
// 
//     void GetRange ( std::vector< float >& res);
    
    
    
    /**
     * Returns the joints stiffness.
     * 
     * @return Vector of stiffness, with 1.0 indicating the max and
     * 0.0 the min.
     * 
     */
    
    std::vector<float> GetStiffness ( );
    
    
    
    /**
     * Sets the stiffness to the joints in the chain.
     * 
     * @param stiffness Stiffness, with 1.0 indicating max and 0.0 min.
     * 
     */
    
    void SetStiffness ( std::vector<float>& stiffness );
    
    
    
    
    /**
     * Returns the position of the limb in Cartesian space. The center of the 
     * space is considered the center of the torso.
     * 
     * @note The method uses encoders data to calculate the position.
     * @return A vector containing the Position6D using meters 
     * and radians (x, y, z, wx, wy, wz) 
     */
    
    std::vector<float> GetPotision ( );
    
    
    
    /**
     * Moves the limb to the provided position and orientation, over time.
     * 
     * @note Non-blocking call.
     * 
     * @param pos Vector of 6D position arrays (x,y,z,wx,wy,wz) in meters 
     * and radians.
     * @param t Time duration in seconds.
     * @param ignoreOrientation True iff only position is taken into account.
     */
    
    void GotoPosition ( std::vector<float>& pos, float t, bool ignoreOrientation = false );
    
    
    
    /**
     * Moves the limb to the provided position and orientation, over time.
     * 
     * @note Blocking call.
     * 
     * @param pos Vector of 6D position arrays (x,y,z,wx,wy,wz) in meters 
     * and radians.
     * @param t Time duration in seconds.
     * @param ignoreOrientation True iff only position is taken into account.
     */
    
    void GotoPositionBlock ( std::vector<float>& pos, float t, bool ignoreOrientation = false );
    
    
    
    /**
     * Wait until all motions posted from that instance have finish their 
     * execution.
     * 
     * @param timeout Wait until timeout.
     * @return True iff timeout was reached.
     */
    
    bool WaitForPostedMotions ( float timeout );  
    
    
    
    /**
     * Access a specific joint in the chain.
     * 
     * @param i The number of the joint.
     * @return An instance of NaoJoint.
     * 
     */
    
    boost::shared_ptr<NaoJoint> operator[] ( unsigned i );
    
    
    
  private:

    std::string _chainName;    
    boost::shared_ptr<AL::ALMotionProxy> _motion;
    
    std::vector<std::string> _jointNames;
    
    std::vector<boost::shared_ptr<NaoJoint> > _joints;
    
};

#endif // NAOJOINTCHAIN_H
