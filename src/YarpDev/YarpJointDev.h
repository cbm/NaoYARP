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


#ifndef YARPJOINTDEV_H
#define YARPJOINTDEV_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>

#include "NaoRobot/NaoJointChain.h"
#include <boost/smart_ptr.hpp>


class YarpJointDev :
            public yarp::dev::DeviceDriver,
            public yarp::dev::IPositionControl,
            public yarp::dev::IControlLimits,
            public yarp::dev::IEncoders,
            public yarp::dev::IImpedanceControl,
            public yarp::dev::ICartesianControl {

public:

    YarpJointDev ();

    virtual ~YarpJointDev();

    virtual bool open ( yarp::os::Searchable& config );

    virtual bool close();

    /**@{ @name Methods inherit from  IPositionControl */

    /**
     * Check if the current trajectory of all is terminated.
     *
     * Non blocking. Not implemented for Nao robot.
     *
     * @param flag pointer to return value.
     * @return always false.
     */

    virtual bool checkMotionDone ( bool* flag ) {
        return false;
    }




    /**
     * Check if the current trajectory of a single axis is terminated.
     *
     * Non blocking. Not implemented for Nao robot.
     *
     * @param flag pointer to return value.
     * @return always false.
     */

    virtual bool checkMotionDone ( int j, bool* flag ) {
        return false;
    }




    /**
     * Get the number of controlled axes.
     *
     * @param ax pointer to store the number.
     * @return true, except if uninitialized
     */

    virtual bool getAxes ( int* ax );



    /**
     * Get reference acceleration of a joint. Returns the acceleration used to
     * generate the trajectory profile.
     *
     * Not implemented for Nao robot.
     *
     * @param j joint number
     * @param acc pointer to storage for the return value
     * @return always false.
     */

    virtual bool getRefAcceleration ( int j, double* acc ) {
        return false;
    }



    /**
     * Get reference acceleration of all joints. These are the values used during the
     * interpolation of the trajectory.
     *
     * Not implemented for Nao robot.
     *
     * @param accs pointer to the array that will store the acceleration values.
     * @return always false.
     */

    virtual bool getRefAccelerations ( double* accs ) {
        return false;
    }


    /**
     * Get reference speed for a joint. Returns the speed used to
     * generate the trajectory profile.
     *
     * @note Speed as a fraction of the max speed of the joint.
     * @todo Return speed in deg/s
     *
     *
     * @param j joint number
     * @param ref pointer to storage for the return value
     * @return true/false on success or failure     *
     */

    virtual bool getRefSpeed ( int j, double* ref );



    /**
     * Get reference speed on all joints. These values are used during the
     * interpolation of the trajectory.
     *
     * @note Speed as a fraction of the max speed of the joints.
     * @todo Return speeds in deg/s
     *
     * @param spds pointer to the array of speed values.
     * @return true/false upon success or failure
     */

    virtual bool getRefSpeeds ( double* spds );



    /**
     * Set new reference point for a single axis.
     *
     * Blocking call.
     *
     * @param j joint number.
     * @param ref specifies the new target point.
     * @return true iff  target is inside limits.
     */

    virtual bool positionMove ( int j, double ref );



    /**
     * Set new reference point for all axes.
     *
     * Blocking call.
     *
     * @param refs array of new reference points.
     * @return true iff  target is inside limits.
     */

    virtual bool positionMove ( const double* refs );



    /**
     * Set new reference point for a single axis,
     * relative to current position.
     *
     * Blocking call.
     *
     * @param j joint number.
     * @param delta specifies the new target point.
     * @return true iff  target is inside limits.
     */

    virtual bool relativeMove ( int j, double delta );



    /**
     * Set new reference point for a all axes,
     * relative to current position.
     *
     * Blocking call.
     *
     * @param deltas joint number.
     * @param delta array of new reference points.
     * @return true iff  target is inside limits.
     */

    virtual bool relativeMove ( const double* deltas );



    /**
     * Command is required by control boards implementing
     * different control methods.
     *
     * @return Always false.
     */

    virtual bool setPositionMode();



    /**
     * Set reference acceleration for a joint. This value is used during the
     * trajectory generation.
     *
     * Not implemented for Nao robot.
     *
     * @param j joint number
     * @param acc acceleration value
     * @return always false.
     */

    virtual bool setRefAcceleration ( int j, double acc ) {
        return false;
    }



    /**
     * Set reference acceleration on all joints. This is the value that is
     * used during the generation of the trajectory.
     *
     * Not implemented for Nao robot.
     *
     * @param accs pointer to the array of acceleration values
     * @return always false.
     */

    virtual bool setRefAccelerations ( const double* accs ) {
        return false;
    }



    /**
     * Set reference speed for a joint, this is the speed used during the
     * interpolation of the trajectory.
     *
     * @note Speed as a fraction of the max speed of the joint.
     * @todo Set speed in deg/s
     *
     * @param j joint number
     * @param sp speed value
     * @return true/false upon success/failure
     */

    virtual bool setRefSpeed ( int j, double sp );



    /**
     * Set reference speed on all joints. These values are used during the
     * interpolation of the trajectory.
     *
     * @note Speed as a fraction of the max speed of the joint.
     * @todo Set speed in deg/s
     *
     * @param spds pointer to the array of speed values.
     * @return true/false upon success/failure
     */

    virtual bool setRefSpeeds ( const double* spds );



    /**
     * Stop motion, all joints.
     *
     * Not implemented as all action calls are blocking.
     *
     * @todo Implement for cartesian space control?
     *
     * @return always false.
     */

    virtual bool stop() {
        return false;
    }



    /**
     * Stop motion, single joint.
     *
     * Not implemented as all action calls are blocking.
     *
     * @todo Implement for Cartesian space control?
     *
     * @param j joint number
     * @return always false.
     */

    virtual bool stop ( int j ) {
        return false;
    }


    /**@}*/



    /**@{ @name Methods inherit from IControlLimits */

    /**
     * Get the software limits for a particular axis.
     *
     * Limits are read from NaoQi on initialization, and thus can not be set.
     *
     * @param axis joint number (not regarded as there is only one axis).
     * @param pointer to store the value of the lower limit.
     * @param pointer to store the value of the upper limit.
     * @return always true.
     */

    virtual bool getLimits ( int axis, double* min, double* max );



    /**
     * Set the software limits for a particular axis.
     *
     * Limits are read from NaoQi on initialization, and thus can not be set.
     *
     * @todo Set software limits
     *
     * @param axis joint number (not regarded as there is only one axis).
     * @param min the value of the lower limit
     * @param max the value of the upper limit
     * @return always false.
     */

    virtual bool setLimits ( int axis, double min, double max ) {
        return false;
    }



    /**@}*/



    /**@{ @name Methods inherit from IEncoders */



    /// \todo what about getAxes from IEncoders?

    /**
     * Read the value of an encoder.
     *
     * @param j encoder number
     * @param v pointer to storage for the return value
     * @return true/false, upon success/failure (you knew it, uh?)
     */

    virtual bool getEncoder ( int j, double* v );



    /**
     * Read the position of all axes.
     *
     * @param encs pointer to the array that will contain the output
     * @return true/false on success/failure
     */

    virtual bool getEncoders ( double* encs );



    /**
     * Read the instantaneous acceleration of an axis.
     *
     * Not implemented for Nao robot.
     *
     * @param j axis number.
     * @param spds pointer to the array that will contain the output.
     * @return always false.
     */

    virtual bool getEncoderAcceleration ( int j, double* spds ) {
        return false;
    }



    /**
     * Read the instantaneous acceleration of all axes.
     *
     * Not implemented for Nao robot.
     *
     * @param accs pointer to the array that will contain the output.
     * @return always false.
     */

    virtual bool getEncoderAccelerations ( double* accs ) {
        return false;
    }



    /**
     * Read the instantaneous speed of an axis.
     *
     * Not implemented for Nao robot.
     *
     * @param j axis number
     * @param sp pointer to storage for the output
     * @return always false.
     */

    virtual bool getEncoderSpeed ( int j, double* sp ) {
        return false;
    }



    /**
     * Read the instantaneous speed of all axes.
     *
     * Not implemented for Nao robot.
     *
     * @param spds pointer to storage for the output values
     * @return always false.
     */

    virtual bool getEncoderSpeeds ( double* spds ) {
        return false;
    }



    /**
     * Reset encoder, single joint. Set the encoder value to zero.
     *
     * Not implemented for Nao robot.
     *
     * @param j encoder number
     * @return always false.
     */

    virtual bool resetEncoder ( int j ) {
        return false;
    }



    /**
     * Reset encoders. Set the encoders value to zero.
     *
     * Not implemented for Nao robot.
     *
     * @return always false.
     */

    virtual bool resetEncoders() {
        return false;
    }



    /**
     * Set the value of the encoder for a given joint.
     *
     * Not implemented for Nao robot.
     *
     * @param j encoder number.
     * @param val new value.
     * @return always false.
     */

    virtual bool setEncoder ( int j, double val ) {
        return false;
    }



    /**
     * Set the value of all encoders.
     *
     * Not implemented for Nao robot.
     *
     * @param vals pointer to the new values.
     * @return always false.
     */

    virtual bool setEncoders ( const double* vals ) {
        return false;
    }



    /**@}*/



    /**@{ @name Methods inherit from IImpedanceControl */



    //TODO what about getAxes from IImpedanceControl?


    /**
     * Get the current impedandance limits for a specific joint.
     *
     * @note Damping and offset parameters are not implemented for Nao.
     *
     * @param j joint number.
     * @param min_stiff pointer to store min stiffness value. (not really used, set to  0)
     * @param max_stiff pointer to store max stiffness value (not really used, set to 1).
     * @param min_damp pointer to store damping value (not used, set to  0).
     * @param max_damp pointer to store damping value (not used, set to 0).
     * @return true on success, otherwise false.
     */

    virtual bool getCurrentImpedanceLimit ( int j, double* min_stiff, double* max_stiff,
    										double* min_damp, double* max_damp );



    /**
     * Get current impedance gains (stiffness,damping,offset) for a specific joint.
     *
     * @note Damping and offset parameters are not implemented for Nao.
     *
     * @param j joint number.
     * @param stiffness pointer to store stiffness value.
     * @param damping pointer to store damping value (not used).
     * @param offset pointer to store offset value (not used).
     * @return true on success, otherwise false.
     */

    virtual bool getImpedance ( int j, double* stiffness, double* damping );



    /**
     * Get current force Offset for a specific joint.
     *
     * @note Offset parameter is not implemented for Nao.
     *
     * @param j joint number.
     * @param offset pointer to store offset value (not used).
     * @return always false.
     */

    virtual bool getImpedanceOffset ( int j, double* offset ) {
        return false;
    }



    /**
     * Set current impedance gains (stiffness,damping,offset) for a specific joint.
     *
     * @note Damping and offset parameters are not implemented for Nao.
     *
     * @param j joint number.
     * @param stiffness stiffness value.
     * @param damping damping value (not used).
     * @param offset offset value (not used).
     * @return true on success, otherwise false.
     */

    virtual bool setImpedance ( int j, double stiffness, double damping );



    /**
     * Set current force Offset for a specific joint.
     *
     * @note Offset parameter is not implemented for Nao.
     *
     * @param j joint number.
     * @param offset offset value (not used).
     * @return always false.
     */

    virtual bool setImpedanceOffset ( int j, double offset ) {
        return false;
    }



    /**@}*/



    /**@{ @name Methods inherit from ICartesianControl */



    /**
     * Ask for inverting a given pose without actually moving there.
     *
     * Not implemented for Nao.
     *
     * @param xd a 3-d vector which contains the desired
     *          position x,y,z (meters).
     * @param od a 4-d vector which contains the desired
     *          orientation using axis-angle representation xa, ya,
     *          za, theta (meters and radians).
     * @param xdhat a 3-d vector which is filled with the final
     *          position x,y,z (meters); it may differ from the
     *          commanded xd.
     * @param odhat a 4-d vector which is filled with the final
     *          orientation using axis-angle representation xa, ya,
     *          za, theta (meters and radians); it may differ from
     *          the commanded od.
     * @param qdhat the joints configuration through which the
     *             couple (xdhat,odhat) is achieved (degrees).
     * @return always false.
     */

    virtual bool askForPose ( const yarp::sig::Vector& xd,
                              const yarp::sig::Vector& od,
                              yarp::sig::Vector& xdhat,
                              yarp::sig::Vector& odhat,
                              yarp::sig::Vector& qdhat ) {
        return false;
    }



    /**
     * Ask for inverting a given pose without actually moving there.
     *
     * Not implemented for Nao.
     *
     * @param q0 a vector of length DOF which contains the starting
     *           joints configuration (degrees), made compatible with
     *           the chain.
     * @param xd a 3-d vector which contains the desired
     *          position x,y,z (meters).
     * @param od a 4-d vector which contains the desired
     *          orientation using axis-angle representation xa, ya,
     *          za, theta (meters and radians).
     * @param xdhat a 3-d vector which is filled with the final
     *          position x,y,z (meters); it may differ from the
     *          commanded xd.
     * @param odhat a 4-d vector which is filled with the final
     *          orientation using axis-angle representation xa, ya,
     *          za, theta (meters and radians); it may differ from
     *          the commanded od.
     * @param qdhat the joints configuration through which the
     *             couple (xdhat,odhat) is achieved (degrees).
     * @return always false.
     */

    virtual bool askForPose ( const yarp::sig::Vector& q0,
                              const yarp::sig::Vector& xd,
                              const yarp::sig::Vector& od,
                              yarp::sig::Vector& xdhat,
                              yarp::sig::Vector& odhat,
                              yarp::sig::Vector& qdhat ) {
        return false;
    }



    /**
     * Ask for inverting a given position without actually moving there.
     *
     * Not implemented for Nao.
     *
     * @param xd a 3-d vector which contains the desired
     *          position x,y,z (meters).
     * @param xdhat a 3-d vector which is filled with the final
     *          position x,y,z (meters); it may differ from the
     *          commanded xd.
     * @param odhat a 4-d vector which is filled with the final
     *          orientation using axis-angle representation xa, ya,
     *          za, theta (meters and radians); it may differ from
     *          the commanded od.
     * @param qdhat the joints configuration through which the
     *             couple (xdhat,odhat) is achieved (degrees).
     * @return always false.
     */

    virtual bool askForPosition ( const yarp::sig::Vector& q0,
                                  const yarp::sig::Vector& xd,
                                  yarp::sig::Vector& xdhat,
                                  yarp::sig::Vector& odhat,
                                  yarp::sig::Vector& qdhat ) {
        return false;
    }



    /**
     * Ask for inverting a given position without actually moving there.
     *
     * Not implemented for Nao.
     *
     * @param q0 a vector of length DOF which contains the starting
     *           joints configuration (degrees), made compatible with
     *           the chain.
     * @param xd a 3-d vector which contains the desired
     *          position x,y,z (meters).
     * @param xdhat a 3-d vector which is filled with the final
     *          position x,y,z (meters); it may differ from the
     *          commanded xd.
     * @param odhat a 4-d vector which is filled with the final
     *          orientation using axis-angle representation xa, ya,
     *          za, theta (meters and radians); it may differ from
     *          the commanded od.
     * @param qdhat the joints configuration through which the
     *             couple (xdhat,odhat) is achieved (degrees).
     * @return always false.
     */

    virtual bool askForPosition ( const yarp::sig::Vector& xd,
                                  yarp::sig::Vector& xdhat,
                                  yarp::sig::Vector& odhat,
                                  yarp::sig::Vector& qdhat ) {
        return false;
    }



    /**
     * Get the actual desired pose and joints configuration as result
     * of kinematic inversion.
     *
     * Blocking call.
     *
     * Not implemented for Nao.
     *
     * @param xdhat a 3-d vector which is filled with the actual
     *          desired position x,y,z (meters); it may differ from
     *          the commanded xd.
     * @param odhat a 4-d vector which is filled with the actual
     *          desired orientation using axis-angle representation
     *          xa, ya, za, theta (meters and radians); it may differ
     *          from the commanded od.
     * @param qdhat the joints configuration through which the
     *             couple (xdhat,odhat) is achieved (degrees).
     * @return always false.
     */

    virtual bool getDesired ( yarp::sig::Vector& xdhat,
                              yarp::sig::Vector& odhat,
                              yarp::sig::Vector& qdhat ) {
        return false;
    }



    /**
     * Get the current DOF configuration of the limb.
     *
     * @param curDof a vector which is filled with the actual DOF
     *           configuration.
     * @return true/false on success/failure.
     *
     * @note The vector lenght is equal to the number of limb's
     *       joints; each vector's position is filled with 1 if the
     *       associated joint is controlled (i.e. it is an actuated
     *       DOF), 0 otherwise.
     */

    virtual bool getDOF ( yarp::sig::Vector& curDof );



    /**
     * Return tolerance for in-target check.
     *
     * Not implemented on Nao robot.
     *
     * @param tol the memory location where tolerance is returned.
     * @return always false.
     *
     * @note The trajectory is supposed to be completed as soon as
     *       norm(xd-end_effector)<tol.
     */

    virtual bool getInTargetTol ( double* tol ) {
        return false;
    }



    /**
     * Return velocities of the end-effector in the task space.
     *
     * Not implemented on Nao robot.
     *
     *
     * @param xdot the 3-d vector containing the derivative of x,y,z
     *             position [m/s] of the end-effector while moving in
     *             the task space as result of the commanded joints
     *             velocities.
     * @param odot the 4-d vector containing the derivative of
     *             end-effector orientation [rad/s] while moving in
     *             the task space as result of the commanded joints
     *             velocities.
     * @return always false.
     */

    virtual bool getJointsVelocities ( yarp::sig::Vector& qdot ) {
        return false;
    }



//     /**
//      * Get the current range for the axis.
//      *
//      * @bug Redundant! Call IControlLimits::getLimits instead.
//      *
//      * @param axis joint index (regardless if it is actuated or
//      *            not).
//      * @param min where the minimum value is returned [deg].
//      * @param max where the maximum value is returned [deg].
//      * @return true/false on success/failure.
//      */

//     virtual bool getLimits ( const int axis, double* min, double* max ) {
//         return getLimits(const_cast<int>(axis), min, max );
//     }



    /**
     * Get the current pose of the end-effector.
     *
     * @param x a 3-d vector which is filled with the actual
     *         position x,y,z (meters).
     * @param od a 4-d vector which is filled with the actual
     * orientation using axis-angle representation xa, ya, za, theta
     * (meters and radians).
     * @param stamp is ignored.
     * @return true/false
     */

    virtual bool getPose ( yarp::sig::Vector& x, yarp::sig::Vector& od, yarp::os::Stamp *stamp=NULL);



    /**
     * Get the current pose of the specified link belonging to the
     * kinematic chain.
     *
     * Not implemented in Nao robot.
     *
     * @param axis joint index (regardless if it is actuated or not).
     * @param x a 3-d vector which is filled with the actual position
     *         x,y,z (meters) of the given link reference frame.
     * @param od a 4-d vector which is filled with the actual
     * orientation of the given link reference frame using axis-angle
     * representation xa, ya, za, theta (meters and radians).
     * @return always false.
     */

    virtual bool getPose ( const int axis, yarp::sig::Vector& x, yarp::sig::Vector& od, yarp::os::Stamp *stamp=NULL) {
        return false;
    }



    /**
     * Get the current joints rest position.
     *
     * Not implemented in Nao robot.
     *
     * @param curRestPos a vector which is filled with the current
     *                  joints rest position components in degrees.
     * @return always false.
     *
     * @note While solving the inverse kinematic, the user may
     *       specify a secondary task that minimizes against a joints
     *       rest position; further, each rest component may be
     *       weighted differently providing the weights vector.
     */

    virtual bool getRestPos ( yarp::sig::Vector& curRestPos ) {
        return false;
    }



    /**
     * Get the current joints rest weights.
     *
     * Not implemented in Nao robot.
     *
     * @param curRestWeights a vector which is filled with the
     *                  current joints rest weights.
     * @return always false.
     *
     * @note While solving the inverse kinematic, the user may
     *       specify a secondary task that minimizes against a joints
     *       rest position; further, each rest component may be
     *       weighted differently providing the weights vector.
     */

    virtual bool getRestWeights ( yarp::sig::Vector& curRestWeights ) {
        return false;
    }



    /**
     * Return joints velocities.
     *
     * Not implemented in Nao robot.
     *
     * @param qdot the vector containing the joints velocities
     *             [deg/s] sent to the robot by the controller.
     * @return always false..
     */

    virtual bool getTaskVelocities ( yarp::sig::Vector& xdot, yarp::sig::Vector& odot ) {
        return false;
    }



    /**
     * Get the current controller mode.
     *
     * @note Nao robot is always in, angle-space, tracking mode, via the Impedance
     * mechanism.
     *
     *
     * @param f is returned true (as the controller is always in tracking
     *         mode).
     * @return always true.
     */

    virtual bool getTrackingMode ( bool* f );



    /**
     * Get the current trajectory duration.
     *
     * @param t time (seconds).
     * @return true/false on success/failure.
     */

    virtual bool getTrajTime ( double* t );



    /**
     * Move the end-effector to a specified pose (position
     * and orientation) in cartesian space.
     *
     * Non blocking call.
     *
     * @param xd a 3-d vector which contains the desired position
     *           x,y,z
     * @param od a 4-d vector which contains the desired orientation
     * using axis-angle representation (xa, ya, za, theta).
     * @param t set the trajectory duration time (seconds). If t<=0
     *         (as by default) the current execution time is kept.
     * @return true/false on success/failure.
     *
     * @note Intended for streaming mode.
     */

    virtual bool goToPose ( const yarp::sig::Vector& xd, const yarp::sig::Vector& od, const double t = 0.0 );



    /**
     * Move the end-effector to a specified pose (position
     * and orientation) in cartesian space.
     *
     * Blocking call.
     *
     * @param xd a 3-d vector which contains the desired position
     *          x,y,z (meters).
     * @param od a 4-d vector which contains the desired orientation
     * using axis-angle representation (xa, ya, za, theta).
     * @param t set the trajectory duration time (seconds). If t<=0
     *         (as by default) the current execution time is kept.
     * @return true/false on success/failure.
     */

    virtual bool goToPoseSync ( const yarp::sig::Vector& xd, const yarp::sig::Vector& od, const double t = 0.0 );


    /**
     * Move the end-effector to a specified position in cartesian
     * space, ignore the orientation.
     *
     * Non blocking call.
     *
     * @param xd a 3-d vector which contains the desired position
     *          x,y,z (meters).
     * @param t set the trajectory duration time (seconds). If t<=0
     *         (as by default) the current execution time is kept.
     * @return true/false on success/failure.
     *
     * @note Intended for streaming mode.
     */

    virtual bool goToPosition ( const yarp::sig::Vector& xd, const double t = 0.0 );



    /**
     * Move the end-effector to a specified position in cartesian
     * space, ignore the orientation.
     *
     * Blocking call.
     *
     * @param xd a 3-d vector which contains the desired position
     *          x,y,z (meters).
     * @param t set the trajectory duration time (seconds). If t<=0
     *         (as by default) the current execution time is kept.
     * @return true/false on success/failure.
     */

    virtual bool goToPositionSync ( const yarp::sig::Vector& xd, const double t = 0.0 );



    /**
     * Restore the controller context previously stored.
     *
     * @param id specify the context id to be restored
     * @return true/false on success/failure.
     *
     * @note The context comprises the values of internal controller
     *       variables, such as the tracking mode, the active dofs,
     *       the trajectory time and so on.
     */

    virtual bool restoreContext ( const int id );



    /**
     * Set a new DOF configuration for the limb.
     *
     * Not implemented for Nao robot.
     *
     * @param newDof a vector which contains the new DOF
     *            configuration.
     * @param curDof a vector where the DOF configuration is
     *              returned as it has been processed after the
     *              request (it may differ from newDof due to the
     *              presence of some internal limb's constraints).
     * @return always false.
     *
     * @note Each vector's position shall contain 1 if the
     *       associated joint can be actuated, 0 otherwise. The
     *       special value 2 indicates that the joint status won't be
     *       modified (useful as a placeholder).
     */

    virtual bool setDOF ( const yarp::sig::Vector& newDof, yarp::sig::Vector& curDof ) {
        return false;
    }



    /**
     * Set tolerance for in-target check.
     *
     * Not implemented for Nao robot.
     *
     * @param tol tolerance.
     * @return always false.
     *
     * @note The trajectory is supposed to be completed as soon as
     *       norm(xd-end_effector)<tol.
     */

    virtual bool setInTargetTol ( const double tol ) {
        return false;
    }



//     /**
//      * Set new range for the axis. Allowed range shall be a valid
//      * subset of the real control limits.
//      *
//      * @bug Redundant! Call IControlLimits::getLimits instead.
//      *
//      * @param axis joint index (regardless it it is actuated or
//      *            not).
//      * @param min the new minimum value [deg].
//      * @param max the new maximum value [deg].
//      * @return true/false on success/failure.
//      */

//     virtual bool setLimits ( const int axis, const double min, const double max ) {
//         return setLimits( const_cast<int>(axis), const_cast<double>(min), const_cast<double>(max) );
//     }



    /**
     * Set a new joints rest position.
     *
     * Not implemented for Nao robot.
     *
     * @param newRestPos a vector which contains the new joints rest
     *                  position components in degrees.
     * @param curRestPos a vector which is filled with the current
     *           joints rest position components in degrees as result
     *           from thresholding with joints bounds.
     * @return always false.
     *
     * @note While solving the inverse kinematic, the user may
     *       specify a secondary task that minimizes against a joints
     *       rest position; further, each rest component may be
     *       weighted differently providing the weights vector.
     */

    virtual bool setRestPos ( const yarp::sig::Vector& newRestPos, yarp::sig::Vector& curRestPos ) {
        return false;
    }



    /**
     * Set a new joints rest position.
     *
     * Not implemented for Nao robot.
     *
     * @param newRestWeights a vector which contains the new joints
     *                  rest weights.
     * @param curRestWeights a vector which is filled with the
     *           current joints rest weights as result from
     *           saturation (w>=0.0).
     * @return always false.
     *
     * @note While solving the inverse kinematic, the user may
     *       specify a secondary task that minimizes against a joints
     *       rest position; further, each rest component may be
     *       weighted differently providing the weights vector.
     */

    virtual bool setRestWeights ( const yarp::sig::Vector& newRestWeights, yarp::sig::Vector& curRestWeights ) {
        return false;
    }



    /**
     * Set the reference velocities of the end-effector in the task space.
     *
     * Not implemented for Nao robot.
     *
     * @param xdot 	the 3-d vector containing the x,y,z reference velocities
     *              [m/s] of the end-effector.
     * @param odot 	the 4-d vector containing the orientation reference
     *           velocity [rad/s] of the end-effector.
     * @return always false.
     */

    virtual bool setTaskVelocities ( const yarp::sig::Vector& xdot, const yarp::sig::Vector& odot ) {
        return false;
    }



    /**
     * Set the controller in tracking or non-tracking mode.
     *
     * Nao's implementation supports only tracking mode!
     *
     * @param f true for tracking mode, false otherwise.
     * @return always false.
     *
     * @note In tracking mode when the controller reaches the target,
     *       it keeps on running in order to maintain the limb in the
     *       desired pose. In non-tracking mode the controller
     *       releases the limb as soon as the desired pose is
     *       reached.
     */

    virtual bool setTrackingMode ( const bool f ) {
        return false;
    }



    /**
     * Set the duration of the trajectory.
     *
     * @param t time in seconds.
     * @return always true.
     */

    virtual bool setTrajTime ( const double t );



    /**
     * Ask for an immediate stop motion.
     * @return true/false on success/failure.
     *
     * @note The control is completely released, i.e. a direct switch
     *       to non-tracking mode is executed.
     */

    virtual bool stopControl();



    /**
     * Store the controller context.
     *
     * @param id specify where to store the returned context id.
     * @return true/false on success/failure.
     *
     * @note The context comprises the values of internal controller
     *       variables, such as the tracking mode, the active dofs,
     *       the trajectory time and so on.
     */

    virtual bool storeContext ( int* id );



    /**
     * Wait until the current trajectory is terminated.
     *
     * @note In the current implementation the period is not
     * used, but instead is using the NaoQi's proxy wait method.
     *
     * @param period specify the check time period (seconds).
     * @param timeout specify the check expiration time (seconds). If
     *         timeout<=0 (as by default) the check will be performed
     *         without time limitation.
     * @return true for success, false for failure and timeout
     *         expired.
     */

    virtual bool waitMotionDone ( const double period = 0.1, const double timeout = 0.0 );



    /**@}*/


private:

    /**
     * Converts axis-angle representation for orientation, into Euler
     * angles.
     *
     * @param xa In axis-angle X-axis.
     * @param ya In axis-angle Y-axis.
     * @param za In axis-angle Z-axis.
     * @param theta In axis-angle Theta.
     *
     * @param wx Out Euler representation X rotation.
     * @param wy Out Euler representation Y rotation.
     * @param wz Out Euler representation Z rotation.
     *
     * @note Assuming the angles are in radians.
     */

    void AxisAngle_To_Euler ( const float& xa, const float& ya, const float& za,  const float& theta,
                              float& wx, float& wy, float& wz );



    /**
     * Converts Euler angles describing orientation, into the
     * Axis-angle representation.
     *
     * @param wx In Euler representation X rotation.
     * @param wy In Euler representation Y rotation.
     * @param wz In Euler representation Z rotation.
     *
     * @param xa Out axis-angle X-axis.
     * @param ya Out axis-angle Y-axis.
     * @param za Out axis-angle Z-axis.
     * @param theta Out axis-angle Theta.
     *
     * @note Assuming the angles are in radians.
     */

    void  Euler_To_AxisAngle ( const float& wx, const float& wy, const float& wz,
                               float& xa, float& ya, float& za, float& theta );



    /**
     * Convenient conversion between two YARP vectors describing
     * position and orientation to a single std vector containing both.
     *
     * The first representation is mostly used by the YARP Cartesian
     * interfaces, while the second one, is used by Aldebaran.
     *
     * @note Parameter od can be empty. In that case the result will still
     * be a 6D vector, but it will only contain position information. The rest
     * positions will be filled with 0's.
     *
     * @note Assuming the angles are in radians.
     *
     * @param xd Yarp 3D vector with the position.
     * @param od Yarp 4D vector with axis-angle representation (xa, ya, za, theta).
     * @param pos Std 6D vector with position and Euler orientation representation
     * 		(x,y,z,wx,wy,wz).
     *
     *
     */
    void PosAxisAngle_To_PosEulerSingle ( const yarp::sig::Vector& xd,
                                          const yarp::sig::Vector& od,
                                          std::vector<float>& pos );



    /**
     * Convenient conversion between two YARP vectors describing
     * position and orientation to a single std vector containing both.
     *
     * The first representation is mostly used by the YARP Cartesian
     * interfaces, while the second one, is used by Aldebaran.
     *
     * @note Assuming the angles are in radians.
     *
     * @param pos Std 6D vector with position and Euler orientation representation
     * 			(x,y,z,wx,wy,wz).
     * @param xd returns Yarp 3D vector with the position.
     * @param od returns Yarp 4D vector with axis-angle representation (xa, ya, za, theta).
     */

    void PosEulerSingle_To_PosAxisAngle ( const std::vector<float>& pos,
                                          yarp::sig::Vector& xd,
                                          yarp::sig::Vector& od );






    boost::shared_ptr<NaoJointChain> _chain;

    std::vector<double> _refSpeeds;
    double _maxRefSpeed; //used for chain angle-space ctl.
    double _trajTime;

    /**
    * Returns useful info on the operating state of the controller.
    * [wait for reply]
    * @param info is a property-like bottle containing the info.
    * @return always return false, FIXME!
    */
    virtual bool getInfo(yarp::os::Bottle &info){
        return false;
    }

    /**
    * Register an event.
    * @param event the event to be registered.
    * @return always return false, FIXME!
    *
    * @note the special type "*" can be used to attach a callback to
    *       all the available events.
    */
    virtual bool registerEvent(yarp::dev::CartesianEvent &event){
        return false;
    }

    /**
    * Unregister an event.
    * @param event the event to be unregistered.
    * @return @return always return false, FIXME!.
    */
    virtual bool unregisterEvent(yarp::dev::CartesianEvent &event){
        return false;
    }
};

#endif // YARPJOINTDEV_H

