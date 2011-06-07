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


#ifndef YARPCAM_H
#define YARPCAM_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/FrameGrabberInterfaces.h>

#include "NaoCam.h"

class YarpCam
            :
            public yarp::dev::DeviceDriver,
            public yarp::dev::IFrameGrabberImage,
            public yarp::dev::IFrameGrabberControls {



public:


    YarpCam(boost::shared_ptr<NaoCam> camera);

    ~YarpCam();

    virtual bool configure ( yarp::os::Searchable& config );

    virtual bool open ( yarp::os::Searchable& config );

    virtual bool close();



    /**
    * Get an rgb image from the frame grabber
    *
    * @param image the image to be filled
    * @return true/false upon success/failure
    */

    virtual bool getImage ( yarp::sig::ImageOf< yarp::sig::PixelRgb >& image );



    /**
    * Return the height of each frame.
    * @return image height
    */

    virtual int height() const;



    /**
    * Return the width of each frame.
    * @return image width
    */

    virtual int width() const;



    /**
     * Read the brightness parameter.
     * @return the current brightness value.
     */

    virtual double getBrightness();



    /**
     * Read the exposure parameter.
     * @return the current exposure value.
     */

    virtual double getExposure();



    /**
     * Read the gain parameter.
     * @return the current gain value.
     */

    virtual double getGain();



    /**
     * Read the gamma parameter.
     * @return Zero (0.0) as the method is not implemented.
     *
     * @note Method not implemented.
     */

    virtual double getGamma() {
        return 0.0f;
    }



    /**
     * Read the hue parameter.
     * @return the current hue value.
     */

    virtual double getHue();



    /**
     * Read the iris parameter.
     * @return Zero (0.0) as the method is not implemented.
     *
     * @note Method not implemented.
     */

    virtual double getIris() {
        return 0.0f;
    }



    /**
     * Read the saturation parameter.
     * @return the current saturation value.
     */

    virtual double getSaturation();



    /**
     * Read the sharpness parameter.
     * @return the current sharpness value.
     */

    virtual double getSharpness();



    /**
     * Read the shutter parameter.
     * @return Zero (0.0) as the method is not implemented.
     *
     * @note Method not implemented.
     */

    virtual double getShutter() {
        return 0.0f;
    }



    /**
     * Read the white balance parameters.
     * @param blue reference to return value for the blue parameter.
     * @param red reference to return value for the red parameter.
     * @return true/false.
     */

    virtual bool getWhiteBalance ( double& blue, double& red );



    /**
     * Set the brightness.
     * @param v new value for parameter.
     * @return true on success.
     */

    virtual bool setBrightness ( double v );



    /**
     * Set the exposure.
     * @param v new value for parameter.
     * @return true on success.
     */

    virtual bool setExposure ( double v );



    /**
     * Set the gain.
     * @param v new value for parameter.
     * @return true on success.
     */

    virtual bool setGain ( double v );



    /**
     * Set the gamma.
     * @param v new value for parameter.
     * @return always false.
     *
     * @note Method not implemented.
     */

    virtual bool setGamma ( double v ) {
        return false;
    }



    /**
     * Set the hue.
     * @param v new value for parameter.
     * @return true on success.
     */

    virtual bool setHue ( double v );



    /**
     * Set the iris.
     * @param v new value for parameter.
     * @return always false.
     *
     * @note Method not implemented.
     */

    virtual bool setIris ( double v ) {
        return false;
    }



    /**
     * Set the saturation.
     * @param v new value for parameter.
     * @return true on success.
     */

    virtual bool setSaturation ( double v );



    /**
     * Set the sharpness.
     * @param v new value for parameter.
     * @return true on success.
     */

    virtual bool setSharpness ( double v );



    /**
     * Set the shutter parameter.
     * @param v new value for parameter.
     * @return always false.
     *
     * @note Method not implemented.
     */

    virtual bool setShutter ( double v ) {
        return false;
    }



    /**
     * Set the white balance for the frame grabber.
     * @param blue component gain.
     * @param red component gain.
     * @return true/false if successful or not.
     */

    virtual bool setWhiteBalance ( double blue, double red );

private:

    boost::shared_ptr<NaoCam> _camera;



};

#endif // YARPCAM_H
