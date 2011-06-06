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


#ifndef NAOCAM_H
#define NAOCAM_H

#include <string>
#include <boost/shared_ptr.hpp>

namespace AL {

class ALVideoDeviceProxy;

}

class NaoCam {

public:


    NaoCam();

    virtual ~NaoCam();

    enum CamRes { yVGA = 0, yQVGA, yQQVGA };

    enum CamColSp { yYUV422ColorSpace = 0,
                    yYUVColorSpace ,
                    yRGBColorSpace,
                    yHSYColorSpace ,
                    yBGRColorSpace,
                    yYYCbCrColorSpace
                  };

    enum CamParam {  yCameraBrightnessID  = 0,
                     yCameraSaturationID,
                     yCameraHueID,
                     yCameraRedChromaID,
                     yCameraBlueChromaID,
                     yCameraGainID,
                     yCameraExposureID,
                     yCameraSharpnessID
                  };


    /**
     * Configure and register video module.
     * If the module is already registered, then
     * it is unregistered and re-registered with the new
     * properties.
     *
     * @param res Resolution of the video module.
     * @param clsp Color space to be used.
     *
     * @return true/false on success/failure.
     */

    bool ConfigureVIM ( CamRes& res, CamColSp& clsp );



    /**
     * Returns the width of the retrieved image.
     *
     * @param width Image width.
     * @return false if VIM is not configured.
     */

    bool GetImageWidth ( int& width);



    /**
     * Returns the height of the retrieved image.
     *
     * @param height Image Height.
     * @return false if VIM is not configured.
     */

    bool GetImageHeight (int& height);



    /**
     * Tries to set a property to the Camera. Values are normalized
     * in the range [0,1], with 1 indicating the maximum value.
     *
     * @param prop The property ID.
     * @param value Value of the property in [0,1].
     *
     * @return true/false on success/failure.
     */
    bool SetCamProp ( CamParam prop, double& value );



    /**
    * Tries to read a property from the Camera. Values are
    * normalized in the range [0,1], with 1 indicating
    * the maximum value.
    *
    * @param prop The property ID.
    * @param value Reserved space for the values
    * of the property in [0,1].
    *
    * @return true/false on success/failure.
    */

    bool GetCamProp ( CamParam prop, double& value );


private:


    /**
     * Function for convenient conversion to NaoQi camera
     * parameter IDs, and  parameter values from normalized
     * to absolute.
     *
     * @param in_param Internal camera property ID.
     * @param in_val Normalized value.
     * @param out_id NaoQi camera property ID.
     * @param out_val Absolute value.
     *
     * @return true/false on success/failure.
     */

    bool ConvertToNaoQi ( const CamParam in_param, const double& in_val,
                          int& out_id, int& out_val );

    /**
     * Function for convenient conversion from NaoQi camera
     * parameter  values from absolute to normalized.
     *
     * @param in_param Internal camera property ID.
     * @param in_val Absolute value.
     * @param out_val Normalized value.
     *
     * @return true/false on success/failure.
     */

    bool ConvertFromNaoQi ( const CamParam in_param, const int& in_val,
                           double& out_val );


    bool _isConfigured;
    std::string _gvmName;
    int _imgWidth;
    int _imgHeight;
    boost::shared_ptr<AL::ALVideoDeviceProxy> _videoProxy;

};

#endif // NAOCAM_H
