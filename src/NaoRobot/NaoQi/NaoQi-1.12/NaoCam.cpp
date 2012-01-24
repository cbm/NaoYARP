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


#include "NaoRobot/NaoCam.h"
#include "NaoRobot/ALBrokerWrapperClass.h"

#include <alcommon/albroker.h>
#include <alcommon/alproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alvisiondefinitions.h>

#include "Tools/logger.h"


NaoCam::NaoCam() : _isConfigured ( false ),
        _gvmName ( "" ),
        _imgWidth ( -1 ),
        _imgHeight ( -1 ) {

    try {
        _videoProxy = boost::shared_ptr<AL::ALVideoDeviceProxy> (
                          new AL::ALVideoDeviceProxy ( ALBrokerWrapper::Instance().GetBroker() ) );
    }
    catch ( AL::ALError& err ) {
        Logger::Instance().WriteMsg ( "NaoCam",
                                      "Error in getting VideoDevice proxy", Logger::FatalError );
        Logger::Instance().WriteMsg ( "NaoCam", err.toString(), Logger::FatalError );
    }

}



NaoCam::~NaoCam() {

    if ( _isConfigured ) {
        _videoProxy->unsubscribe ( _gvmName );
        _isConfigured = false;
    }

}



bool NaoCam::ConfigureVIM ( CamRes res, CamColSp clsp ) {

    if ( _isConfigured ) {
        _videoProxy->unsubscribe ( _gvmName );
        _isConfigured = false;
    }

    int pResolution;

    switch ( res ) {

    case yQQVGA:
        pResolution = AL::kQQVGA;
        break;

    case yQVGA:
        pResolution = AL::kQVGA;
        break;

    case yVGA:
        pResolution = AL::kVGA;
    };



    int pColorSpace;

    switch ( clsp ) {

    case yYUV422ColorSpace:
        pColorSpace = AL::kYUV422ColorSpace;
        break;

    case  yYUVColorSpace:
        pColorSpace = AL::kYUVColorSpace;
        break;

    case   yRGBColorSpace:
        pColorSpace = AL::kRGBColorSpace;
        break;

    case yHSYColorSpace:
        pColorSpace = AL::kHSYColorSpace;
        break;

    case yBGRColorSpace:
        pColorSpace = AL::kBGRColorSpace;
        break;

    case yYYCbCrColorSpace:
        pColorSpace = AL::kYYCbCrColorSpace;

    };

    
    AL::ALValue size = _videoProxy->resolutionToSizes(pResolution);
    
    _imgWidth = size[0];
    _imgHeight = size[1];

    if ( _imgWidth == -1 || _imgHeight == -1 )
        return false;

    _gvmName = _videoProxy->subscribe ( "yarpVideoProxy", pResolution, pColorSpace, 15 );

    _isConfigured = true;

    return true;
}



bool NaoCam::GetImage ( char** image, unsigned& size ) {

    if ( ! _isConfigured )
        return false;

    static AL::ALValue img;

    img.clear();

    img = _videoProxy->getImageRemote ( _gvmName );

    _imgWidth = ( int ) img[0];

    _imgHeight = ( int ) img[1];

    int nChannels = ( int ) img[2];

    size = _imgWidth * _imgHeight * nChannels;

    *image = ( char* ) ( img[6].GetBinary() );

    return true;
}



bool NaoCam::GetImageWidth ( int& width ) {

    if ( ! _isConfigured )
        return false;

    width = _imgWidth;

    return true;
}



bool NaoCam::GetImageHeight ( int& height ) {

    if ( ! _isConfigured )
        return false;

    height = _imgHeight;

    return true;
}



bool NaoCam::SetCamProp ( CamParam prop, double& value ) {
    int id;
    int val;
    ConvertToNaoQi ( prop, value, id, val );

    try {
        _videoProxy->setParam ( id, val );
    }
    catch ( ... ) {
        return false;
    }

    return true;
}



bool NaoCam::GetCamProp ( CamParam prop, double& value ) {
    int id;
    int val;
    ConvertToNaoQi ( prop, value, id, val );

    try {
        val = _videoProxy->getParam ( id );
        ConvertFromNaoQi ( prop, val, value );
    }
    catch ( ... ) {
        return false;
    }

    return true;
}



bool NaoCam::ConvertToNaoQi ( const CamParam in_param, const double& in_val,
                              int& out_id, int& out_val ) {

    switch ( in_param ) {

    case yCameraBrightnessID:
        out_id = AL::kCameraBrightnessID;
        out_val = in_val * 255;
        break;

    case yCameraSaturationID:
        out_id = AL::kCameraSaturationID;
        out_val = in_val * 255;
        break;

    case yCameraHueID:
        out_id = AL::kCameraHueID;
        out_val = ( in_val - 0.5 ) *  2  * 180;
        break;

    case yCameraRedChromaID:
        out_id = AL::kCameraRedChromaID;
        out_val = in_val * 255;
        break;

    case yCameraBlueChromaID:
        out_id = AL::kCameraBlueChromaID;
        out_val = in_val * 255;
        break;

    case yCameraGainID:
        out_id = AL::kCameraGainID;
        out_val = in_val * 255;
        break;

    case yCameraExposureID:
        out_id = AL::kCameraExposureID;
        out_val = in_val * 510;
        break;

    case yCameraSharpnessID:
        out_id = AL::kCameraSharpnessID;
        out_val = in_val * 31;
        break;

    default:
        return false;

    };

    return true;

}



bool NaoCam::ConvertFromNaoQi ( const CamParam in_param, const int& in_val, double& out_val ) {

    switch ( in_param ) {

    case yCameraBrightnessID:

    case yCameraSaturationID:

    case yCameraRedChromaID:

    case yCameraBlueChromaID:

    case yCameraGainID:
        out_val =  in_val / 255.0f;
        break;

    case yCameraHueID:
        out_val = ( in_val + 180 ) / 360.0f;
        break;

    case yCameraExposureID:
        out_val =  in_val / 510.0f;
        break;

    case yCameraSharpnessID:
        out_val =  in_val / 31.0f;
        break;

    default:
        return false;

    };

    return true;

}
