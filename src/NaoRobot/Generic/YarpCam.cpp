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


#include "YarpCam.h"

YarpCam::YarpCam ( boost::shared_ptr<NaoCam> camera ) : _camera ( camera ) {
    ;
}

YarpCam::~YarpCam() {
    ;
}


bool YarpCam::configure ( yarp::os::Searchable& config ) {
    return yarp::os::IConfig::configure ( config );
}

bool YarpCam::open ( yarp::os::Searchable& config ) {
    return yarp::dev::DeviceDriver::open ( config );
}

bool YarpCam::close() {
    return yarp::dev::DeviceDriver::close();
}


bool YarpCam::getImage ( yarp::sig::ImageOf< yarp::sig::PixelRgb >& image ) {

}


int YarpCam::height() const {
    int res;
    _camera->GetImageHeight ( res );
    return res;
}

int YarpCam::width() const {
    int res;
    _camera->GetImageWidth ( res );
    return res;
}



double YarpCam::getBrightness() {
    double res = 0.0f;
    _camera->GetCamProp ( NaoCam::yCameraBrightnessID, res );
    return res;
}

double YarpCam::getExposure() {
    double res = 0.0f;
    _camera->GetCamProp ( NaoCam::yCameraExposureID, res );
    return res;
}


double YarpCam::getGain() {
    double res = 0.0f;
    _camera->GetCamProp ( NaoCam::yCameraGainID, res );
    return res;
}


double YarpCam::getHue() {
    double res = 0.0f;
    _camera->GetCamProp ( NaoCam::yCameraHueID, res );
    return res;
}


double YarpCam::getSaturation() {
    double res = 0.0f;
    _camera->GetCamProp ( NaoCam::yCameraSaturationID, res );
    return res;
}


double YarpCam::getSharpness() {
    double res = 0.0f;
    _camera->GetCamProp ( NaoCam::yCameraSharpnessID, res );
    return res;
}


bool YarpCam::getWhiteBalance ( double& blue, double& red ) {
    _camera->GetCamProp ( NaoCam::yCameraBlueChromaID, blue );
    _camera->GetCamProp ( NaoCam::yCameraRedChromaID, red );
}


bool YarpCam::setBrightness ( double v ) {
    return _camera->SetCamProp ( NaoCam::yCameraBrightnessID, v );
}


bool YarpCam::setExposure ( double v ) {
    return _camera->SetCamProp ( NaoCam::yCameraExposureID, v );
}


bool YarpCam::setGain ( double v ) {
    return _camera->SetCamProp ( NaoCam::yCameraGainID, v );
}


bool YarpCam::setHue ( double v ) {
    return _camera->SetCamProp ( NaoCam::yCameraHueID, v );
}


bool YarpCam::setSaturation ( double v ) {
    return _camera->SetCamProp ( NaoCam::yCameraSaturationID, v );
}


bool YarpCam::setSharpness ( double v ) {
    return _camera->SetCamProp ( NaoCam::yCameraSharpnessID, v );
}


bool YarpCam::setWhiteBalance ( double blue, double red ) {
    return _camera->SetCamProp ( NaoCam::yCameraBlueChromaID, blue ) &&
           _camera->SetCamProp ( NaoCam::yCameraRedChromaID, red );
}






















