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


#ifndef YARPACC_H
#define YARPACC_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/GenericSensorInterfaces.h>

#include "NaoRobot/NaoInertial.h"

class YarpAcc:
            public yarp::dev::DeviceDriver,
            public yarp::dev::IGenericSensor {

public:

    YarpAcc ();

    virtual ~YarpAcc();

    virtual bool open ( yarp::os::Searchable& config );

    virtual bool close();


    /**
     * Calibrate the sensor, single channel.
     *
     * @note Not implemented for Nao.
     *
     * @param ch channel number
     * @param v reset valure
     * @return always false.
     */

    virtual bool calibrate ( int ch, double v ) {
        return false;
    }




    /**
     * Get the number of channels of the sensor.
     * Three accelerometer channels are expected.
     *
     * @param nc pointer to storage, return value
     * @return true iff all channels are available.
     */

    virtual bool getChannels ( int* nc );



    /**
     * Read a vector from the sensor. If successive,
     * three values are stored (x,y,z), else returns false.
     *
     * @param out a vector containing the sensor's last readings.
     * @return true iff all channels are available.
     */

    virtual bool read ( yarp::sig::Vector& out );



private:

    boost::shared_ptr<NaoInertial> _inertial;
    const unsigned _expValues;

};

#endif // YARPACC_H
