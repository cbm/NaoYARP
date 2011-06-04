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


#ifndef NAOINERTIAL_H
#define NAOINERTIAL_H


#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>

namespace AL {

class ALMemoryProxy;

}

class NaoInertial {

public:

    NaoInertial();

    virtual ~NaoInertial();



    /**
     * Returns a vector containing accelerometer data
     * (3D vector: axis_x, axis_y, axis_z).
     *
     * @note If an error occur, less values may be returned.
     * A warning message will be printed.
     *
     * @return Vector of read data.
     */

    std::vector<float> GetAccValues();



    /**
     * Returns a vector containing gyroscope data
     * (3D vector: axis_x, axis_y, axis_z).
     *
     * @note If an error occur, less values may be returned.
     * A warning message will be printed.
     *
     * @return Vector of read data.
     */

    std::vector<float> GetGyrValues();



    /**
     * Returns a vector containing FSR data.
     *
     * 8D vector, in the following order:
     * 		LeftFoot  - Front - Left
     * 			 	  - Right
     * 			  - Rear  - Left
     * 				  - Right
     * 		RightFoot - Front - Left
     * 				  - Right
     * 			  - Rear  - Left
     * 				  - Right
     *
     * @note If an error occur, less values may be returned.
     * A warning message will be printed.
     *
     * @return Vector of read data.
     */

    std::vector<float> GetFSRValues();



    /**
     * Returns a vector containing ultra-sound data
     * (2D vector: left and right values).
     *
     * @note If an error occur, less values may be returned.
     * A warning message will be printed.
     *
     * @return Vector of read data.
     */

    std::vector<float> GetUltraSoundValues();



private:

    /**
     * Returns data retrieved from ALMemory.
     *
     * @param devList Devices to be read.
     * @param msg Warning msg in case that some data can't be retrieved.
     * @param num Expected number of data.
     *
     * @return Vector of read data.
     */

    std::vector<float> GetDataWithCheck ( std::vector<std::string>& devList,
                                          std::string& msg,
                                          unsigned num );





    boost::shared_ptr<AL::ALMemoryProxy> _memoryProxy;


    std::vector<std::string> _accDevNameList;

    std::vector<std::string> _gyrDevNameList;

    std::vector<std::string> _FSRDevNameList;

    std::vector<std::string> _ultraSoundDevNameList;

};

#endif // NAOINERTIAL_H
