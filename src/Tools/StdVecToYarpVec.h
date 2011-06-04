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


#ifndef STDVECTOYARPVEC_H
#define STDVECTOYARPVEC_H

#include <vector>
#include <yarp/sig/Vector.h>

namespace Conv {


template<typename T>
yarp::sig::Vector StdVecToYarpVec ( const std::vector<T>& in ) {

    yarp::sig::Vector out;

    std::vector<float>::const_iterator it;

    for ( it = in.begin(); it < in.end(); it++ )
        out.push_back ( *it );

    return out;

}

}

#endif // STDVECTOYARPVEC_H
