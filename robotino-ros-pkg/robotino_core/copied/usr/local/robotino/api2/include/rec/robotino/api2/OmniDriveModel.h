//  Copyright (C) 2004-2013, Robotics Equipment Corporation GmbH

//Copyright (c) ...
//
//REC Robotics Equipment Corporation GmbH, Planegg, Germany. All rights reserved.
//Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
//1) Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
//2) Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
//
//THIS SOFTWARE IS PROVIDED BY REC ROBOTICS EQUIPMENT CORPORATION GMBH �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REC ROBOTICS EQUIPMENT CORPORATION GMBH
//BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
//GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//Copyright (c) ...
//
//REC Robotics Equipment Corporation GmbH, Planegg, Germany. Alle Rechte vorbehalten.
//Weiterverbreitung und Verwendung in nichtkompilierter oder kompilierter Form, mit oder ohne Ver�nderung, sind unter den folgenden Bedingungen zul�ssig:
//1) Weiterverbreitete nichtkompilierte Exemplare m�ssen das obige Copyright, diese Liste der Bedingungen und den folgenden Haftungsausschluss im Quelltext enthalten.
//2) Weiterverbreitete kompilierte Exemplare m�ssen das obige Copyright, diese Liste der Bedingungen und den folgenden Haftungsausschluss in der Dokumentation und/oder anderen Materialien, die mit dem Exemplar verbreitet werden, enthalten.
//
//DIESE SOFTWARE WIRD VON REC ROBOTICS EQUIPMENT CORPORATION GMBH OHNE JEGLICHE SPEZIELLE ODER IMPLIZIERTE GARANTIEN ZUR VERF�GUNG GESTELLT, DIE UNTER
//ANDEREM EINSCHLIESSEN: DIE IMPLIZIERTE GARANTIE DER VERWENDBARKEIT DER SOFTWARE F�R EINEN BESTIMMTEN ZWECK. AUF KEINEN FALL IST REC ROBOTICS EQUIPMENT CORPORATION GMBH
//F�R IRGENDWELCHE DIREKTEN, INDIREKTEN, ZUF�LLIGEN, SPEZIELLEN, BEISPIELHAFTEN ODER FOLGESCH�DEN (UNTER ANDEREM VERSCHAFFEN VON ERSATZG�TERN ODER -DIENSTLEISTUNGEN;
//EINSCHR�NKUNG DER NUTZUNGSF�HIGKEIT; VERLUST VON NUTZUNGSF�HIGKEIT; DATEN; PROFIT ODER GESCH�FTSUNTERBRECHUNG), WIE AUCH IMMER VERURSACHT UND UNTER WELCHER VERPFLICHTUNG
//AUCH IMMER, OB IN VERTRAG, STRIKTER VERPFLICHTUNG ODER UNERLAUBTER HANDLUNG (INKLUSIVE FAHRL�SSIGKEIT) VERANTWORTLICH, AUF WELCHEM WEG SIE AUCH IMMER DURCH DIE BENUTZUNG
//DIESER SOFTWARE ENTSTANDEN SIND, SOGAR, WENN SIE AUF DIE M�GLICHKEIT EINES SOLCHEN SCHADENS HINGEWIESEN WORDEN SIND.

#ifndef _REC_ROBOTINO_API2_OMNIDRIVEMODEL_H_
#define _REC_ROBOTINO_API2_OMNIDRIVEMODEL_H_

#include "rec/robotino/api2/utils.h"
#include <cmath>

namespace rec
{
	namespace robotino
	{
		namespace api2
		{
			/**
			* @brief	Project single motor speeds to velocity vector and vice versa.
			* 
			*/
			class OmniDriveModel
			{
			public:
				OmniDriveModel()
					: _rb( 0.132 )
					, _rw( 0.040 )
					, _gear( 16.0 )
				{
				}

				/**
				 * Sets the distance from robot center to wheel center.
				 * @param rb Distance from robot center to wheel center in m.
				 */
				void setRb( double rb ) { _rb = rb; }

				/**
				 * Sets the radius of the wheels.
				 * @param rw Radius of the wheels in m.
				 */
				void setRw( double rw ) { _rw = rw; }

				/**
				 * Sets the gear.
				 * @param gear Gear
				 */
				void setGear( double gear ) { _gear = gear; }

				/** @return Distance from robot center to wheel center in m. */
				double rb() const { return _rb; }

				/** @return Radius of the wheels in m. */
				double rw() const { return _rw; }

				/** @return gear. */
				double gear() const { return _gear; }

				/**
				* Project the velocity of the robot in cartesian coordinates to single motor speeds.
				*
				* @param m1		The resulting speed of motor 1 in rpm
				* @param m2		The resulting speed of motor 2 in rpm
				* @param m3		The resulting speed of motor 3 in rpm
				* @param vx		Velocity in x-direction in m/s
				* @param vy		Velocity in y-direction in m/s
				* @param omega	Angular velocity in rad/s
				* @throws		RobotinoException if no valid drive layout parameters are available.
				* @remark This function is thread save
				*/
				void project( float* m1, float* m2, float* m3, float vx, float vy, float omega ) const
				{
					//Projection matrix
					static const double v0[2] = { -0.5 * sqrt( 3.0 ),  0.5 };
					static const double v1[2] = {  0.0              , -1.0 };
					static const double v2[2] = {  0.5 * sqrt( 3.0 ),  0.5 };

					//Scale omega with the radius of the robot
					double vOmegaScaled = _rb * (double)omega ;

					//Convert from m/s to RPM
					const double k = 60.0 * _gear / ( 2.0 * rec::robotino::api2::PI * _rw );

					//Compute the desired velocity
					*m1 = static_cast<float>( ( v0[0] * (double)vx + v0[1] * (double)vy + vOmegaScaled ) * k );
					*m2 = static_cast<float>( ( v1[0] * (double)vx + v1[1] * (double)vy + vOmegaScaled ) * k );
					*m3 = static_cast<float>( ( v2[0] * (double)vx + v2[1] * (double)vy + vOmegaScaled ) * k );
				}

				/**
				* Project single motor speeds to velocity in cartesian coordinates.
				*
				* @param vx		The resulting speed in x-direction in m/s
				* @param vy		The resulting speed in y-direction in m/s
				* @param omega	The resulting angular velocity in rad/s
				* @param m1		Speed of motor 1 in rpm
				* @param m2		Speed of motor 2 in rpm
				* @param m3		Speed of motor 3 in rpm
				* @throws		RobotinoException if no valid drive layout parameters are available.
				* @remark This function is thread save
				*/
				void unproject( float* vx, float* vy, float* omega, float m1, float m2, float m3 ) const
				{
					//Convert from RPM to mm/s
					const double k = 60.0 * _gear / ( 2.0 * rec::robotino::api2::PI * _rw );

					*vx = static_cast<float>( ( (double)m3 - (double)m1 ) / sqrt( 3.0 ) / k );
					*vy = static_cast<float>( 2.0 / 3.0 * ( (double)m1 + 0.5 * ( (double)m3 - (double)m1 ) - (double)m2 ) / k );

					double vw = (double)*vy + (double)m2 / k;

					*omega = static_cast<float>( vw / _rb );
				}

			private:
				/**
				* Distance from robot center to wheel center in m.
				*/
				double _rb;

				/**
				* Radius of the wheels in m.
				*/
				double _rw;

				/**
				* Gear between motor and wheel.
				*/
				double _gear;
			};
		}
	}
}

#endif //_REC_ROBOTINO_API2_OMNIDRIVEMODEL_H_
