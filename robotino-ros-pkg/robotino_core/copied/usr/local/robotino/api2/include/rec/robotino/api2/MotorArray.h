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

#ifndef _REC_ROBOTINO_API2_MotorArray_H_
#define _REC_ROBOTINO_API2_MotorArray_H_

#include "rec/robotino/api2/defines.h"
#include "rec/robotino/api2/ComObject.h"

namespace rec
{
	namespace robotino
	{
		namespace api2
		{
			class MotorArrayImpl;

			/**
			* @brief	Represents a single MotorArray
			*/
			class
#ifdef REC_ROBOTINO_API2_CLASS_ATTRIBUTE
	REC_ROBOTINO_API2_CLASS_ATTRIBUTE
#endif
			MotorArray : public ComObject
			{
				friend class MotorArrayImpl;
			public:
				/** Constructs a MotorArray instance. */
				MotorArray();

				/** Destructor */
				virtual ~MotorArray();

				/**
				* @return Returns the number of drive MotorArrays on Robotino
				* @throws nothing
				* @remark This function is thread safe
				*/
				static unsigned int numMotors();

				/** 
				* Sets the associated communication object.
				*
				* @param id The id of the associated communication object.
				* @throws	RobotinoException if given id is invalid.
				* @remark This function is thread safe
				*/
				void setComId( const ComId& id );

				/**
				* Sets the setpoint speed of this MotorArray.
				*
				* @param speeds	Array of set point speed in rpm for all motors.
				* @param size	Size of speeds array.
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread safe
				*/
				void setSpeedSetPoints( const float* speeds, unsigned int size );

				/**
				* Retrieves the actual velocities of this MotorArray.
				*
				* Example:
				* @code
				* std::vector<float> vec( numMotors() );
				* actualVelocities( &vec[0] );
				* for( int i=0; i<vec.size(); ++i )
				* {
				*   std::cout << "actual velocity[" << i << "]: " << vec[i] << " rpm" << std::endl;
				* }
				* @endcode
				*
				* @param[out] readings Pass an array of size numMotors() to store velocities of all motors in rpm.
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread safe
				*/
				void actualVelocities( float* readings ) const;

				/**
				* Retrieves the actual position of this MotorArray.
				*
				* Example:
				* @code
				* std::vector<float> vec( numMotors() );
				* actualPositions( &vec[0] );
				* for( int i=0; i<vec.size(); ++i )
				* {
				*   std::cout << "actual position[" << i << "]: " << vec[i] << std::endl;
				* }
				* @endcode
				*
				* @param[out] readings Pass an array of size numMotors() to store  actual positions of all motors
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread safe
				*/
				void actualPositions( int* readings ) const;

				/**
				* Retrieves the current of this MotorArray.
				*
				* Example:
				* @code
				* std::vector<float> vec( numMotors() );
				* motorCurrents( &vec[0] );
				* for( int i=0; i<vec.size(); ++i )
				* {
				*   std::cout << "motor current[" << i << "]: " << vec[i] << " A" << std::endl;
				* }
				* @endcode
				*
				* @param[out] readings Pass an array of size numMotors() to store  currents of all motors in A
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread safe
				*/
				void motorCurrents( float* readings ) const;

				/**
				* Called when new velocities are available.
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents
				*/

				virtual void velocitiesChangedEvent( const float* velocities, unsigned int size );

				/**
				* Called when new positions are available.
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents
				*/

				virtual void positionsChangedEvent( const int* positions, unsigned int size );

				/**
				* Called when new currents are available.
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents
				*/

				virtual void currentsChangedEvent( const float* currents, unsigned int size );

			private:
				MotorArrayImpl* _impl;
			};
		}
	}
}
#endif
