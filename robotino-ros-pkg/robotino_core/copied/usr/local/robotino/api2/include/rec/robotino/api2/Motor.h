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

#ifndef _REC_ROBOTINO_API2_MOTOR_H_
#define _REC_ROBOTINO_API2_MOTOR_H_

#include "rec/robotino/api2/defines.h"
#include "rec/robotino/api2/ComObject.h"

namespace rec
{
	namespace robotino
	{
		namespace api2
		{
			class MotorImpl;

			/**
			* @brief	Represents a single motor
			*/
			class
#ifdef REC_ROBOTINO_API2_CLASS_ATTRIBUTE
	REC_ROBOTINO_API2_CLASS_ATTRIBUTE
#endif
			Motor : public ComObject
			{
				friend class MotorImpl;
			public:
				Motor();

				virtual ~Motor();

				/**
				* @return Returns the number of drive motors on Robotino
				* @throws nothing
				* @remark This function is thread save
				*/
				static unsigned int numMotors();

				/** 
				* Sets the associated communication object.
				*
				* @param id The id of the associated communication object.
				* @throws	RobotinoException if given id is invalid.
				* @remark This function is thread save
				*/
				void setComId( const ComId& id );

				/**
				* Sets the number of this motor.
				*
				* @param number	number of this motor
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				void setMotorNumber( unsigned int number );

				/**
				* Sets the setpoint speed of this motor.
				*
				* @param speed	Set point speed in rpm.
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				void setSpeedSetPoint( float speed );

				/**
				* Resets the position of this motor.
				* @param position New position after reset.
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				void resetPosition( int position );

				/**
				* Controls the brakes of this motor.
				*
				* @param brake	If set to TRUE, the speed set-point is constantly set to 0. If set to FALSE, the speed set-point can be set by setSpeedSetPoint.
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				void setBrake( bool brake );

				/**
				* Sets the proportional, integral and  differential constant of the PID controller.
				* Robotino v2
				* The range of values is from 0 to 255. These values are scaled by the microcontroller firmware
				* to match with the PID controller implementation.
				* If value is given, the microcontroller firmware uses its build in default value.
				*
				* Robotino v3
				* Parameters are floating point values used by the microcontroller directly. If parameter is less than 0 the default parameter is used.
				*
				* @param kp proportional constant. Typical value for Robotino v2 is 200. Typical value for Robotino v3 is 0.1.
				* @param ki integral constant. Typical value for Robotino v2 is 10. Typical value for Robotino v3 is 0.005.
				* @param kd differential constant. Typical value 0. Not used by Robotino v3.
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				void setPID( float kp, float ki, float kd );

				/**
				* Retrieves the actual speed of this motor.
				*
				* @return	Speed in rpm.
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				float actualVelocity() const;

				/**
				* Retrieves the actual position of this motor.
				*
				* @return actual position
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				int actualPosition() const;

				/**
				* Retrieves the current of this motor.
				* @return motor current in A.
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				float motorCurrent() const;

				/**
				* Called when new readings are available.
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents
				*/
				virtual void motorReadingsChanged( float velocity, int position, float current );

			private:
				MotorImpl* _impl;
			};
		}
	}
}
#endif
