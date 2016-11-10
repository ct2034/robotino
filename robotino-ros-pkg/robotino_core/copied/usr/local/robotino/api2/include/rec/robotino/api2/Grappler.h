//  Copyright (C) 2004-2010, Robotics Equipment Corporation GmbH

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

#ifndef _REC_ROBOTINO_API2_Grappler_H_
#define _REC_ROBOTINO_API2_Grappler_H_

#include "rec/robotino/api2/defines.h"
#include "rec/robotino/api2/ComObject.h"
#include "rec/robotino/api2/GrapplerReadings.h"

namespace rec
{
	namespace robotino
	{
		namespace api2
		{
			class GrapplerImpl;

			/**
			* @brief	Access to Robotino's (optional) Grappler
			*/
			class
#ifdef REC_ROBOTINO_API2_CLASS_ATTRIBUTE
	REC_ROBOTINO_API2_CLASS_ATTRIBUTE
#endif
			Grappler : public ComObject
			{
				friend class GrapplerImpl;
			public:
				Grappler();

				virtual ~Grappler();

				/** 
				* Sets the associated communication object.
				*
				* @param id The id of the associated communication object.
				* @throws	RobotinoException if given id is invalid.
				* @remark This function is thread save
				*/
				void setComId( const ComId& id );

				/**
				* Call this function from your main thread to get the virtual Grappler functions called.
				* The virtual functions are called directly by a call of this function
				* @throws nothing
				*/
				void processEvents();

				/**
				* Set position and speed set-point for one axis.
				* @param axis Axis number. Axes are counted starting with 0.
				* @param angle This is the position set-point in deg
				* @param speed This is the speed set-point in rpm
				* @return Returns true on success. Returns false if axis is out of range.
				* @throws	RobotinoException if the underlying communication object is invalid
				* @remark This function is thread save
				*/
				bool setAxis( unsigned int axis, float angle, float speed );

				/**
				* Set position and speed set-points for all axes.
				* @param angles These are the position set-points in deg for all axes
				* @param anglesSize The size of the angles array
				* @param speeds These are the speed set-points in rpm for all axes
				* @param speedsSize The size of the speeds array
				* @return Returns true on success. Returns false if
				* \li anglesSize != speedsSize
				* \li anglesSize >= number of servos detected

				* @throws	RobotinoException if the underlying communication object is invalid
				* @remark This function is thread save
				*/
				bool setAxes( const float* angles, unsigned int anglesSize, const float* speeds, unsigned int speedsSize );

				/**
				* Enable/Disable power of a servo channel
				* @param channel
				* \li 0 - RX64
				* \li 1 - RX28
				* \li 2 - RX10

				* @param enable True - enable
				*               False - disable
				* @return Returns true on success. Returns false if channel is out of range, i.e. > 2.
				* @throws	RobotinoException if the underlying communication object is invalid
				* @remark This function is thread save
				*/
				bool setPowerEnabled( unsigned int channel, bool enable );

				/**
				* Toggle the servo torque status.
				* @throws	RobotinoException if the underlying communication object is invalid
				* @remark This function is thread save
				*/
				void toggleTorque();

				/**
				* Get the current readings.
				* @return Grappler readings.
				* \li \c channels The channel of the servo found.
				* \li \c ids The servos id.
				* \li \c angles The servos current angle.
				* \li \c speeds The servos current speed.
				* \li \c errors The servos current error (if any).

				* @throws nothing
				* @remark This function is thread save
				*/
				GrapplerReadings readings() const;

				/**
				* Get the current info. Readings contain
				* \li \c channels The channel of the servo found.
				* \li \c ids The servos id.
				* \li \c cwAxesLimits The servos clockwise axis limit.
				* \li \c ccwAxesLimits The servos counter-clockwise axis limit.

				* @return Grappler servo info.
				* @throws nothing
				* @remark This function is thread save
				*/
				GrapplerReadings info() const;

				/**
				* Called when servos are detected.
				* @param info Servo info.
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents
				*/
				virtual void servoInfoEvent( const rec::robotino::api2::GrapplerReadings& info );

				/**
				* Called when new Grappler data is available.
				* @param readings The sensor readings.
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents
				*/
				virtual void readingsEvent( const rec::robotino::api2::GrapplerReadings& readings );

				/**
				* Called when new Grappler data is available.
				* @param readings The sensor readings.
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents
				*/
				virtual void storePositionsEvent( const rec::robotino::api2::GrapplerReadings& readings );

				/**
				* Called when toggle torque button is pressed at the grappler
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents
				*/
				virtual void toggleTorqueEvent();

			private:
				GrapplerImpl* _impl;
			};
		}
	}
}
#endif
