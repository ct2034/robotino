//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

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

#ifndef _REC_ROBOTINO_API2_ODOMETRY_H_
#define _REC_ROBOTINO_API2_ODOMETRY_H_

#include "rec/robotino/api2/defines.h"
#include "rec/robotino/api2/ComObject.h"

namespace rec
{
	namespace robotino
	{
		namespace api2
		{
			class OdometryImpl;
			/**
			* @brief	Represents Robotino's odoemtry module.
			*/
			class
#ifdef REC_ROBOTINO_API2_CLASS_ATTRIBUTE
	REC_ROBOTINO_API2_CLASS_ATTRIBUTE
#endif
			Odometry : public ComObject
			{
				friend class OdometryImpl;
			public:
				Odometry();

				virtual ~Odometry();

				/** 
				* Sets the associated communication object.
				*
				* @param id The id of the associated communication object.
				* @throws	RobotinoException if given id is invalid.
				* @remark This function is thread save
				*/
				void setComId( const ComId& id );

				/**
				* @param x Global x position of Robotino in m.
				* @param y Global y position of Robotino in m.
				* @param phi Global orientation of Robotino in rad.
				* @param sequence The sequence number of the current readings. When starting your application the sequence number is 0. The sequence number
				*         is increased 1 each time the odometry is updated.
				* @throws nothing.
				* @remark This function is thread save
				*/
				void readings( double* x, double* y, double* phi, unsigned int* sequence = 0 ) const;

				/**
				* Set Robotino's odometry to the given coordinates
				* @param x Global x position in m
				* @param y Global y position in m
				* @param phi Global phi orientation in rad
				* @param blocking If false the function returns true immediately. If true the function blocks until the odometry is set or if the opration timed out.
				*                 In the case of timeout the function returns false.
				* @throws RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				bool set( double x, double y, double phi, bool blocking = false );

				/**
				* Called when new odometry data is available.
				* @param x Global x position of Robotino in m.
				* @param y Global y position of Robotino in m.
				* @param phi Global orientation of Robotino in rad.
				* @param vx x velocity of Robotino.
				* @param vy y velocity of Robotino.
				* @param omega Angular velocity of Robotino.
				* @param sequence The sequence number of the current readings. When starting your application the sequence number is 0. The sequence number
				*         is increased 1 each time the odometry is updated.
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents
				*/
				virtual void readingsEvent( double x, double y, double phi, float vx, float vy, float omega, unsigned int sequence );

				/**
				* Called when new odometry data is available.
				* @param x Global x position of Robotino in the map frame in m.
				* @param y Global y position of Robotino in the map frame in m.
				* @param phi Global orientation of Robotino in the map frame in rad.
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents
				*/
				virtual void mapFrameCoordinatesEvent( double x, double y, double phi );

				/**
				* Called when new odometry data is available.
				* @param x x-position of Robotino in the map image in pixel.
				* @param y y-position of Robotino in the map image in pixel.
				* @param phi Global orientation of Robotino in the map frame in rad.
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents
				*/
				virtual void mapImagePixelCoordinatesEvent( int x, int y, double phi );

			private:
				OdometryImpl* _impl;
			};
		}
	}
}
#endif //_REC_ROBOTINO_API2_ODOMETRY_H_


