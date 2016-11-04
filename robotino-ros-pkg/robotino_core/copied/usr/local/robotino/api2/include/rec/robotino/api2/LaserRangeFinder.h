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

#ifndef _REC_ROBOTINO_API2_LASERRANGEFINDER_H_
#define _REC_ROBOTINO_API2_LASERRANGEFINDER_H_

#include "rec/robotino/api2/defines.h"
#include "rec/robotino/api2/ComObject.h"
#include "rec/robotino/api2/LaserRangeFinderReadings.h"

#include <vector>
#include <string>

namespace rec
{
	namespace robotino
	{
		namespace api2
		{
			class LaserRangeFinderImpl;

			/**
			* @brief	Access to Robotino's (optional) laser rangefinder
			*/
			class
#ifdef REC_ROBOTINO_API2_CLASS_ATTRIBUTE
	REC_ROBOTINO_API2_CLASS_ATTRIBUTE
#endif
			LaserRangeFinder : public ComObject
			{
				friend class LaserRangeFinderImpl;
			public:
				LaserRangeFinder();

				LaserRangeFinder(const ComId& id);

				virtual ~LaserRangeFinder();

				/**
				* @return Returns the number of supported rangefinders.
				*/
				static unsigned int numLaserRangeFinders();

				/**
				* Sets the number of this rangefinder device.
				*
				* Set the number of this rangefinder to number. The default rangefinder number is 0. Setting the rangefinder number
				* only makes sense when your Robotino is equipped with more than one rangefinder.
				*
				* @param number	The rangefinder number. Range [0; numLaserRangeFinders()-1]. When setting number < 0 no rangefinder is selected.
				* @throws	RobotinoException if the given input number is out of range. RobotinoException if given com object is invalid.
				* @see numCameras
				*/
				void setLaserRangeFinderNumber( int number );

				/**
				* Get the current camera number.
				*
				* @return Returns the current rangefinder number or -1 if no rangefinder is selected.
				* @throws nothing
				* @see setLaserRangeFinderNumber
				*/
				int laserRangeFinderNumber() const;

				/** 
				* Sets the associated communication object.
				*
				* @param id The id of the associated communication object.
				* @throws	RobotinoException if given id is invalid.
				* @remark This function is thread save
				*/
				void setComId( const ComId& id );

				/**
				* Get the current readings.
				* @return The latest readings.
				* @throws nothing
				* @remark This function is thread save
				*/
				LaserRangeFinderReadings readings() const;

				/**
				* Called when new scan data is available.
				* @param scan The sensor readings.
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents
				*/
				virtual void scanEvent( const LaserRangeFinderReadings& scan );

			private:
				LaserRangeFinderImpl* _impl;
			};
		}
	}
}
#endif
