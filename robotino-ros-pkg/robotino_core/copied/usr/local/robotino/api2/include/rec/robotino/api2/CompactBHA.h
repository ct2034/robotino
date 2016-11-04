//  Copyright (C) 2004-2013, Robotics Equipment Corporation GmbH

//Copyright (c) ...
//
//REC Robotics Equipment Corporation GmbH, Planegg, Germany. All rights reserved.
//Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
//1) Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
//2) Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
//
//THIS SOFTWARE IS PROVIDED BY REC ROBOTICS EQUIPMENT CORPORATION GMBH ï¿½AS ISï¿½ AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REC ROBOTICS EQUIPMENT CORPORATION GMBH
//BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
//GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//Copyright (c) ...
//
//REC Robotics Equipment Corporation GmbH, Planegg, Germany. Alle Rechte vorbehalten.
//Weiterverbreitung und Verwendung in nichtkompilierter oder kompilierter Form, mit oder ohne Verï¿½nderung, sind unter den folgenden Bedingungen zulï¿½ssig:
//1) Weiterverbreitete nichtkompilierte Exemplare mï¿½ssen das obige Copyright, diese Liste der Bedingungen und den folgenden Haftungsausschluss im Quelltext enthalten.
//2) Weiterverbreitete kompilierte Exemplare mï¿½ssen das obige Copyright, diese Liste der Bedingungen und den folgenden Haftungsausschluss in der Dokumentation und/oder anderen Materialien, die mit dem Exemplar verbreitet werden, enthalten.
//
//DIESE SOFTWARE WIRD VON REC ROBOTICS EQUIPMENT CORPORATION GMBH OHNE JEGLICHE SPEZIELLE ODER IMPLIZIERTE GARANTIEN ZUR VERFï¿½GUNG GESTELLT, DIE UNTER
//ANDEREM EINSCHLIESSEN: DIE IMPLIZIERTE GARANTIE DER VERWENDBARKEIT DER SOFTWARE FÜR EINEN BESTIMMTEN ZWECK. AUF KEINEN FALL IST REC ROBOTICS EQUIPMENT CORPORATION GMBH
//FÜR IRGENDWELCHE DIREKTEN, INDIREKTEN, ZUFÄLLIGEN, SPEZIELLEN, BEISPIELHAFTEN ODER FOLGESCHÄDEN (UNTER ANDEREM VERSCHAFFEN VON ERSATZGÜTERN ODER -DIENSTLEISTUNGEN;
//EINSCHRÄNKUNG DER NUTZUNGSFÄHIGKEIT; VERLUST VON NUTZUNGSFÄHIGKEIT; DATEN; PROFIT ODER GESCHÄFTSUNTERBRECHUNG), WIE AUCH IMMER VERURSACHT UND UNTER WELCHER VERPFLICHTUNG
//AUCH IMMER, OB IN VERTRAG, STRIKTER VERPFLICHTUNG ODER UNERLAUBTER HANDLUNG (INKLUSIVE FAHRLÄSSIGKEIT) VERANTWORTLICH, AUF WELCHEM WEG SIE AUCH IMMER DURCH DIE BENUTZUNG
//DIESER SOFTWARE ENTSTANDEN SIND, SOGAR, WENN SIE AUF DIE MÖGLICHKEIT EINES SOLCHEN SCHADENS HINGEWIESEN WORDEN SIND.

#ifndef _REC_ROBOTINO_API2_COMPACTBHA_H_
#define _REC_ROBOTINO_API2_COMPACTBHA_H_

#include "rec/robotino/api2/defines.h"
#include "rec/robotino/api2/ComObject.h"

#include <vector>
#include <string>

namespace rec
{
	namespace robotino
	{
		namespace api2
		{
			class CompactBHAImpl;

			/**
			 * @brief	Access to Robotino's (optional) compact version of the Bionic Handling Assistant. See <a href="http://www.festo.com/cms/en_corp/11367.htm">Robotino XT</a>.
			 */
			class
#ifdef REC_ROBOTINO_API2_CLASS_ATTRIBUTE
	REC_ROBOTINO_API2_CLASS_ATTRIBUTE
#endif
			CompactBHA : public ComObject
			{
				friend class CompactBHAImpl;
			public:
				CompactBHA();

				virtual ~CompactBHA();

				/**
				 * Sets the associated communication object.
				 *
				 * @param id	The id of the associated communication object.
				 * @throws	RobotinoException If the given communication object doesn't provide a camera.
				 */
				void setComId( const ComId& id );

				/**
				 * Sets pressure of all bellows
				 *
				 * @param pressures Array of pressures in bar. Size of this array must be 8.
				 * @throws	RobotinoException if given com object is invalid.
				 */
				void setPressures( const float* pressures );

				/**
				 * Turns compressors on and off. If on, they do only run when pressure is too low.
				 *
				 * @param enabled State of compressors.
				 * @throws	RobotinoException if given com object is invalid.
				 */
				void setCompressorsEnabled( bool enabled );

				/**
				 * Opens and closes the water drain valve.
				 *
				 * @param open State of the valve.
				 * @throws	RobotinoException if given com object is invalid.
				 */
				void setWaterDrainValve( bool open );

				/**
				 * Opens and closes the gripper valve 1.
				 *
				 * @param open State of the valve.
				 * @throws	RobotinoException if given com object is invalid.
				 */
				void setGripperValve1( bool open );

				/**
				 * Opens and closes the gripper valve 2.
				 *
				 * @param open State of the valve.
				 * @throws	RobotinoException if given com object is invalid.
				 */
				void setGripperValve2( bool open );

				/**
				 * Read current pressure of all bellows
				 *
				 * @param[out] readings Array of pressures in bar. Size of this array must be 8.
				 * @throws	RobotinoException if given com object is invalid.
				 */
				void pressures( float* readings ) const;

				/**
				 * Read pressure sensor
				 *
				 * @return	The signal from the pressure sensor
				 * @throws	RobotinoException if given com object is invalid.
				 */
				bool pressureSensor() const;

				/**
				 * Read string potentiometers
				 *
				 * @param[out] readings Array of readings. Size of this array must be 6.
				 * @throws	RobotinoException if given com object is invalid.
				 */
				void stringPots( float* readings ) const;

				/**
				 * Read foil potentiometer
				 *
				 * @return	The current reading from the foil pot.
				 * @throws	RobotinoException if given com object is invalid.
				 */
				float foilPot() const;
				
				/**
				 * Called when pressure readings from the CBHA are received.
				 * @param pressures Array of current pressures in bar.
				 * @param size Size of pressures array.
				 * @throws nothing.
				 * @remark This function is called from the thread in which Com::processEvents() is called.
				 * @see Com::processEvents
				 */
				virtual void pressuresChangedEvent( const float* pressures, unsigned int size );

				/**
				 * Called when a signal from the pressure sensor is received.
				 * @param pressureSensor Signal from the pressure sensor.
				 * @throws nothing.
				 * @remark This function is called from the thread in which Com::processEvents() is called.
				 * @see Com::processEvents
				 */
				virtual void pressureSensorChangedEvent( bool pressureSensor );

				/**
				 * Called when signals from the string potentiometers are received.
				 * @param readings Array of signals from the string pots.
				 * @param size Size of values array.
				 * @throws nothing.
				 * @remark This function is called from the thread in which Com::processEvents() is called.
				 * @see Com::processEvents
				 */
				virtual void stringPotsChangedEvent( const float* readings, unsigned int size );

				/**
				 * Called when a signal from the foil potentiometer is received.
				 * @param value Signal from the foil pot.
				 * @throws nothing.
				 * @remark This function is called from the thread in which Com::processEvents() is called.
				 * @see Com::processEvents
				 */
				virtual void foilPotChangedEvent( float value );

			private:
				CompactBHAImpl* _impl;
			};
		}
	}
}

#endif //_REC_ROBOTINO_API2_COMPACTBHA_H_
