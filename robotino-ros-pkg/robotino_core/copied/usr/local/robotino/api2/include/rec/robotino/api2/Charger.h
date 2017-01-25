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

#ifndef _REC_ROBOTINO_API2_Charger_H_
#define _REC_ROBOTINO_API2_Charger_H_

#include "rec/robotino/api2/defines.h"
#include "rec/robotino/api2/ComObject.h"

namespace rec
{
	namespace robotino
	{
		namespace api2
		{
			class ChargerImpl;

			/**
			* @brief	Represents a single Charger
			*/
			class
#ifdef REC_ROBOTINO_API2_CLASS_ATTRIBUTE
	REC_ROBOTINO_API2_CLASS_ATTRIBUTE
#endif
			Charger : public ComObject
			{
				friend class ChargerImpl;
			public:
				Charger();

				virtual ~Charger();

				/**
				* @return Returns the maximum number of chargers on Robotino
				* @throws nothing
				* @remark This function is thread save
				*/
				static unsigned int numChargers();

				/** 
				* Sets the associated communication object.
				*
				* @param id The id of the associated communication object.
				* @throws	RobotinoException if given id is invalid.
				* @remark This function is thread save
				*/
				void setComId( const ComId& id );

				/**
				* Sets the number of this Charger.
				*
				* @param number	number of this Charger
				* @throws nothing
				* @remark This function is thread save
				*/
				void setChargerNumber( int number );

				/**
				* Get the current charger number
				*
				* @return Charger number
				* @throws nothing
				* @remark This function is thread save
				*/
				int chargerNumber() const;

				/**
				* Clear error.
				*
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				void clearError();

				/**
				* Retrieves the actual speed of this Charger.
				*
				* @return	Internal time of charger modulo 0xFFFF in seconds.
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				unsigned int time() const;

				/**
				* Battery voltage measured by the charger.
				*
				* @return Battery voltage in Volts.
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				float batteryVoltage() const;

				/**
				* Charging current.
				* @return Charging current in A.
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				float chargingCurrent() const;

				/**
				* Temperature of battery 1.
				* @return Temperature in °C.
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				float bat1temp() const;

				/**
				* Temperature of battery 2.
				* @return Temperature in °C.
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				float bat2temp() const;

				/**
				* The current charger state as a number.
				* @return The state.
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				int state_number() const;

				/**
				* The current charger state as human readable string.
				* @return State in string representation.
				* @throws	RobotinoException if the current communication object is invalid.
				* @remark This function is thread save
				*/
				const char* state() const;

				void version( int* major, int* minor, int* patch );

				/**
				* Called when new info are available.
				* @param time Charger time in seconds
				* @param batteryVoltage Voltage of the battery attached to the charger in V.
				* @param chargingCurrent Charging current in A.
				* @param bat1temp Temperature of battery 1 in °C.
				* @param bat2temp Temperature of battery 2 in °C.
				* @param state_number Charger state
				* @param state Charger state string representation
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents
				*/
				virtual void chargerInfoChanged( unsigned int time, float batteryVoltage, float chargingCurrent, float bat1temp, float bat2temp, int state_number, const char* state );

				virtual void chargerErrorChanged( unsigned int time, const char* message );

				virtual void chargerVersionChanged( int major, int minor, int patch );

			private:
				ChargerImpl* _impl;
			};
		}
	}
}
#endif
