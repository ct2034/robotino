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

#ifndef _REC_ROBOTINO_API2_GrapplerREADINGS_H_
#define _REC_ROBOTINO_API2_GrapplerREADINGS_H_

#include <vector>

namespace rec
{
	namespace robotino
	{
		namespace api2
		{
			/**
			* @brief	Sensor readings of Robotino's (optional) Grappler
			*/
			class
			GrapplerReadings
			{
			public:
				/**
				 * Maximum number of servos
				 */
				static const unsigned int maxNumServos = 16;

				/**
				 * Constructs an empty GrapplerReadings instance.
				 */
				GrapplerReadings()
					: numServos( 0 )
					, sequenceNumber( 0 )
					, isTorqueEnabled( false )
				{
					clear();
				}

				/**
				 * Destructor
				 */
				virtual ~GrapplerReadings()
				{
				}

				/**
				 * @return true if this object does not store any information.
				 */
				bool isEmpty() const
				{
					return 0 == numServos;
				}

				/**
				 * Removes all information stored in this object.
				 */
				void clear()
				{
					numServos = 0;
					sequenceNumber = 0;
					isTorqueEnabled = false;
					for( unsigned int i=0; i<maxNumServos; ++i )
					{
						angles[i] = 0.0f;
						speeds[i] = 0.0f;
						errors[i] = -1;
						channels[i] = -1;
						ids[i] = -1;
						cwAxesLimits[i] = 0.0f;
						ccwAxesLimits[i] = 0.0f;
					}
				}

				/**
				 *Servo angles in deg
				 */
				float angles[maxNumServos];

				/**
				 *Servo speeds in rpm
				 */
				float speeds[maxNumServos];

				/**
				 * Servo errors.
				 * The error is a bit mask encoding the following errors
				 * Bit 0 - Input voltage error.
				 * Bit 1 - Angle limit error
				 * Bit 2 - Overheating error
				 * Bit 3 - range error
				 * Bit 4 - checksum error
				 * Bit 5 - Overload error
				 * Bit 6 - Instruction error
				 * Bit 7 - na
				 */
				int errors[maxNumServos];

				/**
				 * The channels of the axes.
				 */
				int channels[maxNumServos];

				/**
				 * The ids of the axes.
				 */
				int ids[maxNumServos];

				/**
				 * The CW limit of the axes in deg.
				 */
				float cwAxesLimits[maxNumServos];

				/**
				 * The CCW limit of the axes in deg.
				 */
				float ccwAxesLimits[maxNumServos];

				/**
				 * The number of servos.
				 */
				unsigned int numServos;

				/**
				 * Sequence number of this data packet.
				 */
				unsigned int sequenceNumber;

				/**
				 * Torque state.
				 */
				bool isTorqueEnabled;
			};
		}
	}
}

#endif //_REC_ROBOTINO_API2_GrapplerREADINGS_H_
