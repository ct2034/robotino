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

#ifndef _REC_ROBOTINO_API2_FACTORY4_H
#define _REC_ROBOTINO_API2_FACTORY4_H

#include "rec/robotino/api2/defines.h"
#include "rec/robotino/api2/ComObject.h"

#include "rec/robotino/api2/Factory4Position.h"
#include "rec/robotino/api2/Factory4RobotInfo.h"
#include "rec/robotino/api2/Factory4McData.h"
#include "rec/robotino/api2/Factory4MapInfo.h"
#include "rec/robotino/api2/Factory4JobInfo.h"
#include "rec/robotino/api2/Factory4JobError.h"
#include "rec/robotino/api2/Factory4MapList.h"


namespace rec
{
    namespace robotino
    {
        namespace api2
        {
			class Factory4Impl;

			/**
			* @brief	Factory4 interface.
			*/
            class
#ifdef REC_ROBOTINO_API2_CLASS_ATTRIBUTE
	REC_ROBOTINO_API2_CLASS_ATTRIBUTE
#endif
			Factory4 : public ComObject
            {
				friend class Factory4Impl;
            public:
				enum
				{
					OPERATIONMODE_OUTOFSERVICE,
					OPERATIONMODE_AUTO,
					OPERATIONMODE_MANUAL
				};

                Factory4();

                virtual ~Factory4();

				/**
				* Sets the associated communication object.
				*
				* @param id The id of the associated communication object.
				* @throws	RobotinoException if given id is invalid.
				* @remark This function is thread save
				*/
                void setComId( const ComId& id );

				/**
				* Call this function from your main thread to get the virtual Factory4 functions called.
				* The virtual functions are called directly by a call of this function
				* @throws nothing
				* @see Com::processEvents
				*/
				void processEvents();

				/*
				For a list of possoble messages see http://wiki.openrobotino.org/index.php?title=SmartFestoFleetCom
				*/
				void sendMessage(const char* message);

				/**
				* Send a modified version of the navigation map to the robot. Provide data as png image.
				*/
				void setMapPlanner(const char* data, unsigned int dataSize);

				void mapDirRequest(const char* infoData, const unsigned int infoDataSize);
				void mapDirRequest(const char* infoData, const unsigned int infoDataSize, const char* data, const unsigned int dataSize);

                void savePathNetwork(const char* data, const unsigned int dataSize);

				void setSmartLocations(const char* data, const unsigned int dataSize);

				/*Events*/

				virtual void mclayoutEvent(const char* data, const unsigned int dataSize);
				virtual void mcstatusEvent(const char* data, const unsigned int dataSize);
				virtual void pathnetworkEvent(const char* data, const unsigned int dataSize);
				virtual void localizationModeEvent(const char* mode, const unsigned int dataSize);
				virtual void smartlogEvent(const char* data, const unsigned int dataSize);
				virtual void smartnavigationplanEvent(const char* data, const unsigned int dataSize);
				virtual void smartlocationsEvent(const char* data, const unsigned int dataSize);
				virtual void smartrobotinfoEvent(const char* data, const unsigned int dataSize);
				virtual void smartmyrobotidEvent(const char* data, const unsigned int dataSize);
				virtual void smartjoblistEvent(const char* data, const unsigned int dataSize);
				virtual void mapDirEvent(const char* data, const unsigned int dataSize);
				virtual void fleetcom_responseEvent(const char* data, const unsigned int dataSize);
				virtual void mapDir_responseEvent(const char* infoData, const unsigned int infoDataSize, const char* data, const unsigned int dataSize);

				/**
				* The current map used for localization and navigation as jpg image.
				*/
				virtual void mapEvent(const char* data, unsigned int dataSize, const rec::robotino::api2::Factory4MapInfo& info);

				/**
				* The navigation map as png image
				*/
				virtual void mapPlannerEvent(const char* data, unsigned int dataSize, const rec::robotino::api2::Factory4MapInfo& info);

            private:
				Factory4Impl* _impl;
            };
        }
    }
}
#endif // _REC_ROBOTINO_API2_FACTORY4_H
