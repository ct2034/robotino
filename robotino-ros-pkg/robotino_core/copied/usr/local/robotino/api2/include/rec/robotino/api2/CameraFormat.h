//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

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

#ifndef _REC_ROBOTINO_API2_CAMERAFORMAT_H_
#define _REC_ROBOTINO_API2_CAMERAFORMAT_H_

#include <string.h>

namespace rec
{
	namespace robotino
	{     
		namespace api2
		{
			/**
			* @brief	Camera format description
			*/
			class CameraFormat
			{
			public:
				/**
				* Constructs a camera format description with empty name and size 0.
				*/
				CameraFormat()
					: width( 0 )
					, height( 0 )
					, _name( NULL )
				{
					_name = new char[1];
					_name[0] = '\0';
				}

				/**
				* Constructs a camera format description.
				*
				* @param width_ Image width.
				* @param height_ Image height.
				* @param name Description name.
				*/
				CameraFormat( unsigned int width_, unsigned int height_, const char* name )
					: width( width_ )
					, height( height_ )
					, _name( NULL )
				{
					setName( name );
				}

				/**
				* Copys an existing camera format description.
				*
				* @param other Existing description to copy.
				*/
				CameraFormat( const CameraFormat& other )
					: width( other.width )
					, height( other.height )
					, _name( NULL )
				{
					setName( other._name );
				}

				/**
				* Destructor.
				*/
				virtual ~CameraFormat()
				{
					delete [] _name;
				}

				/**
				* Copys an existing camera format description.
				*
				* @param other Existing description to copy.
				*/
				CameraFormat& operator=( const CameraFormat& other )
				{
					setName( other._name );
					width = other.width;
					height = other.height;
					return *this;
				}

				/**
				* Set the format name
				* @param str The name like "mjpeg", "yuyv" ...
				*/
				void setName( const char* str )
				{
					size_t len = strlen( str );
					delete [] _name;
					_name = new char[ len+1 ];
					strncpy( _name, str, len+1 );
				}

				/**
				* Read the format name
				* @return Format name
				*/
				const char* name() const
				{
					return _name;
				}

				/**
				* Image width in pixel
				*/
				unsigned int width;

				/**
				* Image height in pixel
				*/
				unsigned int height;

			private:
				char* _name;
			};
		}
	}
}

#endif //_REC_ROBOTINO_API2_CAMERAFORMAT_H_
