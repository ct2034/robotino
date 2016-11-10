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

#ifndef _REC_ROBOTINO_API2_CAMERACAPABILITIES_H_
#define _REC_ROBOTINO_API2_CAMERACAPABILITIES_H_

#include "rec/robotino/api2/CameraFormat.h"

namespace rec
{
	namespace robotino
	{     
		namespace api2
		{
			/**
			 * @brief	Capabilities of Robotino's camera
			 */
			class CameraCapabilities
			{
			public:
				/**
				 * Constructs a CameraCapabilities instance.
				 */
				CameraCapabilities()
					: brightness( false )
					, contrast( false )
					, saturation( false )
					, autoWhiteBalance( false )
					, gain( false )
					, whiteBalanceTemperature( false )
					, backlightCompensation( false )
					, autoExposure( false )
					, exposure( false )
					, autoFocus( false )
					, focus( false )
					, sharpness( false )
					, _name( NULL )
					, _formats( NULL )
					, _numFormats( 0 )
				{
					_name = new char[1];
					_name[0] = '\0';
				}

				/**
				 * Copies an existing CameraCapabilities instance.
				 */
				CameraCapabilities( const CameraCapabilities& other )
					: brightness( other.brightness )
					, contrast( other.contrast )
					, saturation( other.saturation )
					, autoWhiteBalance( other.autoWhiteBalance )
					, gain( other.gain )
					, whiteBalanceTemperature( other.whiteBalanceTemperature )
					, backlightCompensation( other.backlightCompensation )
					, autoExposure( other.autoExposure )
					, exposure( other.exposure )
					, autoFocus( other.autoFocus )
					, focus( other.focus )
					, sharpness( other.sharpness )
					, _name( NULL )
					, _formats( NULL )
					, _numFormats( 0 )
				{
					setName( other._name );
					for( unsigned int i=0; i<other._numFormats; ++i )
					{
						addFormat( other.format( i ) );
					}
				}

				/**
				 * Desonstructor.
				 */
				virtual ~CameraCapabilities()
				{
					delete [] _name;
					delete [] _formats;
				}

				/**
				 * Copies an existing CameraCapabilities instance.
				 */
				CameraCapabilities& operator=( const CameraCapabilities& other )
				{
					setName( other._name );
					delete [] _formats;
					_formats = NULL;
					_numFormats = 0;
					for( unsigned int i=0; i<other._numFormats; ++i )
					{
						addFormat( other.format( i ) );
					}

					brightness = other.brightness;
					contrast = other.contrast;
					saturation = other.saturation;
					autoWhiteBalance = other.autoWhiteBalance;
					gain = other.gain;
					whiteBalanceTemperature = other.whiteBalanceTemperature;
					backlightCompensation = other.backlightCompensation;
					autoExposure = other.autoExposure;
					exposure = other.exposure;
					autoFocus = other.autoFocus;
					focus = other.focus;
					sharpness = other.sharpness;
					return *this;
				}

				/**
				 * Set the format name
				 * @param name Format name
				 */
				void setName( const char* name )
				{
					delete [] _name;
					if ( 0 == name )
					{
						_name = new char[1];
						_name[0] = '\0';
						return;
					}
					size_t len = strlen( name );
					_name = new char[ len+1 ];
					strncpy( _name, name, len+1 );
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
				 * Add a camera format.
				 * @param format CameraFormat to add.
				 * @see numFormats(), format()
				 */
				void addFormat( const CameraFormat& format )
				{
					if( 0 == _numFormats % 16 )
					{
						CameraFormat* old = _formats;
						
						_formats = new CameraFormat[_numFormats+16];

						for( unsigned int i=0; i<_numFormats; ++i )
						{
							_formats[i] = old[i];
						}

						delete [] old;
					}

					_formats[_numFormats] = format;

					++_numFormats;
				}

				/**
				 * Retrieve a camera format.
				 * @param index Index of the camera format.
				 * @return CameraFormat with the given index or empty format if index >= numFormats().
				 * @see numFormats(), addFormat()
				 */
				CameraFormat format( unsigned int index ) const
				{
					if( index >= _numFormats )
					{
						return CameraFormat();
					}
					return _formats[index];
				}

				/**
				 * @return The number of camera formats.
				 * @see format(), addFormat()
				 */
				unsigned int numFormats() const
				{
					return _numFormats;
				}

				/** Camera supports brightness setting. */
				bool brightness;

				/** Camera supports contrast setting. */
				bool contrast;

				/** Camera supports saturation setting. */
				bool saturation;

				/** Camera supports auto white balance setting. */
				bool autoWhiteBalance;

				/** Camera supports gain setting. */
				bool gain;

				/** Camera supports white balance temperature setting. */
				bool whiteBalanceTemperature;

				/** Camera supports backlight compensation setting. */
				bool backlightCompensation;

				/** Camera supports auto exposure setting. */
				bool autoExposure;

				/** Camera supports exposure setting. */
				bool exposure;

				/** Camera supports auto focus setting. */
				bool autoFocus;

				/** Camera supports focus setting. */
				bool focus;

				/** Camera supports sharpness setting. */
				bool sharpness;

			private :
				char* _name;
				CameraFormat* _formats;
				unsigned int _numFormats;
			};
		}
	}
}

#endif //_REC_ROBOTINO_API2_CAMERACAPABILITIES_H_
