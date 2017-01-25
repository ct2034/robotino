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

#ifndef _REC_ROBOTINO_API2_LASERRANGEFINDERREADINGS_H_
#define _REC_ROBOTINO_API2_LASERRANGEFINDERREADINGS_H_

#include <vector>
#include <string.h>

namespace rec
{
	namespace robotino
	{
		namespace api2
		{
			/**
			 * @brief	Sensor readings of Robotino's (optional) laser rangefinder
			 */
			class LaserRangeFinderReadings
			{
			public:
				/**
				 * Constructs an empty LaserRangeFinderReadings instance.
				 */
				LaserRangeFinderReadings()
					: seq( 0 )
					, stamp( 0 )
					, angle_min( 0.0f )
					, angle_max( 0.0f )
					, angle_increment( 0.0f )
					, time_increment( 0.0f )
					, range_min( 0.0f )
					, range_max( 0.0f )
					, _frame_id( NULL )
					, _ranges( NULL )
					, _rangesSize( 0 )
					, _intensities( NULL )
					, _intensitiesSize( 0 )
				{
					_frame_id = new char[1];
					_frame_id[0] = '\n';
				}

				/**
				 * Copies an existing LaserRangeFinderReadings instance.
				 */
				LaserRangeFinderReadings( const LaserRangeFinderReadings& other )
					: seq( other.seq )
					, stamp( other.stamp )
					, angle_min( other.angle_min )
					, angle_max( other.angle_max )
					, angle_increment( other.angle_increment )
					, time_increment( other.time_increment )
					, range_min( other.range_min )
					, range_max( other.range_max )
					, _frame_id( NULL )
					, _ranges( NULL )
					, _rangesSize( 0 )
					, _intensities( NULL )
					, _intensitiesSize( 0 )
				{
					set_frame_id( other._frame_id );
					setRanges( other._ranges, other._rangesSize );
					setIntensities( other._intensities, other._intensitiesSize );
				}

				/**
				 * Destructor
				 */
				~LaserRangeFinderReadings()
				{
					delete [] _frame_id;
					delete [] _ranges;
					delete [] _intensities;
				}

				/**
				 * Copies an existing LaserRangeFinderReadings instance.
				 */
				LaserRangeFinderReadings& operator=( const LaserRangeFinderReadings& other )
				{
					seq = other.seq;
					stamp = other.stamp ;
					angle_min = other.angle_min ;
					angle_max = other.angle_max ;
					angle_increment = other.angle_increment ;
					time_increment = other.time_increment ;
					range_min = other.range_min ;
					range_max = other.range_max ;
					_frame_id = NULL ;
					_ranges = NULL ;
					_rangesSize = 0 ;
					_intensities = NULL ;
					_intensitiesSize = 0 ;

					set_frame_id( other._frame_id );
					setRanges( other._ranges, other._rangesSize );
					setIntensities( other._intensities, other._intensitiesSize );

					return *this;
				}

				/**
				 * @return The frame id.
				 */
				const char* frame_id() const
				{
					return _frame_id;
				}

				/**
				 * Sets the frame id.
				 * @param frame_id The frame id.
				 */
				void set_frame_id( const char* frame_id )
				{
					delete [] _frame_id;
					if ( 0 == frame_id )
					{
						_frame_id = new char[1];
						_frame_id[0] = '\n';
						return;
					}
					size_t len = strlen( frame_id );
					_frame_id = new char[ len+1 ];
					strncpy( _frame_id, frame_id, len+1 );
				}

				/**
				 * Retrieves the range array.
				 * @param readings Array of ranges.
				 * @param rangesSize Number of ranges.
				 */
				void ranges( const float** readings, unsigned int* rangesSize = NULL ) const
				{
					if( NULL != rangesSize )
					{
						*rangesSize = _rangesSize;
					}
					*readings = _ranges;
				}

				/**
				 * Sets the range array.
				 * @param ranges Array of ranges.
				 * @param rangesSize Number of ranges.
				 */
				void setRanges( const float* ranges, unsigned int rangesSize )
				{
					if( rangesSize != _rangesSize )
					{
						_rangesSize = rangesSize;
						delete [] _ranges;
						_ranges = NULL;
						if( 0 == _rangesSize )
						{
							return;
						}

						_ranges = new float[ rangesSize ];
					}

					memcpy( _ranges, ranges, _rangesSize * sizeof( float ) );
				}

				/**
				 * @return Number of ranges.
				 */
				unsigned int numRanges() const
				{
					return _rangesSize;
				}

				/** Clears the range data. */
				void clearRanges()
				{
					delete [] _ranges;
					_ranges = NULL;
					_rangesSize = 0;
				}

				/**
				 * Retrieves the intensity array.
				 * @param readings Array of intensities.
				 * @param intensitiesSize Number of intensities.
				 */
				void intensities( const float** readings, unsigned int* intensitiesSize = NULL ) const
				{
					if( NULL != intensitiesSize )
					{
						*intensitiesSize = _intensitiesSize;
					}
					*readings = _intensities;
				}

				/**
				 * Sets the intensity array.
				 * @param intensities Array of intensities.
				 * @param intensitiesSize Number of intensities.
				 */
				void setIntensities( const float* intensities, unsigned int intensitiesSize )
				{
					if( intensitiesSize != _intensitiesSize )
					{
						_intensitiesSize = intensitiesSize;
						delete [] _intensities;
						_intensities = NULL;
						if( 0 == _intensitiesSize )
						{
							return;
						}
						
						_intensities = new float[ intensitiesSize ];
					}

					memcpy( _intensities, intensities, _intensitiesSize * sizeof( float ) );
				}

				/**
				 * @return Number of intensities.
				 */
				unsigned int numIntensities() const
				{
					return _intensitiesSize;
				}

				/** Clears the intensity data. */
				void clearIntensities()
				{
					delete [] _intensities;
					_intensities = NULL;
					_intensitiesSize = 0;
				}

				/** Sequence number */
				unsigned int seq;
				/** Time stamp */
				unsigned int stamp;

				/** Minimum angle */
				float angle_min;
				/** Maximum angle */
				float angle_max;
				/** Angle increment */
				float angle_increment;
				/** Time increment */
				float time_increment;
				/** Scan time */
				float scan_time;
				/** Minimum range */
				float range_min;
				/** Maximum range */
				float range_max;

			private:
				char* _frame_id;

				float* _ranges;
				unsigned int _rangesSize;

				float* _intensities;
				unsigned int _intensitiesSize;
			};
		}
	}
}

#endif //_REC_ROBOTINO_API2_LASERRANGEFINDERREADINGS_H_
