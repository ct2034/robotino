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

#ifndef _REC_ROBOTINO_API2_Kinect_H_
#define _REC_ROBOTINO_API2_Kinect_H_

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
			class KinectImpl;

			/**
			* @brief	Access to Robotino's (optional) Kinect
			*/
			class
#ifdef REC_ROBOTINO_API2_CLASS_ATTRIBUTE
	REC_ROBOTINO_API2_CLASS_ATTRIBUTE
#endif
			Kinect : public ComObject
			{
				friend class KinectImpl;
			public:
				/** Constructs a Kinect instance. */
				Kinect();

				/** Destructor */
				virtual ~Kinect();

				/** 
				* Sets the associated communication object.
				*
				* @param id The id of the associated communication object.
				* @throws	RobotinoException if given id is invalid.
				* @remark This function is thread save
				*/
				void setComId( const ComId& id );

				/**
				* @return Returns the number of supported Kinect devices.
				*/
				static unsigned int numKinects();

				/**
				* Sets the number of this Kinect device.
				*
				* Set the number of this Kinect to number. The default Kinect number is 0. Setting the Kinect number
				* only makes sense when your Robotino is equipped with more than one Kinect.
				*
				* @param number	The Kinect number. Range [0; numKinects()-1]. When setting number < 0 no Kinect is selected.
				* @throws	RobotinoException if the given Kinect number is out of range. RobotinoException if given com object is invalid.
				* @see numKinects
				*/
				void setKinectNumber( int number );

				/**
				* Get the current Kinect number.
				*
				* @return Returns the current Kinect number or -1 if no Kinect is selected.
				* @throws nothing
				* @see setKinectNumber
				*/
				int kinectNumber() const;

				/**
				* Call this function from your main thread to get the virtual Kinect functions called.
				* The virtual functions are called directly by a call of this function
				* @throws nothing
				*/
				void processEvents();

				/**
				* Set tilt angle.
				* @param angleDeg The tilt angle in degrees.
				* @throws	RobotinoException if the underlying communication object is invalid
				* @remark This function is thread save
				*/
				void setTilt( double angleDeg );

				/**
				* Set LED.
				* @param state The state of the LED.
				*				\li 0 Turn LED off
				*				\li 1 Turn LED to Green
				*				\li 2 Turn LED to Red
				*				\li 3 Turn LED to Yellow
				*				\li 4 Make LED blink Green
				*				\li 5 Make LED blink Green
				*				\li 6 Make LED blink Red/Yellow 
				* @throws	RobotinoException if the underlying communication object is invalid
				* @remark This function is thread save
				*/
				void setLED( unsigned int state );


				/**
				* Set the depth data format
				* @param format
					\li 0 11 bit depth information in one uint16_t/pixel
					\li 1 10 bit depth information in one uint16_t/pixel
					\li 2 11 bit packed depth information
					\li 3 10 bit packed depth information
					\li 4 processed depth data in mm, aligned to 640x480 RGB
					\li 5 depth to each pixel in mm, but left unaligned to RGB image
				* @throws	RobotinoException if the underlying communication object is invalid
				* @remark This function is thread save
				*/
				void setDepthFormat( unsigned int format );

				/**
				* Set the video data format
				* @param format
					\li 0 Decompressed RGB mode (demosaicing done by libfreenect)
					\li 1 Bayer compressed mode (raw information from camera)
					\li 2 8-bit IR mode
					\li 3 10-bit IR mode
					\li 4 10-bit packed IR mode
					\li 5 YUV RGB mode
					\li 6 YUV Raw mode
				* @throws	RobotinoException if the underlying communication object is invalid
				* @remark This function is thread save
				*/
				void setVideoFormat( unsigned int format );

				/**
				* Swap channels in the received video image
				*
				* By default the channel order is RGB.
				* @param enable If true the channel order is set to BGR.
				*/
				void setBGREnabled( bool enable );

				/**
				* Get the current video channel order.
				* @return If true the channel order is BGR. Otherwise the channel order is RGB (the default).
				*/
				bool isBGRenabled() const;

				/**
				* Called when new depth data is available
				* @param data Contains the depth image data. You must not delete this buffer. Instead you should make a copy of this buffer within this function call.
				* @param dataSize Size of data
				* @param width Horizontal resolution
				* @param height Vertical resolution
				* @param format Data format
				* @param stamp Time stamp
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents processEvents
				*/
				virtual void depthEvent( const unsigned short* data, unsigned int dataSize, const unsigned short* object_data, unsigned int object_dataSize, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );

				/**
				* Called when new depth data is available
				* @param data Contains the image data as RGB interleaved image. You must not delete this buffer. Instead you should make a copy of this buffer within this function call.
				* @param dataSize Size of data
				* @param width Horizontal resolution
				* @param height Vertical resolution
				* @param step Bytes per scanline
				* @param format Video format
				* @param stamp Time stamp
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents processEvents
				*/
				virtual void videoEvent( const unsigned char* data, unsigned int dataSize, unsigned int width, unsigned int height, unsigned int step, unsigned int format, unsigned int stamp );

				/**
				* Called when new tilt data is available
				* @param angleDeg Angle in degrees.
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents processEvents
				*/
				virtual void tiltEvent( double angleDeg );

				/**
				* Called when new acceleration data is available
				* @param x x acceleration.
				* @param y y acceleration.
				* @param z z acceleration.
				* @remark This function is called from the thread in which Com::processEvents() is called.
				* @see Com::processEvents processEvents
				*/
				virtual void accelEvent( double x, double y, double z );

			private:
				KinectImpl* _impl;
			};
		}
	}
}
#endif
