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

#ifndef _REC_ROBOTINO_API2_CAMERA_H_
#define _REC_ROBOTINO_API2_CAMERA_H_

#include "rec/robotino/api2/defines.h"
#include "rec/robotino/api2/ComObject.h"
#include "rec/robotino/api2/CameraCapabilities.h"

namespace rec
{
	namespace robotino
	{     
		namespace api2
		{
			class CameraImpl;

			/**
			* @brief	Represents a camera.
			*/
			class
#ifdef REC_ROBOTINO_API2_CLASS_ATTRIBUTE
	REC_ROBOTINO_API2_CLASS_ATTRIBUTE
#endif
			Camera : public ComObject
			{
				friend class CameraImpl;
			public:
				/**
				* Constructs a camera.
				*/
				Camera();

				Camera(const ComId& id);

				/**
				* Destructor.
				*/
				virtual ~Camera();

				/** 
				* Sets the associated communication object.
				*
				* @param id The id of the associated communication object.
				* @throws	RobotinoException if given id is invalid.
				* @remark This function is thread save
				*/
				void setComId( const ComId& id );

				/**
				* @return Returns the number of supported cameras.
				*/
				static unsigned int numCameras();

				/**
				* Sets the number of this camera device.
				*
				* Set the number of this camera to number. The default camera number is 0. Setting the camera number
				* only makes sense when your Robotino is equipped with more than one camera.
				*
				* @param number	The camera number. Range [0; numCameras()-1]. When setting number < 0 no camera is selected.
				* @throws	RobotinoException if the given input number is out of range. RobotinoException if given com object is invalid.
				* @see numCameras
				*/
				void setCameraNumber( int number );

				/**
				* Get the current camera number.
				*
				* @return Returns the current camera number or -1 if no camera is selected.
				* @throws nothing
				* @see setCameraNumber
				*/
				int cameraNumber() const;

				/**
				* Enable/Disable decoding of JPG images. The decoding is enabled by default.
				*
				* On iOS libjpeg is not linked into the API2 libraries. Decoding a jpeg image
				* by this camera class will crash your app. Disable decoding and look for
				* imageReceivedEvents with dataSize>0 and width==0. This is a jpeg image which
				* must be decoded by yourself. If you have a Qt app you can use
				* QImage::fromData(data,dataSize,"jpg") for decoding.
				*
				* @enable If true, decoding is enabled. If false, decoding is disabled.
				* @throws nothing
				*/
				void setJPGDecodingEnabled(bool enable);

				/**
				* Shows the current settings for decoding JPG images.
				*
				* @return Returns the current setting. True - decoding enabled, false otherwise.
				* @throws nothing
				*/
				bool isJPGDecodingEnabled() const;

				/**
				* Call this function from your main thread to get the virtual Camera functions called.
				* The virtual functions are called directly by a call of this function
				* @throws nothing
				* @see Com::processEvents
				*/
				void processEvents();

				/**
				* Enable this camera to receive images and to write inmages.
				*/
				void setReadWrite();

				/**
				* Enable this camera to write inmages but disable the receiving of images.
				* This disables calls to imageReceivedEvent.
				*/
				void setWriteOnly();

				/**
				* Publish a jpg image to the camera topic.
				@param data The jpg image data.
				@param dataSize Number of bytes in the data array.
				*/
				void setJPGImage( const unsigned char* data, unsigned int dataSize ); 

				/**
				* Sets this camera to the given resolution (if possible)
				*
				* @param width Requested image width
				* @param height Requested image height
				* @param format "jpg" to request JPEG compress images. "raw" to request raw RGB images.
				* @throws	RobotinoException if given com object is invalid.
				* @see capabilites
				*/
				void setFormat( unsigned int width, unsigned int height, const char* format );

				/**
				* Set the brightness.
				* @param value
				* @throws	RobotinoException if given com object is invalid.
				* @see capabilites
				*/
				void setBrightness( int value );

				/**
				* Set the contrast.
				* @param value
				* @throws	RobotinoException if given com object is invalid.
				* @see capabilites
				*/
				void setContrast( int value );

				/**
				* Set the saturation.
				* @param value
				* @throws	RobotinoException if given com object is invalid.
				* @see capabilites
				*/
				void setSaturation( int value );

				/**
				* Enable auto white balance.
				* @param enable
				* @throws	RobotinoException if given com object is invalid.
				* @see capabilites
				*/
				void setAutoWhiteBalanceEnabled( bool enable );

				/**
				* Set the gain.
				* @param value
				* @throws	RobotinoException if given com object is invalid.
				* @see capabilites
				*/
				void setGain( int value );

				/**
				* Set the white balance temperature.
				* @param value
				* @throws	RobotinoException if given com object is invalid.
				* @see capabilites
				*/
				void setWhiteBalanceTemperature( int value );

				/**
				* Set the backlight compensation.
				* @param value
				* @throws	RobotinoException if given com object is invalid.
				* @see capabilites
				*/
				void setBacklightCompensation( int value );

				/**
				* Enable auto exposure.
				* @param enable
				* @throws	RobotinoException if given com object is invalid.
				* @see capabilites
				*/
				void setAutoExposureEnabled( bool enable );

				/**
				* Set the exposure.
				* @param value
				* @throws	RobotinoException if given com object is invalid.
				* @see capabilites
				*/
				void setExposure( int value );

				/**
				* Enable auto focus.
				* @param enable
				* @throws	RobotinoException if given com object is invalid.
				* @see capabilites
				*/
				void setAutoFocusEnabled( bool enable );

				/**
				* Set the focus.
				* @param value
				* @throws	RobotinoException if given com object is invalid.
				* @see capabilites
				*/
				void setFocus( int value );

				/**
				* Set the sharpness.
				* @param value
				* @throws	RobotinoException if given com object is invalid.
				* @see capabilites
				*/
				void setSharpness( int value );

				/**
				* @return The camera's capapibilities.
				*/
				CameraCapabilities capabilities() const;

				/**
				* Get the current calibration
				* @param buffer A array of doubles provided by you to be filled with the calibration
				* @param bufferSize The size of the buffer provided
				* @return Returns the number of values copied to buffer.
				*/
				int calibration( double** buffer, unsigned int bufferSize );

				/**
				* Check for new images.
				* @param dataSize Stores the size in bytes of the available image. When you get the image with getImage your buffer must be at least of size dataSize.
				* @throws nothing
				* @see getImage
				*/
				bool isNewImageAvailable( unsigned int* dataSize ) const;

				/**
				* Get the latest image.

				* @param data Buffer where the RGB interleaved image is copied to.
				* @param dataSize Maximum dataSize bytes are copied to the buffer data is pointing to.
				* @param width The number of pixels per line. If width=height=step=0 this is a JPG encoded image.
				* @param height The number of lines
				* @param step The number of bytes per line.
				* @return Returns true if the image fits into the provided buffer, i.e. dataSize >= step*height. Returns false otherwise.
				* @throws nothing.
				* @see isNewImageAvailable
				*/
				bool getImage( unsigned char** data,
					unsigned int dataSize,
					unsigned int* width,
					unsigned int* height,
					unsigned int* step );

				/**
				* Swap channels in the received image
				*
				* By default the channel order is RGB.
				* @param enable If true the channel order is set to BGR.
				*/
				void setBGREnabled( bool enable );

				/**
				* Get the current channel order.
				* @return If true the channel order is BGR. Otherwise the channel order is RGB (the default).
				*/
				bool isBGRenabled() const;

				/**
				* @return Returns true if the connection is a local connection. Otherwise returns false.
				*/
				bool isLocalConnection() const;

				/**
				* Called when an image is received.

				* @param data Contains the image data as RGB interleaved image. You must not delete this buffer. Instead you should make a copy of this buffer within this function call.
				* @param dataSize Is the size of the data buffer containing the RGB image.
				* @param width The number of pixels per line. If width=height=step=0 this is a JPG encoded image.
				* @param height The number of lines
				* @param step The number of bytes per line.
				* @throws nothing.
				* @remark This function is called from the thread in which Com::processEvents() or Camera::processEvents() is called.
				* @see Com::processEvents
				*/
				virtual void imageReceivedEvent( const unsigned char* data,
					                             unsigned int dataSize,
											     unsigned int width,
												 unsigned int height,
												 unsigned int step );

				/**
				* Called when camera capabilities are received
				* @throws nothing.
				* @remark This function is called from the thread in which Com::processEvents() or Camera::processEvents() is called.
				* @see Com::processEvents
				*/
				virtual void capabilitiesChangedEvent( const rec::robotino::api2::CameraCapabilities& capablities );

				/**
				* Called when camera settings are received
				* @throws nothing.
				* @remark This function is called from the thread in which Com::processEvents() or Camera::processEvents() is called.
				* @see Com::processEvents
				*/
				virtual void settingsChangedEvent( unsigned int width, unsigned int height, const char* format );

				/**
				* Called when camera calibration is received
				* @throws nothing.
				* @remark This function is called from the thread in which Com::processEvents() or Camera::processEvents() is called.
				* @see Com::processEvents, Camera::processEvents
				*/
				virtual void calibrationChanged( const double* data, unsigned int dataSize );

			private:
				CameraImpl* _impl;
			};
		}
	}
}

#endif
