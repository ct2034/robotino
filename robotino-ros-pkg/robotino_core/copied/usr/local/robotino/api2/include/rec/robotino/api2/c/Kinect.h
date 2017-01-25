//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_Kinect_H_
#define _REC_ROBOTINO_API2_C_Kinect_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file Kinect.h
    \brief In "rec/robotino/api2/c/Kinect.h" you can find functions for reading Robotino's laser rangefinder.

		Use Kinect_construct() to create a new rangefinder object. Associate the rangefinder object with a com object using Kinect_setComId().
		Use Kinect_getReadings() to get sensor readings from Robotino's rangefinder.
*/

/** KinectId */
typedef int KinectId;

/** Invalid KinectId is -1 */
#define INVALID_KinectID -1

/**
Construct an Kinect object
@return Returns the ID of the newly constructed Kinect object.
*/
DLLEXPORT KinectId Kinect_construct();

/**
Destroy the Kinect object assigned to id
@param id The id of the Kinect object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given KinectId is invalid.
*/
DLLEXPORT BOOL Kinect_destroy( KinectId id );

/**
Associated an Kinect object with a communication interface, i.e. binding the Kinect to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given KinectId is invalid.
*/
DLLEXPORT BOOL Kinect_setComId( KinectId id, ComId comId );

/**
 * Sets the number of this Kinect.
 *
 * @param id The id of the kinect object to be set
 * @param number	number of this Kinect
 * @throws	RobotinoException if the current communication object is invalid.
 */
DLLEXPORT BOOL Kinect_setKinectNumber( KinectId id, unsigned int number );

/**
Grab image.
@param id The kinect id.
@return Returns TRUE (1) if a new sensor readings are available since the last call of Kinect_grab. Returns FALSE (0) otherwise.
*/
DLLEXPORT BOOL Kinect_grab( KinectId id );

/**
Size of data aquired by grab.
@param id The kinect id.
@param depthDataWidth data width.
@param depthDataHeight data height.
@param objectDataWidth object data width.
@param objectDataHeight object data height.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given KinectId is invalid or if no data has been grabed up to this point in time.
*/
DLLEXPORT BOOL Kinect_dataSize( KinectId id, unsigned int* depthDataWidth, unsigned int* depthDataHeight, unsigned int* objectDataWidth, unsigned int* objectDataHeight );

/**
Get readings from Robotino's kinect. Do not forget to call Kinect_grab first.
Get the data sizes by calling Kinect_dataSize first.
The following conditions must be met:
\li sizeof(depth_data) >= depthDataWidth * depthDataHeight * sizeof(float)
\li sizeof(object_data) >= objectDataWidth * objectDataHeight * sizeof(float)
The easiest way to meet these conditions is to create arrays containing more elements than a rangefinder will provide readings:
float depth_data[320*240];
float object_data[320*240];
@param id The kinect id.
@param depth_data Depth information. See parameter format.
@param object_data RobotinoSIM provides object IDs here. This can be used to easily find depth readings belonging to the same object.
@param depthDataWidth data width.
@param depthDataHeight data height.
@param objectDataWidth object data width.
@param objectDataHeight object data height.
@param format Format: 0-standard 11bit kinect format, 4-16bit mm values
@param stamp Timestamp
@return Returns TRUE (1) on success. Returns FALSE (0) if the given KinectId is invalid.
@see Kinect_numMeasurements Kinect_grab
*/
DLLEXPORT BOOL Kinect_getReadings(
	KinectId id,
	float* depth_data, float* object_data
	, unsigned int* depthDataWidth, unsigned int* depthDataHeight
	, unsigned int* objectDataWidth, unsigned int* objectDataHeight
	, unsigned int* format, unsigned int* stamp
	);

#endif //_REC_ROBOTINO_API2_C_Kinect_H_
