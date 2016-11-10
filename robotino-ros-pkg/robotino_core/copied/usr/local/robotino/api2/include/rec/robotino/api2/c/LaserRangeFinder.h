//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_LASERRANGEFINDER_H_
#define _REC_ROBOTINO_API2_C_LASERRANGEFINDER_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file LaserRangeFinder.h
    \brief In "rec/robotino/api2/c/LaserRangeFinder.h" you can find functions for reading Robotino's laser rangefinder.

		Use LaserRangeFinder_construct() to create a new rangefinder object. Associate the rangefinder object with a com object using LaserRangeFinder_setComId().
		Use LaserRangeFinder_getReadings() to get sensor readings from Robotino's rangefinder.
*/

/** LaserRangeFinderId */
typedef int LaserRangeFinderId;

/** Invalid LaserRangeFinderId is -1 */
#define INVALID_LASERRANGEFINDERID -1

/**
Construct an LaserRangeFinder object
@return Returns the ID of the newly constructed LaserRangeFinder object.
*/
DLLEXPORT LaserRangeFinderId LaserRangeFinder_construct();

/**
Destroy the LaserRangeFinder object assigned to id
@param id The id of the LaserRangeFinder object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given LaserRangeFinderId is invalid.
*/
DLLEXPORT BOOL LaserRangeFinder_destroy( LaserRangeFinderId id );

/**
Associated an LaserRangeFinder object with a communication interface, i.e. binding the LaserRangeFinder to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given LaserRangeFinderId is invalid.
*/
DLLEXPORT BOOL LaserRangeFinder_setComId( LaserRangeFinderId id, ComId comId );

/**
Grab image.
@param id The rangefinder id.
@return Returns TRUE (1) if a new sensor readings are available since the last call of LaserRangeFinder_grab. Returns FALSE (0) otherwise.
*/
DLLEXPORT BOOL LaserRangeFinder_grab( LaserRangeFinderId id );

/**
Number of range and intensity measurements aquired by grab.
@param id The rangefinder id.
@param numRangeMeasurements Number of range measurements.
@param numIntensityMeasurements Number of intensity measurements.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given LaserRangeFinderId is invalid or if no readings have been grabed up to this point in time.
*/
DLLEXPORT BOOL LaserRangeFinder_numMeasurements( LaserRangeFinderId id, unsigned int* numRangeMeasurements, unsigned int* numIntensityMeasurements );

/**
Get readings from Robotino's rangefinder. Do not forget to call LaserRangeFinder_grab first.
Get the number of measurements by calling LaserRangeFinder_numMeasurements() first.
The following conditions must be met:
\li sizeof(ranges) >= numRangeMeasurements * sizeof(float)
\li sizeof(intensities) >= numIntensityMeasurements * sizeof(float)
The easiest way to meet these conditions is to create arrays containing more elements than a rangefinder will provide readings:
float range[1000];
float intensities[1000];
@param id The rangefinder id.
@param seq The sequenze number.
@param stamp Time stamp.
@param angle_min Minimum angle in radians.
@param angle_max Maximum angle in radians.
@param angle_increment Angle between two scans in radians.
@param time_increment Time between two scans in s.
@param scan_time Time to scan the field of view in s.
@param range_min Minimum range in meters.
@param range_max Maximum range in meters.
@param ranges The range measurements in meters.
@param numRanges Number of range measurements
@param intensities The intensity measurements.
@param numIntensities Number of intensity measurements
@return Returns TRUE (1) on success. Returns FALSE (0) if the given LaserRangeFinderId is invalid.
@see LaserRangeFinder_numMeasurements LaserRangeFinder_grab
*/
DLLEXPORT BOOL LaserRangeFinder_getReadings(
	LaserRangeFinderId id,
	unsigned int* seq,
	unsigned int* stamp,
	float* angle_min,
	float* angle_max,
	float* angle_increment,
	float* time_increment,
	float* scan_time,
	float* range_min,
	float* range_max,
	float* ranges,
	unsigned int* numRanges,
	float* intensities,
	unsigned int* numIntensities
	);

#endif //_REC_ROBOTINO_API2_C_LASERRANGEFINDER_H_
