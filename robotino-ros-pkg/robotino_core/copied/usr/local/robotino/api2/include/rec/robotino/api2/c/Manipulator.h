//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_MANIPULATOR_H_
#define _REC_ROBOTINO_API2_C_MANIPULATOR_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file Manipulator.h
    \brief In "rec/robotino/api2/c/Manipulator.h" you can find functions for controlling Robotino's manipulator.

		Use Manipulator_construct() to create a new manipulator object. Associate the manipulator object with a com object using Manipulator_setComId().
		Use Manipulator_setAxis() to drive one axis to the desired position.
		Use Manipulator_getReadings() to get sensor readings from Robotino's manipulator.
*/

/** ManipulatorId */
typedef int ManipulatorId;

/** Invalid ManipulatorId is -1 */
#define INVALID_MANIPULATORID -1

/**
Construct an Manipulator object
@return Returns the ID of the newly constructed Manipulator object.
*/
DLLEXPORT ManipulatorId Manipulator_construct();

/**
Destroy the Manipulator object assigned to id
@param id The id of the Manipulator object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ManipulatorId is invalid.
*/
DLLEXPORT BOOL Manipulator_destroy( ManipulatorId id );

/**
Associated an Manipulator object with a communication interface, i.e. binding the Manipulator to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ManipulatorId is invalid.
*/
DLLEXPORT BOOL Manipulator_setComId( ManipulatorId id, ComId comId );

/**
Grab image.
@param id The manipulator id.
@return Returns TRUE (1) if a new sensor readings are available since the last call of Manipulator_grab. Returns FALSE (0) otherwise.
*/
DLLEXPORT BOOL Manipulator_grab( ManipulatorId id );

/**
Get readings from Robotino's manipulator. Do not forget to call Manipulator_grab first.
Conditions to be met:
\li sizeof(angles) must be 9*sizeof(float)
\li sizeof(speeds) must be 9*sizeof(float)
\li sizeof(cwAxesLimits) must be 9*sizeof(float)
\li sizeof(ccwAxesLimits) must be 9*sizeof(float)
Simply create array like
float angles[9];
float speeds[9];
float cwAxesLimits[9];
float ccwAxesLimits[9];
@param id The manipulator id.
@param seq The sequenze number.
@param angles Array storing the current axes positions in degrees.
@param numAngles The number of elemets stored in angles. This is equal to the number of axes.
@param speeds Array storing the current axes speeds in rpm.
@param numSpeeds The number of elemets stored in speeds. This is equal to the number of axes.
@param errors Error code of axes.
@param numErrors The number of elemets stored in errors. This is equal to the number of axes.
@param motors_enabled Is 1 if the motors are enabled. Otherwise 0.
@param store_current_position Is 1 if the store position button is pressed. Otherwise 0.
@param cwAxesLimits Array storing the current axes limits in degrees.
@param numCwAxesLimits The number of elemets stored in cwAxesLimits. If axes limits had been received this is equal to the number of axes. Otherwise 0.
@param ccwAxesLimits Array storing the current axes limits in degrees.
@param numCcwAxesLimits The number of elemets stored in ccwAxesLimits. If axes limits had been received this is equal to the number of axes. Otherwise 0.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ManipulatorId is invalid.
@see Manipulator_grab
*/
DLLEXPORT BOOL Manipulator_getReadings(
	ManipulatorId id,
	unsigned int* seq,
	float* angles,
	unsigned int* numAngles,
	float* speeds,
	unsigned int* numSpeeds,
	int* errors,
	unsigned int* numErrors,
	int* motors_enabled,
	int* store_current_position,
	float* cwAxesLimits,
	unsigned int* numCwAxesLimits,
	float* ccwAxesLimits,
	unsigned int* numCcwAxesLimits
	);

/**
Set position and speed of one axis.
@param id The manipulator id.
@param axis Axis number. Axes are counted starting with 0.
@param angle This is the position set-point in deg
@param speed This is the speed set-point in rpm
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ManipulatorId is invalid.
*/
DLLEXPORT BOOL Manipulator_setAxis( ManipulatorId id, unsigned int axis, float angle, float speed );

/**
Set position and speed of one axis.
@param id The manipulator id.
@param angles Array containing the set-points in deg
@param numAngles Number of elements in angles
@param speeds Array containing the speed set-points in rpm
@param numSpeeds Number of elements in speeds
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ManipulatorId is invalid.
*/
DLLEXPORT BOOL Manipulator_setAxes( ManipulatorId id, const float* angles, unsigned int numAngles, const float* speeds, unsigned int numSpeeds );

/**
Enabled/Disable power of one channel
@param id The manipulator id.
@param channel The channel at which to enable/disable power.
@param enable If 1 power is enabled. 0 to disable power
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ManipulatorId is invalid.
*/
DLLEXPORT BOOL Manipulator_setPowerEnabled( ManipulatorId id, unsigned int channel, int enable );

/**
Toggle the power state of all servos' motors.
@param id The manipulator id.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ManipulatorId is invalid.
*/
DLLEXPORT BOOL Manipulator_toggleTorque( ManipulatorId id );

#endif //_REC_ROBOTINO_API2_C_MANIPULATOR_H_
