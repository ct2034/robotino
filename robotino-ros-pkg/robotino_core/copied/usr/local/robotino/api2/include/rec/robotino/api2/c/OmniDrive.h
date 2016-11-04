//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_OMNIDRIVE_H_
#define _REC_ROBOTINO_API2_C_OMNIDRIVE_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file OmniDrive.h
    \brief In "rec/robotino/api2/c/OmniDrive.h" you can find functions for manipulating Robotino's omnidrive.

		Use OmniDrive_construct() to create a new omnidrive object. Associate the omnidrive object with a com object using OmniDrive_setComId().
		Use OmniDrive_setVelocity() to drive Robotino.
*/

/** OmniDriveId */
typedef int OmniDriveId;

/** Invalid OmniDriveId is -1 */
#define INVALID_OMNIDRIVEID -1

/**
Construct an OmniDrive object
@return Returns the ID of the newly constructed OmniDrive object.
*/
DLLEXPORT OmniDriveId OmniDrive_construct();

/**
Destroy the OmniDrive object assigned to id
@param id The id of the OmniDrive object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given OmniDriveId is invalid.
*/
DLLEXPORT BOOL OmniDrive_destroy( OmniDriveId id );

/**
Associated an OmniDrive object with a communication interface, i.e. binding the OmniDrive to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given OmniDriveId or ComId is invalid.
*/
DLLEXPORT BOOL OmniDrive_setComId( OmniDriveId id, ComId comId );

/**
Drive Robotino associated with id
@param id The omnidrive id
@param vx		Velocity in x-direction in m/s
@param vy		Velocity in y-direction in m/s
@param omega	Angular velocity in rad/s
@return Returns TRUE (1) on success. Returns FALSE (0) if the given OmniDriveId is invalid.
@remark This function should be called about every 100ms.
*/
DLLEXPORT BOOL OmniDrive_setVelocity( OmniDriveId id, float vx, float vy, float omega );

/**
Project the velocity of the robot in cartesian coordinates to single motor speeds.
@param id The omnidrive id
@param m1		The resulting speed of motor 1 in rpm
@param m2		The resulting speed of motor 2 in rpm
@param m3		The resulting speed of motor 3 in rpm
@param vx		Velocity in x-direction in mm/s
@param vy		Velocity in y-direction in mm/s
@param omega	Angular velocity in deg/s
@return Returns TRUE (1) on success. Returns FALSE (0) if the given OmniDriveId is invalid.
*/
DLLEXPORT BOOL OmniDrive_project( OmniDriveId id, float* m1, float* m2, float* m3, float vx, float vy, float omega );

/**
For debugging only.
*/
DLLEXPORT int OmniDrive_num_objects( void );

#endif //_REC_ROBOTINO_API2_C_OMNIDRIVE_H_
