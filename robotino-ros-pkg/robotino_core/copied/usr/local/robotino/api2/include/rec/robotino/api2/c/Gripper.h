//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_GRIPPER_H_
#define _REC_ROBOTINO_API2_C_GRIPPER_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file Gripper.h
    \brief In "rec/robotino/api2/c/Gripper.h" you can find functions for reading Robotino's bumper.

		Use Gripper_construct() to create a new gripper object. Associate the gripper object with a com object using Gripper_setComId().
		Use Gripper_open() to open the gripper.
		Use Gripper_close() to open the gripper.
		Use Gripper_isOpened() to check if the gripper is opened.
		Use Gripper_isClosed() to check if the gripper is closed.
*/

/** GripperId */
typedef int GripperId;

/** Invalid GripperId is -1 */
#define INVALID_GRIPPERID -1

/**
Construct an gripper object
@return Returns the ID of the newly constructed gripper object.
*/
DLLEXPORT GripperId Gripper_construct();

/**
Destroy the gripper object assigned to id
@param id The id of the gripper object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given GripperId is invalid.
*/
DLLEXPORT BOOL Gripper_destroy( GripperId id );

/**
Associated a gripper object with a communication interface, i.e. binding the gripper to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given GripperId or ComId is invalid.
*/
DLLEXPORT BOOL Gripper_setComId( GripperId id, ComId comId );

/**
Open gripper.
*/
DLLEXPORT BOOL Gripper_open( GripperId id );

/**
Close gripper.
*/
DLLEXPORT BOOL Gripper_close( GripperId id );

/**
@return Returns true if gripper is opened. False otherwise.
@see isClosed
*/
DLLEXPORT BOOL Gripper_isOpened( GripperId id );

/**
@return Returns true if gripper is closed. False otherwise.
@see isOpened
*/
DLLEXPORT BOOL Gripper_isClosed( GripperId id );

#endif //_REC_ROBOTINO_API2_C_GRIPPER_H_
