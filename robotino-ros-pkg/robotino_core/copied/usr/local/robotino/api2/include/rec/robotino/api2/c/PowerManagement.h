//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_POWERMANAGEMENT_H_
#define _REC_ROBOTINO_API2_C_POWERMANAGEMENT_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file PowerManagement.h
    \brief In "rec/robotino/api2/c/PowerManagement.h" you can find functions for reading Robotino's bumper.

		Use PowerManagement_construct() to create a new power management object. Associate the power management object with a com object using PowerManagement_setComId().
		Use PowerManagement_open() to open the power management.
		Use PowerManagement_close() to open the power management.
		Use PowerManagement_isOpened() to check if the power management is opened.
		Use PowerManagement_isClosed() to check if the power management is closed.
*/

/** PowerManagementId */
typedef int PowerManagementId;

/** Invalid PowerManagementId is -1 */
#define INVALID_POWERMANAGEMENTID -1

/**
Construct an power management object
@return Returns the ID of the newly constructed power management object.
*/
DLLEXPORT PowerManagementId PowerManagement_construct();

/**
Destroy the power management object assigned to id
@param id The id of the power management object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given PowerManagementId is invalid.
*/
DLLEXPORT BOOL PowerManagement_destroy( PowerManagementId id );

/**
Associated a power management object with a communication interface, i.e. binding the power management to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given PowerManagementId or ComId is invalid.
*/
DLLEXPORT BOOL PowerManagement_setComId( PowerManagementId id, ComId comId );

/**
Retrieves the current power drain.
@param id The id of the power management object.
@return	The power drain in mA.
*/
DLLEXPORT float PowerManagement_current( PowerManagementId id );

/**
Retrieves the battery voltage.
@param id The id of the power management object.
@return	Battery voltage in V.
*/
DLLEXPORT float PowerManagement_voltage( PowerManagementId id );

#endif //_REC_ROBOTINO_API2_C_POWERMANAGEMENT_H_
