//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_ODOMETRY_H_
#define _REC_ROBOTINO_API2_C_ODOMETRY_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file Odometry.h
    \brief In "rec/robotino/api2/c/Odometry.h" you can find functions for reading Robotino's bumper.

		Use Odometry_construct() to create a new odometry object. Associate the odometry object with a com object using Odometry_setComId().
		Use Odometry_open() to open the odometry.
		Use Odometry_close() to open the odometry.
		Use Odometry_isOpened() to check if the odometry is opened.
		Use Odometry_isClosed() to check if the odometry is closed.
*/

/** OdometryId */
typedef int OdometryId;

/** Invalid OdometryId is -1 */
#define INVALID_ODOMETRYID -1

/**
Construct an odometry object
@return Returns the ID of the newly constructed odometry object.
*/
DLLEXPORT OdometryId Odometry_construct();

/**
Destroy the odometry object assigned to id
@param id The id of the odometry object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given OdometryId is invalid.
*/
DLLEXPORT BOOL Odometry_destroy( OdometryId id );

/**
Associated a odometry object with a communication interface, i.e. binding the odometry to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given OdometryId or ComId is invalid.
*/
DLLEXPORT BOOL Odometry_setComId( OdometryId id, ComId comId );

/**
@param id The id of the odometry object to be destroyed
@return Global x position of Robotino in mm.
*/
DLLEXPORT float Odometry_x( OdometryId id );

/**
@param id The id of the odometry object to be destroyed
@return Global y position of Robotino in mm.
*/
DLLEXPORT float Odometry_y( OdometryId id );

/**
@param id The id of the odometry object to be destroyed
@return Global orientation of Robotino in degree.
*/
DLLEXPORT float Odometry_phi( OdometryId id );

/**
* Set Robotino's odoemtry to the given coordinates
@param id The id of the odometry object to be destroyed
@param x Global x position in mm
@param y Global y position in mm
@param phi Global phi orientation in degrees
@return Returns TRUE (1) on success. Returns FALSE (0) if the given OdometryId is invalid.
*/
DLLEXPORT BOOL Odometry_set( OdometryId id, float x, float y, float phi );

#endif //_REC_ROBOTINO_API2_C_ODOMETRY_H_
