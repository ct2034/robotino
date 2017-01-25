//  Copyright (C) 2004-2013, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_COMPACTBHA_H_
#define _REC_ROBOTINO_API2_C_COMPACTBHA_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file CompactBHA.h
	\brief In "rec/robotino/api2/c/CompactBHA.h" you can find functions for controlling Robotino's compactBHA.

		Use CompactBHA_construct() to create a new compactBHA object. Associate the compactBHA object with a com object using CompactBHA_setComId().
		Use CompactBHA_setAxis() to drive one axis to the desired position.
		Use CompactBHA_getReadings() to get sensor readings from Robotino's compactBHA.
*/

/** CompactBHAId */
typedef int CompactBHAId;

/** Invalid CompactBHAId is -1 */
#define INVALID_COMPACTBHAID -1

/**
Construct an CompactBHA object
@return Returns the ID of the newly constructed CompactBHA object.
*/
DLLEXPORT CompactBHAId CompactBHA_construct();

/**
Destroy the CompactBHA object assigned to id
@param id The id of the CompactBHA object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given CompactBHAId is invalid.
*/
DLLEXPORT BOOL CompactBHA_destroy( CompactBHAId id );

/**
Associated an CompactBHA object with a communication interface, i.e. binding the CompactBHA to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given CompactBHAId is invalid.
*/
DLLEXPORT BOOL CompactBHA_setComId( CompactBHAId id, ComId comId );

/**
Sets pressure of all bellows
@param id The CompactBHA id.
@param pressures Array of pressures in bar. Size of this array must be 8.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given CompactBHAId is invalid.
*/
DLLEXPORT BOOL CompactBHA_setPressures( CompactBHAId id, const float* pressures );

/**
Turns compressors on and off. If on, they do only run when pressure is too low.
@param id The CompactBHA id.
@param enabled State of compressors.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given CompactBHAId is invalid.
*/
DLLEXPORT BOOL CompactBHA_setCompressorsEnabled( CompactBHAId id, BOOL enabled );

/**
Opens and closes the water drain valve.
@param id The CompactBHA id.
@param open State of the valve.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given CompactBHAId is invalid.
*/
DLLEXPORT BOOL CompactBHA_setWaterDrainValve( CompactBHAId id, BOOL open );

/**
Opens and closes the gripper valve 1.
@param id The CompactBHA id.
@param open State of the valve.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given CompactBHAId is invalid.
*/
DLLEXPORT BOOL CompactBHA_setGripperValve1( CompactBHAId id, BOOL open );

/**
Opens and closes the gripper valve 2.
@param id The CompactBHA id.
@param open State of the valve.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given CompactBHAId is invalid.
*/
DLLEXPORT BOOL CompactBHA_setGripperValve2( CompactBHAId id, BOOL open );

/**
Read current pressure of all bellows
@param id The CompactBHA id.
@param[out] readings Array of pressures in bar. Size of this array must be 8.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given CompactBHAId is invalid.
*/
DLLEXPORT BOOL CompactBHA_pressures( CompactBHAId id, float* readings );

/**
Read pressure sensor
@param id The CompactBHA id.
@return Returns the signal from the pressure sensor. Returns FALSE (0) if the given CompactBHAId is invalid.
*/
DLLEXPORT BOOL CompactBHA_pressureSensor( CompactBHAId id );

/**
Read string potentiometers
@param id The CompactBHA id.
@param[out] readings Array of readings. Size of this array must be 6.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given CompactBHAId is invalid.
*/
DLLEXPORT BOOL CompactBHA_stringPots( CompactBHAId id, float* readings );

/**
Read foil potentiometers
@param id The CompactBHA id.
@return Returns the current reading from the foil pot. Returns 0 if the given CompactBHAId is invalid.
*/
DLLEXPORT float CompactBHA_foilPot( CompactBHAId id );

#endif //_REC_ROBOTINO_API2_C_COMPACTBHA_H_
