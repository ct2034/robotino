//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_MOTOR_H_
#define _REC_ROBOTINO_API2_C_MOTOR_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file Motor.h
\brief In "rec/robotino/api2/c/Motor.h" you can find functions for reading Robotino's bumper.

Use Motor_construct() to create a new motor object. Associate the motor object with a com object using Motor_setComId().
Use Motor_setSetPointSpeed() to set the motor's speed.
*/

/** MotorId */
typedef int MotorId;

/** Invalid MotorId is -1 */
#define INVALID_MOTORID -1

/**
Construct an motor object
@param number	number of this motor. Range [0; numMotors()-1].
@return Returns the ID of the newly constructed motor object.
*/
DLLEXPORT MotorId Motor_construct( unsigned int number );

/**
Destroy the motor object assigned to id
@param id The id of the motor object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given MotorId is invalid.
*/
DLLEXPORT BOOL Motor_destroy( MotorId id );

/**
Associated a motor object with a communication interface, i.e. binding the motor to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given MotorId or ComId is invalid.
*/
DLLEXPORT BOOL Motor_setComId( MotorId id, ComId comId );

/**
 * Sets the number of this motor.
 *
 * @param id The id of the motor object to be set
 * @param number	number of this motor
 * @throws	RobotinoException if the current communication object is invalid.
 */
DLLEXPORT BOOL Motor_setMotorNumber( MotorId id, unsigned int number );

/**
@return Returns the number of drive motors on Robotino
*/
DLLEXPORT unsigned int numMotors();

/**
Sets the setpoint speed of this motor.
@param id The id of the motor object.
@param speed	Set point speed in rpm.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given MotorId is invalid.
*/
DLLEXPORT BOOL Motor_setSetPointSpeed( MotorId id, float speed );

/**
Resets the position of this motor.
@param id The id of the motor object.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given MotorId is invalid.
*/
DLLEXPORT BOOL Motor_resetPosition( MotorId id );

/**
Controls the brakes of this motor.
@param id The id of the motor object.
@param brake	If set to TRUE, this will activate the brake. If set to FALSE, the brake is released.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given MotorId is invalid.
*/
DLLEXPORT BOOL Motor_setBrake( MotorId id, BOOL brake );

/**
Sets the proportional, integral and  differential constant of the PID controller.
@param id The id of the motor object.
@param kp proportional constant. Robotino v2: Typical value 200. Robotino v3: Typical value is 0.1.
@param ki integral constant. Robotino v2: Typical value 10. Robotino v3: Typical value is 0.005.
@param kd differential constant. Robotino v2: Typical value 0. Robotino v3: Not used.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given MotorId is invalid.
*/
DLLEXPORT BOOL Motor_setPID( MotorId id, float kp, float ki, float kd );

/**
Retrieves the actual speed of this motor.
@param id The id of the motor object.
@return	Speed in rpm.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given MotorId is invalid.
*/
DLLEXPORT float Motor_actualSpeed( MotorId id );

/**
Retrieves the actual position of this motor.
@param id The id of the motor object.
@return actual position
*/
DLLEXPORT int Motor_actualPosition( MotorId id );

/**
Retrieves the current of this motor.
@param id The id of the motor object.
@return motor current in A.
*/
DLLEXPORT float Motor_motorCurrent( MotorId id );

/**
The current is measured by a 10 bit adc and is not converted into A.
@param id The id of the motor object.
@return The current delivered by to this motor. Range from 0 to 1023.
*/
DLLEXPORT float Motor_rawCurrentMeasurment( MotorId id );

#endif //_REC_ROBOTINO_API2_C_MOTOR_H_
