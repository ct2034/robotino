//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_SERVER_DEFINES_H_
#define _REC_ROBOTINO_SERVER_DEFINES_H_

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/*
 * Defines that help to correctly specify dllexport and dllimport on
 * Windows platforms.
 */
//#ifdef WIN32
//#  ifdef rec_robotino_server_EXPORTS
//#    define REC_ROBOTINO_SERVER_EXPORT __declspec(dllexport)
//#  else
//#    define REC_ROBOTINO_SERVER_EXPORT __declspec(dllimport)
//#  endif // rec_robotino_server_EXPORTS
//#else // WIN32
//#  define REC_ROBOTINO_SERVER_EXPORT __attribute__ ((visibility ("default")))
//#endif // WIN32

#define REC_ROBOTINO_SERVER_EXPORT

#endif // _REC_ROBOTINO_SERVER_DEFINES_H_
