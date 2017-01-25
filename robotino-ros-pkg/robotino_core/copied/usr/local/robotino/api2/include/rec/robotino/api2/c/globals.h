//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_GLOBALS_H_
#define _REC_ROBOTINO_API2_C_GLOBALS_H_

typedef int BOOL;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#define NULL 0
#endif

#if defined WIN32 || defined _WIN32
  #ifdef __cplusplus
    #define DLLEXPORT extern "C" __declspec(dllexport)
  #else
    #define DLLEXPORT __declspec(dllexport)
  #endif
#else
  #ifdef __cplusplus
    #define DLLEXPORT extern "C" __attribute__ ((visibility ("default")))
  #else
    #define DLLEXPORT __attribute__ ((visibility ("default")))
  #endif
#endif

#endif //_REC_ROBOTINO_API2_C_GLOBALS_H_

/**  \mainpage rec::robotino::api2 C API documentation

This is a C wrapper to the <A HREF="../rec_robotino_api2/index.html">rec::robotino::api2 C++</A> API for Robotino.

For instructions how to install the API2 and how to build C programs using the API2 C interface refer to the <A HREF="http://wiki.openrobotino.org/index.php?title=API2">Robotino Wiki</A>.
*/
