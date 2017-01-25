#ifndef _REC_ROBOTINO_RPC_PROCESS_H_
#define _REC_ROBOTINO_RPC_PROCESS_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/Primitive.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"
#include "rec/robotino/rpc/ProcessStatus.h"
#include "rec/robotino/rpc/ProcessOutput.h"

//*********************** launch
BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_process_launch_t, 1.0 )
ADD_MEMBER( command )
ADD_MEMBER( parameters )
ADD_MEMBER( workingdirectory )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_STRING_MEMBER( command )
DECLARE_PRIMITIVE_MEMBER( QStringList, parameters )
DECLARE_STRING_MEMBER( workingdirectory )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_PARAM( rec_robotino_rpc_process_launch_t, rec_robotino_rpc_process_launch )
DEFINE_PRIMITIVE_RESULT( rec_robotino_rpc_process_launch, int )
//*********************** launch

//*********************** terminate
DEFINE_PRIMITIVE_PARAM( rec_robotino_rpc_process_terminate, int ) /*parameter is the process id*/
DEFINE_PRIMITIVE_RESULT( rec_robotino_rpc_process_terminate, int )
//*********************** terminate

//*********************** kill
DEFINE_PRIMITIVE_PARAM( rec_robotino_rpc_process_kill, int ) /*parameter is the process id*/
DEFINE_PRIMITIVE_RESULT( rec_robotino_rpc_process_kill, int )
//*********************** kill

//************************ getids
DEFINE_EMPTY_PARAM( rec_robotino_rpc_process_getids )
DEFINE_PRIMITIVE_RESULT( rec_robotino_rpc_process_getids, QVector< int > )
//************************ getids


//************************ status
DEFINE_PRIMITIVE_PARAM( rec_robotino_rpc_process_getstatus, int ) /*parameter is the process id*/
DEFINE_PRIMITIVE_RESULT( rec_robotino_rpc_process_getstatus, rec::robotino::rpc::ProcessStatus )

DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_process_status, rec::robotino::rpc::ProcessStatus )
//************************ status

//************************ output
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_process_output, rec::robotino::rpc::ProcessOutput )
//************************ output

#endif //_REC_ROBOTINO_RPC_process_H_
