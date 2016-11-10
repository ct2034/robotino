#ifndef _REC_ROBOTINO_RPC_CURRENT_CONTROLLER_H_
#define _REC_ROBOTINO_RPC_CURRENT_CONTROLLER_H_

#include "rec/rpc/ClientInfo.h"

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

#include <QHostAddress>

/**
Actor names:
\li none
\li motor
\li digitaloutput
\li relay
\li camera
\li shutdown
\li grappler
\li northstar
\li odometry
\li display
\li all
*/

DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_current_controller, rec::rpc::ClientInfoMap )

DEFINE_PRIMITIVE_PARAM( rec_robotino_rpc_aquire_control, QStringList )
DEFINE_PRIMITIVE_RESULT( rec_robotino_rpc_aquire_control, QStringList )

DEFINE_PRIMITIVE_PARAM( rec_robotino_rpc_release_control, QStringList )
DEFINE_PRIMITIVE_RESULT( rec_robotino_rpc_release_control, QStringList )

#endif //_REC_ROBOTINO_RPC_CURRENT_CONTROLLER_H_
