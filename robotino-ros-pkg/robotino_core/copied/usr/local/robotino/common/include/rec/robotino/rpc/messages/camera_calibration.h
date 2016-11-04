#ifndef _REC_ROBOTINO_RPC_CAMERA_CALIBRATION_H_
#define _REC_ROBOTINO_RPC_CAMERA_CALIBRATION_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/String.h"

#include <QVector>

DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_camera0_calibration, QVector<double> )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_camera1_calibration, QVector<double> )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_camera2_calibration, QVector<double> )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_camera3_calibration, QVector<double> )

#endif //_REC_ROBOTINO_RPC_CAMERA_CALIBRATION_H_
