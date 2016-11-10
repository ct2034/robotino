/*
Copyright (c) 2011, REC Robotics Equipment Corporation GmbH, Planegg, Germany
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.
- Neither the name of the REC Robotics Equipment Corporation GmbH nor the names of
  its contributors may be used to endorse or promote products derived from this software
  without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _REC_RPC_ROSHELPER_H_
#define _REC_RPC_ROSHELPER_H_

#include "rec/rpc/defines.h"
#include "rec/rpc/utils.h"

#include <QtCore>
#include <QMetaType>
#include <QPixmap>
#include <QImage>
#include <QRgb>
#include <QVector>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>

#include <nav_msgs/OccupancyGrid.h>

namespace rec
{
	namespace rpc
	{
		class MapInfo
		{
		public:
			MapInfo()
				: resolution( 0.0f )
				, frame_id( "/map" )
			{
				offset[0] = 0.0f;
				offset[1] = 0.0f;
			}

			MapInfo( float resolution_, float offsetX, float offsetY, std::string frame_id_ = "/map" )
				: resolution( resolution_ )
				, frame_id( "/map" )
			{
				offset[0] = offsetX;
				offset[1] = offsetY;
			}

			MapInfo( const nav_msgs::OccupancyGridConstPtr& msg, std::string frame_id_ = "/map" )
			{
				resolution = msg->info.resolution;
				offset[0] = -((int)msg->info.width) - (msg->info.origin.position.x / resolution);	// remember: horizontal mirroring!
				offset[1] = msg->info.origin.position.y / resolution;
				frame_id = frame_id_;
			}

			bool isEmpty() const { return 0.0f == resolution; }

			void clear() { resolution = 0.0f; }

			float resolution;
			float offset[2];
			std::string frame_id;
		};

		static REC_RPC_FUNCTION_IS_NOT_USED QPointF getPoint( const MapInfo& mapInfo, const tf::Pose& p )
		{
			QPointF point;
			point.setX( -1.0 / mapInfo.resolution * p.getOrigin()[0] - mapInfo.offset[0] );
			point.setY( 1.0 / mapInfo.resolution * p.getOrigin()[1] - mapInfo.offset[1] );
			return point;
		}

		static REC_RPC_FUNCTION_IS_NOT_USED QPointF getPoint( const MapInfo& mapInfo, const geometry_msgs::Point& p )
		{
			QPointF point;
			point.setX( -1.0 / mapInfo.resolution * p.x - mapInfo.offset[0] );
			point.setY( 1.0 / mapInfo.resolution * p.y - mapInfo.offset[1] );
			return point;
		}

		static REC_RPC_FUNCTION_IS_NOT_USED QVector< QPointF > getPoints( tf::TransformListener* tf, const MapInfo& mapInfo, const sensor_msgs::PointCloud& pointCloudIn )
		{
			QVector< QPointF > points;

			try
			{
				tf::StampedTransform transform;
				tf->lookupTransform( mapInfo.frame_id, pointCloudIn.header.frame_id, ros::Time(0), transform );

				points.resize( pointCloudIn.points.size() );

				for( unsigned int i=0; i<pointCloudIn.points.size(); ++i )
				{
					tf::Pose p;
					p.setOrigin( btVector3( pointCloudIn.points[i].x, pointCloudIn.points[i].y, 0 ) );

					p = transform * p;

					points[i] = getPoint( mapInfo, p );
				}
			}
			catch(tf::LookupException& ex) {
				ROS_ERROR("No Transform available Error: %s\n", ex.what());
			}
			catch(tf::ConnectivityException& ex) {
				ROS_ERROR("Connectivity Error: %s\n", ex.what());
			}
			catch(tf::ExtrapolationException& ex) {
				ROS_ERROR("Extrapolation Error: %s\n", ex.what());
			}

			return points;
		}

		static REC_RPC_FUNCTION_IS_NOT_USED double getYaw( const tf::Pose& p )
		{
			btScalar useless_pitch, useless_roll, yaw;
			p.getBasis().getRPY( useless_roll, useless_pitch, yaw );

			double x = cos( yaw );
			double y = sin( yaw );

			return rad2deg( atan2( y, -x ) );
		}

		static REC_RPC_FUNCTION_IS_NOT_USED bool poseStampedToMap( tf::TransformListener* tf, const MapInfo& mapInfo, const geometry_msgs::PoseStamped& pose, QPointF* point, double* rotation_deg )
		{
			try
			{
				tf::StampedTransform transform;
				tf->lookupTransform( mapInfo.frame_id, pose.header.frame_id, ros::Time(0), transform );

				tf::Pose p;
				p.setOrigin( btVector3( pose.pose.position.x, pose.pose.position.y, pose.pose.position.z ) );
				p.setRotation( btQuaternion( pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w ) );

				p = transform * p;

				if( rotation_deg )
				{
					*rotation_deg = getYaw( p );
				}

				*point = getPoint( mapInfo, p );
				return true;
			}
			catch(tf::LookupException& ex) {
				ROS_ERROR("No Transform available Error: %s\n", ex.what());
				return false;
			}
			catch(tf::ConnectivityException& ex) {
				ROS_ERROR("Connectivity Error: %s\n", ex.what());
				return false;
			}
			catch(tf::ExtrapolationException& ex) {
				ROS_ERROR("Extrapolation Error: %s\n", ex.what());
				return false;
			}

			return false;
		}

		static REC_RPC_FUNCTION_IS_NOT_USED bool poseToMap( tf::TransformListener* tf, const MapInfo& mapInfo, const geometry_msgs::Pose& pose, const std::string& sourceFrame, const ros::Time& time, QPointF* point, double* rotation_deg  )
		{
			geometry_msgs::PoseStamped poseStamped;
			poseStamped.header.frame_id = sourceFrame;
			poseStamped.header.stamp = time;
			poseStamped.pose = pose;

			if( poseStampedToMap( tf, mapInfo, poseStamped, point, rotation_deg ) )
			{
				return true;
			}
			else
			{
				return false;
			}
		}

		static REC_RPC_FUNCTION_IS_NOT_USED bool toMap( tf::TransformListener* tf, const MapInfo& mapInfo, const geometry_msgs::Point& position, const std::string& sourceFrame, const ros::Time& time, QPointF* point )
		{
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = sourceFrame;
			pose.header.stamp = time;
			pose.pose.position = position;
			pose.pose.orientation.w = 1.0;

			if( poseStampedToMap( tf, mapInfo, pose, point, NULL ) )
			{
				return true;
			}
			else
			{
				return false;
			}
		}

		static REC_RPC_FUNCTION_IS_NOT_USED bool toMap( tf::TransformListener* tf, const MapInfo& mapInfo, const geometry_msgs::Pose& pose, const std::string& sourceFrame, const ros::Time& time, QPointF* point, double* rotation_deg  )
		{
			geometry_msgs::PoseStamped poseStamped;
			poseStamped.header.frame_id = sourceFrame;
			poseStamped.header.stamp = time;
			poseStamped.pose = pose;

			if( poseStampedToMap( tf, mapInfo, poseStamped, point, rotation_deg ) )
			{
				return true;
			}
			else
			{
				return false;
			}
		}

		static REC_RPC_FUNCTION_IS_NOT_USED geometry_msgs::Pose toPose( const MapInfo& mapInfo, const QPointF& point, double rot_degrees )
		{
			geometry_msgs::Pose pose;
			pose.position.x = ( - mapInfo.resolution * ( point.x() + mapInfo.offset[0] ) );
			pose.position.y = ( mapInfo.resolution * ( point.y() + mapInfo.offset[1] ) );
			pose.position.z = 0;

			double rot = deg2rad( rot_degrees );
			double x = cos( rot );
			double y = sin( rot );
			rot = atan2( y, -x );

			tf::Quaternion q = tf::createQuaternionFromYaw( rot );

			pose.orientation.x = q.x();
			pose.orientation.y = q.y();
			pose.orientation.z = q.z();
			pose.orientation.w = q.w();

			return pose;
		}

		static REC_RPC_FUNCTION_IS_NOT_USED QImage fromOccupancyGrid( const nav_msgs::OccupancyGridConstPtr& msg )
		{
			QImage imgBuffer2( (unsigned char*)&(*msg->data.begin()), msg->info.width, msg->info.height, msg->info.width * sizeof( int8_t ), QImage::Format_Indexed8 );
			QImage imgBuffer( imgBuffer2.copy() );
			QVector<QRgb> colorTable;
			unsigned int i = 0;
			for( i = 0; i < 101; ++i )
			{
				QRgb col = 0xa0000000;
				int val = (100 - i) / 100 * 255;
				col |= val | (val << 8) | (val << 16);
				colorTable.push_back(col);
			}
			for( /*i*/; i < 255; ++i )
			{
				colorTable.push_back( 0xFFFF0000 );	// red for warning
			}
			colorTable.push_back( 0xFF888888 ); // grey for 255 = -1 = "unknown"
			imgBuffer.setColorTable( colorTable );

			return imgBuffer;
		}
	}
}

Q_DECLARE_METATYPE( rec::rpc::MapInfo )

#endif //_REC_RPC_ROSHELPER_H_
