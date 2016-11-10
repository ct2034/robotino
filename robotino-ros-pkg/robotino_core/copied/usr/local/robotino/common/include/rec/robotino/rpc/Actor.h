//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_RPC_ACTOR_H_
#define _REC_ROBOTINO_RPC_ACTOR_H_

#include "rec/robotino/rpc/defines.h"
#include <QString>
#include <QStringList>

namespace rec
{
	namespace robotino
	{
		namespace rpc
		{
			typedef enum
			{
				NoActor = 0x0,
				MotorActor = 0x1,
				DigitalOutputActor = 0x2,
				RelayActor = 0x4,
				Camera0Actor = 0x8,
				Camera1Actor = 0x10,
				Camera2Actor = 0x20,
				Camera3Actor = 0x40,
				ShutdownActor = 0x80,
				GrapplerActor = 0x100,
				NorthStarActor = 0x200,
				OdometryActor = 0x400,
				DisplayActor = 0x800,
				CBHAActor = 0x1000,
				AllActors = 0x80000000
			} Actor;

			static REC_ROBOTINO_RPC_FUNCTION_IS_NOT_USED QList< Actor > seperateActor( Actor actor )
			{
				QList<Actor> l;
				for( int i=0; i<32; ++i )
				{
					if( actor & 1<<i )
					{
						l << (Actor)(1<<i);
					}
				}
				return l;
			}

			static REC_ROBOTINO_RPC_FUNCTION_IS_NOT_USED QStringList actorToName( Actor actor )
			{
				QStringList l;

				if( actor & MotorActor ) l << "motor";
				if( actor & DigitalOutputActor ) l << "digitaloutput";
				if( actor & RelayActor ) l << "relay";
				if( actor & Camera0Actor ) l << "camera0";
				if( actor & Camera1Actor ) l << "camera1";
				if( actor & Camera2Actor ) l << "camera2";
				if( actor & Camera3Actor ) l << "camera3";
				if( actor & ShutdownActor ) l << "shutdown";
				if( actor & GrapplerActor ) l << "grappler";
				if( actor & NorthStarActor ) l << "northstar";
				if( actor & OdometryActor ) l << "odometry";
				if( actor & DisplayActor ) l << "display";
				if( actor & CBHAActor ) l << "cbha";
				if( actor & AllActors ) l << "all";

				if( l.isEmpty() ) l << "none";

				return l;
			}

			static REC_ROBOTINO_RPC_FUNCTION_IS_NOT_USED Actor actorFromName( const QStringList& names )
			{
				unsigned int a = NoActor;

				Q_FOREACH( const QString& name, names )
				{
					if( "motor" == name )
					{
						a |= MotorActor;
					}
					else if( "digitaloutput" == name )
					{
						a |= DigitalOutputActor;
					}
					else if( "relay" == name )
					{
						a |= RelayActor;
					}
					else if( "camera0" == name )
					{
						a |= Camera0Actor;
					}
					else if( "camera1" == name )
					{
						a |= Camera1Actor;
					}
					else if( "camera2" == name )
					{
						a |= Camera2Actor;
					}
					else if( "camera3" == name )
					{
						a |= Camera3Actor;
					}
					else if( "shutdown" == name )
					{
						a |= ShutdownActor;
					}
					else if( "grappler" == name )
					{
						a |= GrapplerActor;
					}
					else if( "northstar" == name )
					{
						a |= NorthStarActor;
					}
					else if( "odometry" == name )
					{
						a |= OdometryActor;
					}
					else if( "display" == name )
					{
						a |= DisplayActor;
					}
					else if( "cbha" == name )
					{
						a |= CBHAActor;
					}
					else if( "all" == name )
					{
						a = AllActors;
						break;
					}
				}

				return (Actor)a;
			}
		}
	}
}

#endif //_REC_ROBOTINO_RPC_ACTOR_H_
