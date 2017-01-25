#ifndef _REC_ROBOTINO_NAVIGATION_POSITIONDRIVER_H_
#define _REC_ROBOTINO_NAVIGATION_POSITIONDRIVER_H_

#include <QtCore>

namespace rec
{
	namespace robotino
	{
		namespace navigation
		{
			class PositionDriver
			{
			public:
				typedef enum
				{
					NoMovement,
					Drive_Turn_Holonom, //Fahren unter Beibehaltung der Startorientierung
					DriveAndTurn_Holonom, //Fahren bei gleichzeitiger Einnahme der Zielorientierung
					Turn_Drive_Turn_NonHolonom, //vy=0, Drehen in Fahrtrichtung, fahren, Drehen in Zielrichtung
					DriveAndTurn_Turn_NonHolonom //vy=0, Drehen in Fahrtrichtung und fahren, Drehen in Zierichtung
				} Movement;

				PositionDriver();

				void setVVector( const QVector< QPointF >& vvec );

				QVector< QPointF > vVector() const;

				void setOmegaVector( const QVector< QPointF >& omegavec );

				QVector< QPointF > omegaVector() const;

				void setTarget( float x, float y, float phi );

				float targetX() const { return _targetX; }
				float targetY() const { return _targetY; }
				float targetPhi() const { return _targetPhi; }

				void setActualPose( float x, float y, float phi );

				float actualX() const { return _actualX; }
				float actualY() const { return _actualY; }
				float actualPhi() const { return _actualPhi; }

				/**
				Start the movement
				*/
				void start( Movement movement, bool startVTimer = true, bool startOmegaTimer = true );

				void update();

				float velocityX() const;
				float velocityY() const;

				float omega() const;

				bool isPositionReached() const;

				bool isOrientationReached() const;

				float distanceToTargetPosition() const;

				float distanceToTargetOrientation() const;

				/**
				Set the time after which velocity reaches 100%.
				@param ramp Time in milliseconds
				*/
				void setVelocityRamp( float ramp );

				/**
				Set the time after which angular velocity reaches 100%.
				@param ramp Time in milliseconds
				*/
				void setAngularVelocityRamp( float ramp );

				Movement activeMovement() const;

				static void rotate( const float x, const float y, float* xrotated, float* yrotated, float phi );

			private:
				float vramp() const;
				float omegaramp() const;

				QVector< QPointF > _vvec;
				QVector< QPointF > _omegavec;

				float _targetX;
				float _targetY;
				float _targetPhi;

				float _actualX;
				float _actualY;
				float _actualPhi;

				float _velocityX;
				float _velocityY;
				float _omega;

				Movement _movement;

				bool _isPositionReached;
				bool _isOrientationReached;

				float _distanceToTargetPosition;
				float _distanceToTargetOrientation;

				float _vramp;
				float _omegaramp;

				QTime _vtimer;
				QTime _omegatimer;

				int _step;
			};

			inline
				QVector< QPointF > PositionDriver::vVector() const
			{
				return _vvec;
			}

			inline
				QVector< QPointF > PositionDriver::omegaVector() const
			{
				return _omegavec;
			}

			inline
				float PositionDriver::velocityX() const
			{
				return _velocityX;
			}

			inline
				float PositionDriver::velocityY() const
			{
				return _velocityY;
			}

			inline
				float PositionDriver::omega() const
			{
				return _omega;
			}

			inline
				bool PositionDriver::isPositionReached() const
			{
				return _isPositionReached;
			}

			inline
				bool PositionDriver::isOrientationReached() const
			{
				return _isOrientationReached;
			}

			inline
				float PositionDriver::distanceToTargetPosition() const
			{
				return _distanceToTargetPosition;
			}

			inline
				float PositionDriver::distanceToTargetOrientation() const
			{
				return _distanceToTargetOrientation;
			}

			inline
				PositionDriver::Movement PositionDriver::activeMovement() const
			{
				return _movement;
			}
		}
	}
	}

#endif //_REC_ROBOTINO_NAVIGATION_POSITIONDRIVER_H_
