#include "View.h"
#include "Scene.h"

#include <QtDebug>

#define ZOOMOUTFACTOR 0.8
#define ZOOMOUTFACTORSLOW 0.95
#define ZOOMINFACTOR 1.25
#define ZOOMINFACTORSLOW 1.05

View::View( QWidget* parent )
: QGraphicsView( parent )
{
	resetMatrix();
	setTransformationAnchor( NoAnchor );
	setResizeAnchor( NoAnchor );
}

void View::resetMatrix()
{
	QGraphicsView::resetMatrix();
	setTransform( QTransform().scale( 1.0, -1.0 ).rotate( 90 ) );
}

void View::fit()
{
	resetMatrix();
	fitInView( sceneRect(), Qt::KeepAspectRatio );
}

void View::zoomAt( const QPoint& p, qreal factor )
{
	QPointF oldScenePos = mapToScene( p );
	scale( factor, factor );
	QPointF newScenePos = mapToScene( p );
	translate( newScenePos.x() - oldScenePos.x(), newScenePos.y() - oldScenePos.y() );
}

void View::mousePressEvent( QMouseEvent* e )
{
	_mouseMoved = false;
	_mousePosAtPressEvent = e->pos();
	_mousePos = e->pos();
	_mouseButtons = e->buttons();
	e->accept();
}

void View::mouseMoveEvent( QMouseEvent* e )
{
	_mouseMoved = true;
	QPoint delta = _mousePos - e->pos();
	_mousePos = e->pos();

	if( e->modifiers() & Qt::AltModifier )
	{
		qreal f = 1.0f;
		if( delta.y() > 0 )
		{
			f = ZOOMOUTFACTORSLOW;
		}
		else
		{
			f = ZOOMINFACTORSLOW;
		}
		zoomAt( _mousePosAtPressEvent, f );
	}
	else
	{
		translate( delta.y(), delta.x() );
	}
}

void View::mouseReleaseEvent( QMouseEvent* e )
{
	if( false == _mouseMoved )
	{
		switch( _mouseButtons )
		{
		case Qt::LeftButton:
			{
				if( e->modifiers() & Qt::CTRL )
				{
					fit();
				}
				else
				{
					if( e->modifiers() & Qt::AltModifier )
					{
						zoomAt( _mousePosAtPressEvent, ZOOMOUTFACTOR );
					}
					else
					{
						zoomAt( _mousePosAtPressEvent, ZOOMINFACTOR );
					}
				}
				break;
			}
		case Qt::RightButton:
			{
				zoomAt( _mousePosAtPressEvent, ZOOMOUTFACTOR );
				break;
			}
		case Qt::MidButton:
			{
				fit();
				break;
			}
		}
	}

	e->accept();
}

void View::mouseDoubleClickEvent( QMouseEvent* e )
{
}

void View::wheelEvent( QWheelEvent* e )
{
	if ( e->orientation() == Qt::Vertical )
	{
		if ( e->delta() > 0 )
			zoomAt( e->pos(), ZOOMINFACTOR );
		else if ( e->delta() < 0 )
			zoomAt( e->pos(), ZOOMOUTFACTOR );
	}
}
