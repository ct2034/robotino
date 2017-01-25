#ifndef _REC_QT_LASERRANGEFINDER_VIEW_H_
#define _REC_QT_LASERRANGEFINDER_VIEW_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif


class View : public QGraphicsView
{
	Q_OBJECT
public:
	View( QWidget* parent );

	public Q_SLOTS:
		void resetMatrix();
		void fit();

private:
	void mousePressEvent( QMouseEvent* e );
	void mouseMoveEvent( QMouseEvent* e );
	void mouseReleaseEvent( QMouseEvent* e );
	void wheelEvent( QWheelEvent* e );
	void mouseDoubleClickEvent( QMouseEvent* e );

	void zoomAt( const QPoint& p, qreal factor );

	QPoint _mousePosAtPressEvent;
	QPoint _mousePos;
	Qt::MouseButtons _mouseButtons;
	bool _mouseMoved;
};

#endif
