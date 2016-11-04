#ifndef _CAMERAVIEW_H_
#define _CAMERAVIEW_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif


class MyCamera;

class CameraView : public QWidget
{
	Q_OBJECT
public:
	CameraView( QWidget* parent, MyCamera* camera );

private Q_SLOTS:
	void on_camera_imageReceived( const QImage& image );

private:
	void paintEvent( QPaintEvent* e );

	MyCamera* _camera;
	QImage _image;
};

#endif //_CAMERAVIEW_H_
