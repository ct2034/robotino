#include "CameraView.h"
#include "MyCamera.h"

CameraView::CameraView( QWidget* parent, MyCamera* camera )
	: QWidget( parent )
	, _camera( camera )
{
	setMinimumSize( 320, 240 );

	bool connected = connect( _camera, SIGNAL( imageReceived( const QImage& ) ), SLOT( on_camera_imageReceived( const QImage& ) ) );
	Q_ASSERT( connected );
}

void CameraView::on_camera_imageReceived( const QImage& image )
{
	_image = image;
	repaint();
}

void CameraView::paintEvent( QPaintEvent* e )
{
	QPainter painter( this );
	painter.drawImage( rect(), _image );
}
