#include "ImageView.h"

ImageView::ImageView( QWidget* parent )
	: QWidget( parent )
{
	setMinimumSize( 320, 240 );
}

void ImageView::on_imageReceived( const QImage& image )
{
	_image = image;
	repaint();
}

void ImageView::paintEvent( QPaintEvent* e )
{
	QPainter painter( this );
	painter.drawImage( rect(), _image );
}
