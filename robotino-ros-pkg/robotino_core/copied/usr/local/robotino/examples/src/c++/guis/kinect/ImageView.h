#ifndef _ImageView_H_
#define _ImageView_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif


class ImageView : public QWidget
{
	Q_OBJECT
public:
	ImageView( QWidget* parent );

private Q_SLOTS:
	void on_imageReceived( const QImage& );

private:
	void paintEvent( QPaintEvent* e );

	QImage _image;
};

#endif //_ImageView_H_
