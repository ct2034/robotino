#include "TabView.h"
#include "ImageView.h"
#include "MyKinect.h"

TabView::TabView( QWidget* parent, MyKinect* kinect )
: QTabWidget( parent )
{
	bool ok;

	ImageView* depthView = new ImageView( this );
	ImageView* videoView = new ImageView( this );
	ImageView* objectView = new ImageView( this );

	ok = connect( kinect, SIGNAL( depthReceived( const QImage& ) ), depthView, SLOT( on_imageReceived( const QImage& ) ) );
	Q_ASSERT( ok );

	ok = connect( kinect, SIGNAL( videoReceived( const QImage& ) ), videoView, SLOT( on_imageReceived( const QImage& ) ) );
	Q_ASSERT( ok );

	ok = connect( kinect, SIGNAL( objectReceived( const QImage& ) ), objectView, SLOT( on_imageReceived( const QImage& ) ) );
	Q_ASSERT( ok );

	addTab( depthView, "Depth" );
	addTab( videoView, "Video" );
	addTab(objectView, "Object" );
}
