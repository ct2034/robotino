//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#ifndef _REC_CV_LT_RGB2BGR_H_
#define _REC_CV_LT_RGB2BGR_H_

namespace rec
{
	namespace cv_lt
	{
		/**
		* @param Swap R and B channel of an RGB image.
		*/
		void rgb2bgr( const char* src,
			const unsigned int srcWidth,
			const unsigned int srcHeight,
			const unsigned int srcStep,
			char* dst,
			const unsigned int dstWidth,
			const unsigned int dstHeight,
			const unsigned int dstStep
			);

		void rgb2bgr( char* srcdst,
			const unsigned int width,
			const unsigned int height,
			const unsigned int step
			);
	}
}

#endif //_REC_CV_LT_RGB2BGR_H_
