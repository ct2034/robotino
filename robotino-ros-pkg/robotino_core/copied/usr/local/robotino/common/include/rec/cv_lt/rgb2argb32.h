//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#ifndef _REC_CV_LT_RGB2ARGB32_H_
#define _REC_CV_LT_RGB2ARGB32_H_

namespace rec
{
	namespace cv_lt
	{
		/**
		* @param dst The image is stored using a 32-bit ARGB format (0xAARRGGBB).
		*/
		void rgb2argb32( const char* src,
			const unsigned int width,
			const unsigned int height,
			const unsigned int srcStep,
			char* dst,
			const unsigned int dstStep,
			const unsigned char alpha = 255
			);
	}
}

#endif //_REC_CV_LT_RGB2ARGB32_H_
