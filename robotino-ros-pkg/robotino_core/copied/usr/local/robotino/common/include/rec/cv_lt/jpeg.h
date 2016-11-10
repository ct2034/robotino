//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_CV_LT_JPEG_H_
#define _REC_CV_LT_JPEG_H_

namespace rec
{
	namespace cv_lt
	{
		// gets information about a jpg image
		bool jpg_info( const char* src, const unsigned int srcSize, unsigned int* width, unsigned int* height, unsigned int* numChannles );

		// decompresses data to buffer
		bool jpg_decompress( const char* src,
			const unsigned int srcSize,
			char* dst,
			const unsigned int dstSize,
			unsigned int* width,
			unsigned int* height,
			unsigned int* numChannels
			);

		// returns 0 on error, or jpeg size
		unsigned int jpg_compress( const char* srcData,
									  const unsigned int width,
									  const unsigned int height,
									  const unsigned int step,
									  const unsigned int numChannels,
									  char* dst,
									  const unsigned int dstSize,
									  int quality = 75 );
	}
}

#endif //_REC_CV_LT_JPEG_H_

