#ifndef _REC_LASERRANGEFINDER_DEFINES_H_
#define _REC_LASERRANGEFINDER_DEFINES_H_

/*
#ifdef WIN32
#  ifdef rec_laserrangefinder_EXPORTS
#    define REC_LASERRANGEFINDER_EXPORT __declspec(dllexport)
#  else
#    define REC_LASERRANGEFINDER_EXPORT __declspec(dllimport)
#  endif // rec_laserrangefinder_EXPORTS
#else // WIN32
#  define REC_LASERRANGEFINDER_EXPORT
#endif // WIN32
*/

#define REC_LASERRANGEFINDER_EXPORT

namespace rec
{
  namespace laserrangefinder
  {
    typedef enum {
      ERR_UNDEFINED,
      ERR_COM_PROTOCOL,
      ERR_QWER
    } ErrorValue;

		typedef enum {
			Lumiflex_Scanner
		} ScannerType;

    typedef enum {
			Range_Measurement,
			Intensity_Measurement
		} MeasurementType;
  }
}

#endif //_REC_LASERRANGEFINDER_DEFINES_H_
