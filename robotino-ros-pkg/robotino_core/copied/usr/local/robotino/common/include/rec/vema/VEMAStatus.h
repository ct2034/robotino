#ifndef REC_VEMA_VEMASTATUS_H
#define REC_VEMA_VEMASTATUS_H

namespace rec
{
	namespace vema
	{

		/**
		* Holds the status of a VEMA valve terminal.
		*/
		class VEMAStatus
		{

		public:

			explicit VEMAStatus(unsigned char status)
				: _status(status)
				, _closedLoopCtrl((status & bitClosedLoopControl) != 0)
				, _readyForExecution((status & bitReadyForExecution) != 0)
				, _digitalOut1((status & bitDigitalOut1) != 0)
				, _digitalOut2((status & bitDigitalOut2) != 0)
				, _calibrationDataValid((status & bitCalibrationDataValid) != 0)
				, _calibrationOngoing((status & bitCalibrationOngoing) != 0)
				, _pressureInTolerance((status & bitPressureInTolerance) != 0)
				, _errorPresent((status & bitErrorPresent) != 0)
			{
			}

			inline bool closedLoopCtrlOn() const
			{
				return _closedLoopCtrl;
			}

			inline bool ready() const
			{
				return _readyForExecution;
			}

			inline bool digitalOut1On() const
			{
				return _digitalOut1;
			}

			inline bool digitalOut2On() const
			{
				return _digitalOut2;
			}

			inline bool calibrationValid() const
			{
				return _calibrationDataValid;
			}

			inline bool calibrationOngoing() const
			{
				return _calibrationOngoing;
			}

			inline bool pressureValid() const
			{
				return _pressureInTolerance;
			}

			inline bool error() const
			{
				return _errorPresent;
			}

		private:

			enum {
				bitClosedLoopControl = 0x01,
				bitReadyForExecution = 0x02,
				bitDigitalOut1 = 0x04,
				bitDigitalOut2 = 0x08,
				bitCalibrationDataValid = 0x10,
				bitCalibrationOngoing = 0x20,
				bitPressureInTolerance = 0x40,
				bitErrorPresent = 0x80
			};

			bool _closedLoopCtrl;
			bool _readyForExecution;
			bool _digitalOut1;
			bool _digitalOut2;
			bool _calibrationDataValid;
			bool _calibrationOngoing;
			bool _pressureInTolerance;
			bool _errorPresent;

			unsigned char _status;

		private:

		};

	}
}

#endif	// REC_VEMA_VEMASTATUS_H
