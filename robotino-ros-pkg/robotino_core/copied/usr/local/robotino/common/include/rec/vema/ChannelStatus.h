#ifndef REC_VEMA_CHANNELSTATUS_H
#define REC_VEMA_CHANNELSTATUS_H

namespace rec
{
namespace vema
{

/**
 * Holds the status of a VEMA valve terminal.
 */
class ChannelStatus
{

public:

    explicit ChannelStatus(unsigned char status)
    	: _status(status)
    	, _closedLoopCtrl((status & bitClosedLoopControl) != 0)
    	, _readyForExecution((status & bitReadyForExecution) != 0)
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
    	bitCalibrationDataValid = 0x10,
    	bitCalibrationOngoing = 0x20,
    	bitPressureInTolerance = 0x40,
    	bitErrorPresent = 0x80
    };

    bool _closedLoopCtrl;
    bool _readyForExecution;
    bool _calibrationDataValid;
    bool _calibrationOngoing;
    bool _pressureInTolerance;
    bool _errorPresent;

	unsigned char _status;

private:

};

}
}

#endif	// REC_VEMA_CHANNELSTATUS_H
