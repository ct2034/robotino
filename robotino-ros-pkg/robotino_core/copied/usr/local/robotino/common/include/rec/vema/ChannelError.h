#ifndef REC_VEMA_CHANNELERROR_H
#define REC_VEMA_CHANNELERROR_H

namespace rec
{
namespace vema
{

/**
 * Holds the status of a VEMA valve terminal.
 */
class ChannelError
{

public:

    explicit ChannelError(unsigned char error)
    	: _error(error)
    {
    }

private:

	unsigned char _error;

private:

};

}
}

#endif	// REC_VEMA_CHANNELERROR_H
