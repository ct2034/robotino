#ifndef REC_VEMA_VEMAERROR_H
#define REC_VEMA_VEMAERROR_H

namespace rec
{
namespace vema
{

/**
 * Holds the status of a VEMA valve terminal.
 */
class VEMAError
{

public:

    explicit VEMAError(unsigned char error)
    	: _error(error)
    {
    }

private:

	unsigned char _error;

private:

};

}
}

#endif	// REC_VEMA_VEMAERROR_H
