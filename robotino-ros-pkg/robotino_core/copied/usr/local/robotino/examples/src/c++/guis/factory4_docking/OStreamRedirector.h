#ifndef _REC_QT_LOG_OSTREAMREDIRECTOR_H_
#define _REC_QT_LOG_OSTREAMREDIRECTOR_H_

#include "Log.h"

#include <iostream>
#include <streambuf>
#include <string>

/**
 * This class redirects a std::ostream (as cout and cerr).
 * Data written to the std::ostream will be written into
 * a Log directly.
 */
class OStreamRedirector : public std::basic_streambuf<char>
{

public:
	OStreamRedirector(std::ostream& stream, Log& log);
	~OStreamRedirector();

protected:
	/**
	* [http://www.cplusplus.com/reference/iostream/streambuf/overflow/]:
	* overflow() happen when a new character is to be written at the put
	* pointer pptr position, but this has reached the end pointer epptr,
	* indicating that apparently no room is currently available at the
	* internal output array.
	*/
	virtual int_type overflow(int_type v);

	/**
	* [http://www.cplusplus.com/reference/iostream/streambuf/overflow/]:
	* Writes up to n characters from the array pointed by s to the output
	* sequence controlled by the stream buffer. (...) This is a virtual
	* member function that can be redefined for a specific behavior in
	* derived classes. Its default behavior in std::streambuf is to
	* perform the expected behavior by calling repeatedly the member
	* function sputc().
	*/
	virtual std::streamsize xsputn(const char *p, std::streamsize n);

private:

	std::ostream&		_stream;		/**< Reference to the redirected stream. */
	std::streambuf*		_oldStreamBuf;	/**< Pointer to the original streambuf of the redirected stream. */
	std::string			_str;			/**< std::string used to temporarily store the written characters. */
	Log&	_log;						/**< Reference to a rec::qt::log::Log display widget. */
};

#endif	// _REC_QT_LOG_OSTREAMREDIRECTOR_H_
