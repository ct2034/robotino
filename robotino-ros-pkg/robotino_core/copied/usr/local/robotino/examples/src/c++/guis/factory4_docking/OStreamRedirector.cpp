#include "OStreamRedirector.h"
#include <streambuf>

OStreamRedirector::OStreamRedirector(std::ostream& stream, Log& log)
	: _stream(stream)
	, _log( log )
	, _oldStreamBuf( stream.rdbuf() )
{
	_stream.rdbuf(this);
}

OStreamRedirector::~OStreamRedirector()
{
	if (!_str.empty())
		_log.postLog(_str.c_str());

	_stream.rdbuf(_oldStreamBuf);
}

OStreamRedirector::int_type OStreamRedirector::overflow(OStreamRedirector::int_type v)
{
	if (v == '\n')
	{
		_log.postLog(_str.c_str());
		_str.clear();
	}
	else
		_str += v;

	return v;
}

std::streamsize OStreamRedirector::xsputn(const char *p, std::streamsize n)
{
	_str.append(p, p + n);

	int pos = 0;
	while (pos != std::string::npos)
	{
		pos = _str.find('\n');
		if (pos != std::string::npos)
		{
			std::string tmp(_str.begin(), _str.begin() + pos);
			_log.postLog(tmp.c_str());
			_str.erase(_str.begin(), _str.begin() + pos + 1);
		}
	}

	return n;
}
