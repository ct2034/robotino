//  Copyright (C) 2004-2015, Robotics Equipment Corporation GmbH

//Copyright (c) ...
//
//REC Robotics Equipment Corporation GmbH, Planegg, Germany. All rights reserved.
//Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
//1) Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
//2) Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
//
//THIS SOFTWARE IS PROVIDED BY REC ROBOTICS EQUIPMENT CORPORATION GMBH ?AS IS? AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REC ROBOTICS EQUIPMENT CORPORATION GMBH
//BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
//GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//Copyright (c) ...
//
//REC Robotics Equipment Corporation GmbH, Planegg, Germany. Alle Rechte vorbehalten.
//Weiterverbreitung und Verwendung in nichtkompilierter oder kompilierter Form, mit oder ohne Ver?nderung, sind unter den folgenden Bedingungen zul?ssig:
//1) Weiterverbreitete nichtkompilierte Exemplare m?ssen das obige Copyright, diese Liste der Bedingungen und den folgenden Haftungsausschluss im Quelltext enthalten.
//2) Weiterverbreitete kompilierte Exemplare m?ssen das obige Copyright, diese Liste der Bedingungen und den folgenden Haftungsausschluss in der Dokumentation und/oder anderen Materialien, die mit dem Exemplar verbreitet werden, enthalten.
//
//DIESE SOFTWARE WIRD VON REC ROBOTICS EQUIPMENT CORPORATION GMBH OHNE JEGLICHE SPEZIELLE ODER IMPLIZIERTE GARANTIEN ZUR VERF?GUNG GESTELLT, DIE UNTER
//ANDEREM EINSCHLIESSEN: DIE IMPLIZIERTE GARANTIE DER VERWENDBARKEIT DER SOFTWARE F?R EINEN BESTIMMTEN ZWECK. AUF KEINEN FALL IST REC ROBOTICS EQUIPMENT CORPORATION GMBH
//F?R IRGENDWELCHE DIREKTEN, INDIREKTEN, ZUF?LLIGEN, SPEZIELLEN, BEISPIELHAFTEN ODER FOLGESCH?DEN (UNTER ANDEREM VERSCHAFFEN VON ERSATZG?TERN ODER -DIENSTLEISTUNGEN;
//EINSCHR?NKUNG DER NUTZUNGSF?HIGKEIT; VERLUST VON NUTZUNGSF?HIGKEIT; DATEN; PROFIT ODER GESCH?FTSUNTERBRECHUNG), WIE AUCH IMMER VERURSACHT UND UNTER WELCHER VERPFLICHTUNG
//AUCH IMMER, OB IN VERTRAG, STRIKTER VERPFLICHTUNG ODER UNERLAUBTER HANDLUNG (INKLUSIVE FAHRL?SSIGKEIT) VERANTWORTLICH, AUF WELCHEM WEG SIE AUCH IMMER DURCH DIE BENUTZUNG
//DIESER SOFTWARE ENTSTANDEN SIND, SOGAR, WENN SIE AUF DIE M?GLICHKEIT EINES SOLCHEN SCHADENS HINGEWIESEN WORDEN SIND.

#ifndef Factory4MapList_H
#define Factory4MapList_H

#include <string.h>

namespace rec
{
    namespace robotino
    {
        namespace api2
        {
            class Factory4MapList
            {
            public:
				static const int maxNameLength = 255;

				Factory4MapList(const unsigned int size)
					: _size(size)
					, _data(0)
				{
					if (_size > 0)
					{
						_data = new char*[_size];
						for (size_t i = 0; i < _size; i++)
						{
							_data[i] = new char[maxNameLength];
						}
					}
				}

				Factory4MapList(const Factory4MapList& other)
					: _size(other._size)
					, _data(0)
				{
					if (_size > 0)
					{
						_data = new char*[_size];
						for (int i = 0; i < _size; ++i)
						{
							_data[i] = new char[maxNameLength];
							strncpy(_data[i], other._data[i], maxNameLength);
						}
					}
				}

				~Factory4MapList()
				{
					clear();
				}

				void clear()
				{
					for (size_t i = 0; i < _size; i++)
					{
						delete [] _data[i];
					}
					delete[] _data;
					_data = NULL;
					_size = 0;
				}

				const unsigned int size() const
				{
					return _size;
				}

				/// Enables access via index
				const char* operator[](int index) const
				{
					return _data[index];
				}

				char* operator[](int index)
				{
					return _data[index];
				}

				Factory4MapList& operator=(const Factory4MapList& other)
				{
					clear();
					if (other._size > 0)
					{
						_size = other._size;
						_data = new char*[_size];
						for (int i = 0; i < _size; ++i)
						{
							_data[i] = new char[maxNameLength];
							strncpy(_data[i], other._data[i], maxNameLength);
						}
					}
					return *this;
				}

			private:
				unsigned int _size;
				char** _data;
            };
        }
    }
}

#endif // Factory4MapList_H
