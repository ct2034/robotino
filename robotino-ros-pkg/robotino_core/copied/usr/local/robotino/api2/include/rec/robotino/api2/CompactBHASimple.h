//  Copyright (C) 2004-2012, Robotics Equipment Corporation GmbH

//Copyright (c) ...
//
//REC Robotics Equipment Corporation GmbH, Planegg, Germany. All rights reserved.
//Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
//1) Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
//2) Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
//
//THIS SOFTWARE IS PROVIDED BY REC ROBOTICS EQUIPMENT CORPORATION GMBH �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REC ROBOTICS EQUIPMENT CORPORATION GMBH
//BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
//GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//Copyright (c) ...
//
//REC Robotics Equipment Corporation GmbH, Planegg, Germany. Alle Rechte vorbehalten.
//Weiterverbreitung und Verwendung in nichtkompilierter oder kompilierter Form, mit oder ohne Ver�nderung, sind unter den folgenden Bedingungen zul�ssig:
//1) Weiterverbreitete nichtkompilierte Exemplare m�ssen das obige Copyright, diese Liste der Bedingungen und den folgenden Haftungsausschluss im Quelltext enthalten.
//2) Weiterverbreitete kompilierte Exemplare m�ssen das obige Copyright, diese Liste der Bedingungen und den folgenden Haftungsausschluss in der Dokumentation und/oder anderen Materialien, die mit dem Exemplar verbreitet werden, enthalten.
//
//DIESE SOFTWARE WIRD VON REC ROBOTICS EQUIPMENT CORPORATION GMBH OHNE JEGLICHE SPEZIELLE ODER IMPLIZIERTE GARANTIEN ZUR VERF�GUNG GESTELLT, DIE UNTER
//ANDEREM EINSCHLIESSEN: DIE IMPLIZIERTE GARANTIE DER VERWENDBARKEIT DER SOFTWARE F�R EINEN BESTIMMTEN ZWECK. AUF KEINEN FALL IST REC ROBOTICS EQUIPMENT CORPORATION GMBH
//F�R IRGENDWELCHE DIREKTEN, INDIREKTEN, ZUF�LLIGEN, SPEZIELLEN, BEISPIELHAFTEN ODER FOLGESCH�DEN (UNTER ANDEREM VERSCHAFFEN VON ERSATZG�TERN ODER -DIENSTLEISTUNGEN;
//EINSCHR�NKUNG DER NUTZUNGSF�HIGKEIT; VERLUST VON NUTZUNGSF�HIGKEIT; DATEN; PROFIT ODER GESCH�FTSUNTERBRECHUNG), WIE AUCH IMMER VERURSACHT UND UNTER WELCHER VERPFLICHTUNG
//AUCH IMMER, OB IN VERTRAG, STRIKTER VERPFLICHTUNG ODER UNERLAUBTER HANDLUNG (INKLUSIVE FAHRL�SSIGKEIT) VERANTWORTLICH, AUF WELCHEM WEG SIE AUCH IMMER DURCH DIE BENUTZUNG
//DIESER SOFTWARE ENTSTANDEN SIND, SOGAR, WENN SIE AUF DIE M�GLICHKEIT EINES SOLCHEN SCHADENS HINGEWIESEN WORDEN SIND.

#ifndef _REC_ROBOTINO_API2_COMPACTBHASIMPLE_H_
#define _REC_ROBOTINO_API2_COMPACTBHASIMPLE_H_

#include <cmath>
#include <algorithm>

namespace rec
{
	namespace robotino
	{
		namespace api2
		{
			/**
			 * @brief	Calculation from xy coordinates to pressures
			 *
			 * See <a href="http://wiki.openrobotino.org/index.php?title=CBHA">here</a> for further details.
			 */
			class CompactBHASimple
			{
			private:
				/*!
				 * Representing special weighted points in 2D space.
				 */
				class Pole
				{

				public:
					Pole(float x, float y, float weight1, float weight2, float weight3)
						: _x(x)
						, _y(y)
					{
						_weight[0] = weight1;
						_weight[1] = weight2;
						_weight[2] = weight3;
					}

					float x() const
					{
						return _x;
					}

					float y() const
					{
						return _y;
					}

					float weight1() const
					{
						return _weight[0];
					}

					float weight2() const
					{
						return _weight[1];
					}

					float weight3() const
					{
						return _weight[2];
					}

					float getWeight(float x, float y, int weightIdx) const
					{
						float dX = _x - x;
						float dY = _y - y;
						float dist = sqrt( dX * dX + dY * dY );
						return std::max( 0.f, 1 - (_weight[weightIdx-1] * dist) );
					}

				private:
					float _x;
					float _y;

					float _weight[3];
				};

				static const Pole _pole1;
				static const Pole _pole2;
				static const Pole _pole3;
				static const Pole _pole4;
				static const Pole _pole5;
				static const Pole _pole6;

			public:
				/**
				 * @param x x-coordinate
				 * @param y y-coordinate
				 * @param b1 Normalized pressure for bellows 1.
				 * @param b2 Normalized pressure for bellows 2.
				 * @param b3 Normalized pressure for bellows 3.
				 */
				static void xy2pressure( float x, float y, float* b1, float* b2, float* b3 )
				{
					/* Transform exterior coordinates to simplify the later
					 * weight calculation. NOTE: Here the transformation is
					 * temporarily implemented in a very simple way, by just
					 * moving outer points to the hexagon circumcircle. */
					float dist = sqrt( x * x + y * y );
					if (dist > 1.0)
					{
						x /= dist;
						y /= dist;
					}

					/* Calculate pressure value for bellows #1 with
					 * weights calculated for poles 3, 4, 5. */
					*b1 = _pole3.getWeight(x, y, 3);
					*b1 += _pole4.getWeight(x, y, 3);
					*b1 += _pole5.getWeight(x, y, 3);

					/* Calculate pressure value for bellows #2 with 
					 * weights calculated for poles 5, 6, 1. */
					*b2 = _pole5.getWeight(x, y, 1);
					*b2 += _pole6.getWeight(x, y, 1);
					*b2 += _pole1.getWeight(x, y, 1);

					/* Calculate pressure value for bellows #3 with
					 * weights calculated for poles 1, 2, 3. */
					*b3 = _pole1.getWeight(x, y, 2);
					*b3 += _pole2.getWeight(x, y, 2);
					*b3 += _pole3.getWeight(x, y, 2);
				}
			};

			const CompactBHASimple::Pole CompactBHASimple::_pole1 = Pole(0.0f, 1.0f, 1.0f, 1.0f, 0.0f);
			const CompactBHASimple::Pole CompactBHASimple::_pole2 = Pole((sqrt((float)3.0) / 2.0f), 0.5f, 0.0f, 1.0f, 0.0f);
			const CompactBHASimple::Pole CompactBHASimple::_pole3 = Pole((sqrt((float)3.0) / 2.0f), -0.5f, 0.0f, 1.0f, 1.0f);
			const CompactBHASimple::Pole CompactBHASimple::_pole4 = Pole(0.0f, -1.0f, 0.0f, 0.0f, 1.0f);
			const CompactBHASimple::Pole CompactBHASimple::_pole5 = Pole(-(sqrt((float)3.0f) / 2.0f), -0.5f, 1.0f, 0.0f, 1.0f);
			const CompactBHASimple::Pole CompactBHASimple::_pole6 = Pole(-(sqrt((float)3.0f) / 2.0f), 0.5f, 1.0f, 0.0f, 0.0f);
		}
	}
}

#endif //_REC_ROBOTINO_API2_COMPACTBHASIMPLE_H_
