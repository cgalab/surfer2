/**  surfer2 -- a straight skeleton implementation
 *
 *  Copyright 2018, 2019 Peter Palfraader
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once

#include "surf.h"

#include "cgaltools.h"

template <class T=void>
class TriangulationUtilsStatic {
  //protected:
  public:
    static const unsigned _cw[3];
    static const unsigned _ccw[3];
    static const unsigned _mod3[5];
};

template <class T>
const unsigned TriangulationUtilsStatic<T>::_cw[3] = {2, 0, 1};

template <class T>
const unsigned TriangulationUtilsStatic<T>::_ccw[3] = {1, 2, 0};

template <class T>
const unsigned TriangulationUtilsStatic<T>::_mod3[5] = {0, 1, 2, 0, 1};

class TriangulationUtils : public TriangulationUtilsStatic<> {
  public:
    static inline unsigned cw(int i) {
      SRF_precondition(0<=i && i<3);
      return _cw[i];
    }
    static inline unsigned ccw(int i) {
      SRF_precondition(0<=i && i<3);
      return _ccw[i];
    }
    static inline unsigned mod3(int i) {
      SRF_precondition(0<=i && i<6);
      return _mod3[i];
    }
};
