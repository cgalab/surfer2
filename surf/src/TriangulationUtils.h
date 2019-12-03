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
