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

#include "config.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include "../easyloggingpp/src/easylogging++.h"
#pragma GCC diagnostic pop

#include <assert.h>

#define SRF_precondition(X) assert(X)

#if defined (DEBUG_EXPENSIVE_PREDICATES) && DEBUG_EXPENSIVE_PREDICATES >= 2
  #define assert_expensive(X) assert(X)
  #define assert_expensive_eq(X, Y) assert( (X) == (Y) )
  #define assert_expensive_ge(X, Y) assert( (X) >= (Y) )
  #define assert_expensive_eq_ptr(X, Y) assert( (X) == (Y) )
#else
  #define assert_expensive(X) assert(true)
  #define assert_expensive_eq(X, Y) assert( abs(CGAL::to_double(X) - CGAL::to_double(Y)) <= 0.0 )
  #define assert_expensive_ge(X, Y) assert( CGAL::to_double(X) - CGAL::to_double(Y) >= 0.0 )
  #define assert_expensive_eq_ptr(X, Y) { \
    const Point_2& __p(X); \
    const Point_2& __q(Y); \
    assert( abs(CGAL::to_double(__p.x()) - CGAL::to_double(__p.x())) <= 0.0 && \
            abs(CGAL::to_double(__p.y()) - CGAL::to_double(__p.y())) <= 0.0); \
    }
#endif

class Event;
class EventQueue;
class KineticTriangle;
class KineticTriangulation;
//class SkeletonDCEL;
class WavefrontEdge;
class WavefrontSupportingLine;
class WavefrontVertex;

#define EXIT_INVALID_INPUT 2
/* Due to CGAL sign errors, the theory goes */
#define EXIT_INIT_INVALID_HEAP 3
/* Due to CGAL sign errors, orientation does not always get computed correctly. */
#define EXIT_CGAL_ORIENTATION_MISMATCH 4

#define STMT_NIL do {} while (0)

#ifndef NDEBUG
  #define DEBUG_STMT(x) x
#else
  #define DEBUG_STMT(x) STMT_NIL
#endif


#define DBG_GENERIC                       ( 1u << 0 )
#define DBG_INPUT                         ( 1u << 1 )
#define DBG_EVENTQ                        ( 1u << 2 )
#define DBG_TRIANGLE                      ( 1u << 3 )
#define DBG_TRIANGLE_ASSERT_VALID         ( 1u << 4 )
#define DBG_TRIANGLE_TIMING               ( 1u << 5 )
#define DBG_TRIANGLE_TIMING2              ( 1u << 6 )
#define DBG_TRIANGLE_FLIP                 ( 1u << 7 )
#define DBG_KT                            ( 1u << 8 )
#define DBG_KT_SETUP                      ( 1u << 9 )
#define DBG_KT_REFINE                     ( 1u << 10 )
#define DBG_KT_EVENT                      ( 1u << 11 )
#define DBG_KT_EVENT2                     ( 1u << 12 )
#define DBG_PROP                          ( 1u << 13 )
#define DBG_HEAP                          ( 1u << 14 )
#define DBG_SKEL                          ( 1u << 15 )

#define DEBUG_MASK (                  \
                DBG_GENERIC                       | \
                DBG_INPUT                         | \
                DBG_EVENTQ                        | \
                DBG_TRIANGLE                      | \
                DBG_TRIANGLE_TIMING               | \
                DBG_TRIANGLE_TIMING2              | \
                DBG_TRIANGLE_FLIP                 | \
                DBG_KT                            | \
                DBG_KT_SETUP                      | \
                DBG_KT_REFINE                     | \
                DBG_KT_EVENT                      | \
                DBG_KT_EVENT2                     | \
                DBG_PROP                          | \
                DBG_SKEL                          | \
                0 )
/*
                DBG_TRIANGLE_ASSERT_VALID         | \
                DBG_HEAP                          | \
*/

#ifdef DEBUG_OUTPUT
  extern unsigned DBG_INDENT_CTR;
  inline void DBG_INDENT_INC() { ++DBG_INDENT_CTR; }
  inline void DBG_INDENT_DEC() { assert(DBG_INDENT_CTR > 0); --DBG_INDENT_CTR; }
  inline std::string DBG_INDENT() {
    std::ostringstream oss;
    for (unsigned i=0; i<DBG_INDENT_CTR; ++i) {
      oss << (((i+1)%2 == 0)  ? u8"· " : "  ");
    }
    return oss.str();
  }

  #ifdef DEBUG_OUTPUT_WITH_FILES
    #define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
    #define LOG_EXTRA_INFO << __FILENAME__ << ":" << __LINE__ << " "
  #else
    #define LOG_EXTRA_INFO
  #endif

  #define DBG_INDENT_LEVEL_STORE unsigned __dbg_indent_ctr = DBG_INDENT_CTR
  #define DBG_INDENT_LEVEL_CHECK assert(__dbg_indent_ctr == DBG_INDENT_CTR)

  #define DBG(FACILITY) if (DEBUG_MASK & (FACILITY)) LOG(DEBUG) << DBG_INDENT()  LOG_EXTRA_INFO << __FUNCTION__ << "(): "
#else
  inline void DBG_INDENT_INC() {}
  inline void DBG_INDENT_DEC() {}

  #define DBG_INDENT_LEVEL_STORE STMT_NIL
  #define DBG_INDENT_LEVEL_CHECK STMT_NIL

  #define DBG(x) do { (void)(x); } while (0); if (0) LOG(DEBUG)
#endif

#define DBG_FUNC_BEGIN(FACILITY) do { DBG_INDENT_INC(); DBG(FACILITY) << "{"; } while (0)
#define DBG_FUNC_END(FACILITY)   do { DBG(FACILITY) << "}"  ; DBG_INDENT_DEC(); } while (0)

#define NOTIMPL_MSG      LOG(ERROR) << "NOT IMPLEMENTED YET: "
#define CANNOTHAPPEN_MSG LOG(ERROR) << "I was told this could never happen.  I trusted you! "

void setup_logging(int argc, char* argv[], bool debugLogs=true);
