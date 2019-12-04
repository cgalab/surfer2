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
#include "tools.h"

#include <CGAL/Aff_transformation_2.h>

class WavefrontSupportingLine {
  private:
    using Transformation = CGAL::Aff_transformation_2<Kernel>;
  public:
    const Line_2 l;
    const NT weight;
    const Vector_2 normal_direction; /* arbitrary length */
    const Vector_2 normal_unit; /* unit length */
    const Vector_2 normal; /* weighted */
  public:
    WavefrontSupportingLine(const Point_2 &u, const Point_2 &v, const NT &p_weight)
      : l(u, v)
      , weight(p_weight)
      , normal_direction(l.to_vector().perpendicular(CGAL::COUNTERCLOCKWISE))
      , normal_unit(normal_direction / CGAL::sqrt(normal_direction.squared_length()))
      , normal(normal_unit * weight)
      { };

    WavefrontSupportingLine(const Line_2 &p_l, const NT &p_weight)
      : l(p_l)
      , weight(p_weight)
      , normal_direction(l.to_vector().perpendicular(CGAL::COUNTERCLOCKWISE))
      , normal_unit(normal_direction / CGAL::sqrt(normal_direction.squared_length()))
      , normal(normal_unit * weight)
      { };

    Line_2 line_at_one() const {
      Transformation translate(CGAL::TRANSLATION, normal);
      return Line_2(translate(l));
    }

    Line_2 line_at(const NT& now) const {
      Transformation translate(CGAL::TRANSLATION, now * normal);
      return Line_2(translate(l));
    }
};

using WavefrontSupportingLineList = FixedVector<WavefrontSupportingLine>;
