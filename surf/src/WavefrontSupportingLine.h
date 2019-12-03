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
