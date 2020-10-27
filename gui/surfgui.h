/**  surfer2 -- a straight skeleton implementation
 *
 *  Copyright 2020 Peter Palfraader
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

#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/PainterOstream.h>

#include <QBrush>
#include <QPen>

#include "cgaltools.h"

#include <CGAL/Simple_cartesian.h>
using GuiKernel  = CGAL::Simple_cartesian<double>;

/* We might want to look into guarding this with
 *   Kernel != GuiKernel, so we only overload this
 *   when/if the Kernel is actually a Core::Expr kernel or something like that.
 *
 * If the kernel already is a double cartesian, then we could simply do
 *    using PainterOstream = CGAL::Qt::PainterOstream<Kernel>;
 * here.
 */
class GuiPoint : public GuiKernel::Point_2 {
  private:
    using Base = GuiKernel::Point_2;
  public:
    GuiPoint(const Kernel::Point_2& p)
      : Base(
          CGAL::to_double(p.x()),
          CGAL::to_double(p.y())
        )
    {};
};

class PainterOstream : public CGAL::Qt::PainterOstream<GuiKernel> {
  private:
    using Base = CGAL::Qt::PainterOstream<GuiKernel>;
    using GuiSegment = typename GuiKernel::Segment_2;
  public:
    PainterOstream(QPainter* p) : Base(p) {};

    inline PainterOstream& operator<<(const Kernel::Segment_2& s) {
      GuiPoint a(s.source());
      GuiPoint b(s.target());
      GuiSegment g(a,b);
      Base::operator<<(g);
      return *this;
    }
};


using Converter = CGAL::Qt::Converter<Kernel>;
