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
using SurfKernel = Kernel;

#include <CGAL/Simple_cartesian.h>
using GuiKernel  = CGAL::Simple_cartesian<double>;

/* We might want to look into guarding this with
 *   SurfKernel != GuiKernel, so we only overload this
 *   when/if the Kernel is actually a Core::Expr kernel or something like that.
 *
 * If the kernel already is a double cartesian, then we could simply do
 *    using PainterOstream = CGAL::Qt::PainterOstream<SurfKernel>;
 * here.
 */
class GuiPoint : public GuiKernel::Point_2 {
  private:
    using Base = GuiKernel::Point_2;
  public:
    GuiPoint(const SurfKernel::Point_2& p)
      : Base(
          CGAL::to_double(p.x()),
          CGAL::to_double(p.y())
        )
    {};
};

class GuiSegment : public GuiKernel::Segment_2 {
  private:
    using Base = GuiKernel::Segment_2;
  public:
    GuiSegment(const SurfKernel::Segment_2& s)
      : Base( GuiPoint(s.source()), GuiPoint(s.target()))
    {};
};

class PainterOstream : public CGAL::Qt::PainterOstream<GuiKernel> {
  private:
    using Base = CGAL::Qt::PainterOstream<GuiKernel>;
  public:
    PainterOstream(QPainter* p) : Base(p) {};

    inline PainterOstream& operator<<(const GuiKernel::Segment_2& s) {
      Base::operator<<(s);
      return *this;
    }
    inline PainterOstream& operator<<(const SurfKernel::Segment_2& s) {
      GuiSegment g(s);
      Base::operator<<(g);
      return *this;
    }
};


using Converter = CGAL::Qt::Converter<Kernel>;
using GConverter = CGAL::Qt::Converter<GuiKernel>;
