#pragma once

#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/PainterOstream.h>

#include <QPen>

#include "BasicInput.h"

class InputGraphicsItem :
  public CGAL::Qt::GraphicsItem
{
  private:
    using Base = CGAL::Qt::GraphicsItem;

  private:
    const BasicInput * const input;
    CGAL::Qt::PainterOstream<Kernel> painterostream;
    QPen vertices_pen;
    QPen segments_pen;
    QPen labels_pen;
    bool visible_labels = false;


  protected:
    QRectF bounding_rect;
    void updateBoundingBox();

  public:
    InputGraphicsItem(const BasicInput * const input);

    QRectF boundingRect() const { return bounding_rect; };

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    void setVerticesPen(const QPen& pen) { vertices_pen = pen; };
    void setSegmentsPen(const QPen& pen) { segments_pen = pen; };
    void setLabelsPen(const QPen& pen) { labels_pen = pen; };
    const QPen& verticesPen() const { return vertices_pen; }
    const QPen& segmentsPen() const { return segments_pen; }
    const QPen& labelsPen() const { return labels_pen; }
    void setVisibleLabels(bool visible) { if (visible_labels != visible) { prepareGeometryChange(); }; visible_labels = visible; }

    void modelChanged();
};
