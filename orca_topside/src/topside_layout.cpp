// MIT License
//
// Copyright (c) 2021 Clyde McQueen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "orca_topside/topside_layout.hpp"

#include <QWidget>

#include <iostream>

namespace orca_topside
{

TopsideLayout::TopsideLayout()
{
  setContentsMargins(QMargins());
  setSpacing(-1);
}

TopsideLayout::~TopsideLayout()
{
  while (!widgets_.isEmpty()) {
    delete widgets_.takeFirst();
  }
}

void TopsideLayout::addWidget(QWidget *widget, int width, int height, Qt::Alignment alignment)
{
  addItem(new QWidgetItem(widget), width, height, alignment);
}

void TopsideLayout::addItem(QLayoutItem *item, int width, int height, Qt::Alignment alignment)
{
  widgets_.append(new ItemWrapper(item, width, height, alignment));
}

void TopsideLayout::set_main_widget(QWidget *widget)
{
  if (!widget || widgets_.size() < 2) {
    std::cout << "nothing to do" << std::endl;
    return;
  }

  for (int i = 1; i < widgets_.size(); ++i) {
    if (widgets_[i]->item->widget() == widget) {
      // Place at the bottom of the z-order
      widget->lower();

      // Move to index 0
      widgets_.move(i, 0);
      break;
    }
  }

  // Redraw
  update();
}

void TopsideLayout::addItem(QLayoutItem *item)
{
  addItem(item, 400, 300, Qt::Alignment());
}

Qt::Orientations TopsideLayout::expandingDirections() const
{
  return Qt::Horizontal | Qt::Vertical;
}

bool TopsideLayout::hasHeightForWidth() const
{
  return false;
}

int TopsideLayout::count() const
{
  return widgets_.size();
}

QLayoutItem *TopsideLayout::itemAt(int index) const
{
  ItemWrapper *wrapper = widgets_.value(index);
  return wrapper ? wrapper->item : nullptr;
}

QSize TopsideLayout::minimumSize() const
{
  return calculate_size(MinimumSize);
}

constexpr QSize from_w(int w, int w_ratio, int h_ratio)
{
  return {w, w * h_ratio / w_ratio};
}

constexpr QSize from_h(int h, int w_ratio, int h_ratio)
{
  return {h * w_ratio / h_ratio, h};
}

// Calc the maximum size of a widget while keeping the aspect ratio
constexpr QSize calc_widget_size(QSize container, int w_ratio, int h_ratio)
{
  if (container.width() * h_ratio / w_ratio > container.height()) {
    return from_h(container.height(), w_ratio, h_ratio);
  } else {
    return from_w(container.width(), w_ratio, h_ratio);
  }
}

// Place a widget within the larger rect
constexpr QPoint place_widget(QRect container, QSize widget, Qt::Alignment alignment)
{
  // Default AlignLeft | AlignTop
  int x = container.x();
  int y = container.y();

  if (alignment & Qt::AlignHCenter) {
    x += (container.width() - widget.width()) / 2;
  } else if (alignment & Qt::AlignRight) {
    x += (container.width() - widget.width());
  }

  if (alignment & Qt::AlignVCenter) {
    y += (container.height() - widget.height()) / 2;
  } else if (alignment & Qt::AlignBottom) {
    y += (container.height() - widget.height());
  }

  return {x, y};
}

// The main event... call setGeometry on all child widgets to set size & position
void TopsideLayout::setGeometry(const QRect & rect)
{
  if (widgets_.empty()) {
    return;
  }

  for (int i = 0; i < widgets_.size(); ++i) {
    QSize size;
    if (i == 0) {
      // First widget takes up as much space as possible while keeping the aspect ratio
      size = calc_widget_size(rect.size(), widgets_[i]->width, widgets_[i]->height);
    } else {
      // The rest of the widgets float on top of the first
      size = {widgets_[i]->width, widgets_[i]->height};
    }

    QPoint top_left = place_widget(rect, size, widgets_[i]->alignment);
    widgets_[i]->item->setGeometry({top_left, size});
  }
}

QSize TopsideLayout::sizeHint() const
{
  return calculate_size(SizeHint);
}

QLayoutItem *TopsideLayout::takeAt(int index)
{
  if (index >= 0 && index < widgets_.size()) {
    ItemWrapper *layoutStruct = widgets_.takeAt(index);
    return layoutStruct->item;
  } else {
    return nullptr;
  }
}

QSize TopsideLayout::calculate_size(SizeType sizeType) const
{
  if (widgets_.empty()) {
    return {};
  } else {
    if (sizeType == MinimumSize) {
      return widgets_[0]->item->minimumSize();
    } else {
      // (sizeType == SizeHint)
      return widgets_[0]->item->sizeHint();
    }
  }
}

}  // namespace orca_topside
