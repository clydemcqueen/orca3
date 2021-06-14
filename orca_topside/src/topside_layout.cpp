#include <iostream>
#include "orca_topside/topside_layout.hpp"

namespace orca_topside
{

TopsideLayout::TopsideLayout(int small_widget_size)
{
  setContentsMargins(QMargins());
  setSpacing(-1);
  small_widget_size_ = small_widget_size;
}

TopsideLayout::~TopsideLayout()
{
  while (!widgets_.isEmpty()) {
    delete widgets_.takeFirst();
  }
}

void TopsideLayout::addWidget(QWidget *widget, AspectRatio aspect_ratio, Qt::Alignment alignment)
{
  addItem(new QWidgetItem(widget), aspect_ratio, alignment);
}

void TopsideLayout::addItem(QLayoutItem *item, AspectRatio aspect_ratio, Qt::Alignment alignment)
{
  widgets_.append(new ItemWrapper(item, aspect_ratio, alignment));
}

void TopsideLayout::addItem(QLayoutItem *item)
{
  addItem(item, SD_4x3, Qt::Alignment());
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
constexpr QSize calc_widget_size(QSize container, TopsideLayout::AspectRatio aspect_ratio)
{
  int w_ratio = aspect_ratio == TopsideLayout::HD_16x9 ? 16 : 4;
  int h_ratio = aspect_ratio == TopsideLayout::HD_16x9 ? 9 : 3;

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
    QSize nominal_size;
    if (i == 0) {
      // First widget takes up as much space as possible
      nominal_size = rect.size();
    } else {
      // The rest of the widgets float on top of the first
      nominal_size = {small_widget_size_, small_widget_size_};
    }

    QSize actual_size = calc_widget_size(nominal_size, widgets_[i]->aspect_ratio);
    QPoint top_left = place_widget(rect, actual_size, widgets_[i]->alignment);

    widgets_[i]->item->setGeometry({top_left, actual_size});
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
