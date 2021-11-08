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

#ifndef ORCA_TOPSIDE__TOPSIDE_LAYOUT_HPP_
#define ORCA_TOPSIDE__TOPSIDE_LAYOUT_HPP_

#include <QLayout>
#include <QRect>

#include <memory>
#include <string>

namespace orca_topside
{

// Arrange the various video widgets:
// -- the first widget in the list is the main widget: it takes up as much space as possible
// -- the other widgets are smaller and arranged around the border on top of the first widget
// -- preserve the aspect ratio of each widget
class TopsideLayout : public QLayout
{
public:
  TopsideLayout();
  ~TopsideLayout() override;

  void addWidget(QWidget *widget, int width, int height, Qt::Alignment alignment);

  void set_main_widget(QWidget *widget);

private:
  void addItem(QLayoutItem *item, int width, int height, Qt::Alignment alignment);

  void addItem(QLayoutItem *item) override;
  int count() const override;
  Qt::Orientations expandingDirections() const override;
  bool hasHeightForWidth() const override;
  QLayoutItem *itemAt(int index) const override;
  QSize minimumSize() const override;
  void setGeometry(const QRect & rect) override;
  QSize sizeHint() const override;
  QLayoutItem *takeAt(int index) override;

  struct ItemWrapper
  {
    ItemWrapper(QLayoutItem *_item, int _width, int _height, Qt::Alignment _alignment)
    {
      item = _item;
      width = _width;
      height = _height;
      alignment = _alignment;
    }

    ~ItemWrapper()
    {
      delete item;
    }

    QLayoutItem *item;
    int width;
    int height;
    Qt::Alignment alignment;
  };

  enum SizeType
  {
    MinimumSize, SizeHint
  };
  QSize calculate_size(SizeType sizeType) const;

  QList<ItemWrapper *> widgets_;
};

}  // namespace orca_topside

#endif  // ORCA_TOPSIDE__TOPSIDE_LAYOUT_HPP_
