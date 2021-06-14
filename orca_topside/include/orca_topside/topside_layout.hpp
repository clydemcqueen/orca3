#ifndef ORCA_TOPSIDE__TOPSIDE_LAYOUT_HPP_
#define ORCA_TOPSIDE__TOPSIDE_LAYOUT_HPP_

#include <QLayout>
#include <QRect>

namespace orca_topside
{

// Arrange the various video widgets:
// -- the first widget in the list takes up as much space as possible
// -- the other widgets are smaller and arranged around the border on top of the first widget
// -- preserve the aspect ratio of widget
class TopsideLayout : public QLayout
{
public:
  enum AspectRatio
  {
    SD_4x3, HD_16x9
  };

  explicit TopsideLayout(int small_widget_size);
  ~TopsideLayout() override;

  void addWidget(QWidget *widget, AspectRatio aspect_ratio, Qt::Alignment alignment);
  void addItem(QLayoutItem *item, AspectRatio aspect_ratio, Qt::Alignment alignment);

  int small_widget_size() const { return small_widget_size_; }
  void set_small_widget_size(int small_widget_size) { small_widget_size_ = small_widget_size; }

  void addItem(QLayoutItem *item) override;
  int count() const override;
  Qt::Orientations expandingDirections() const override;
  bool hasHeightForWidth() const override;
  QLayoutItem *itemAt(int index) const override;
  QSize minimumSize() const override;
  void setGeometry(const QRect & rect) override;
  QSize sizeHint() const override;
  QLayoutItem *takeAt(int index) override;

private:
  struct ItemWrapper
  {
    ItemWrapper(QLayoutItem *_item, AspectRatio _aspect_ratio, Qt::Alignment _alignment)
    {
      item = _item;
      aspect_ratio = _aspect_ratio;
      alignment = _alignment;
    }

    ~ItemWrapper()
    {
      delete item;
    }

    QLayoutItem *item;
    AspectRatio aspect_ratio;
    Qt::Alignment alignment;
  };

  enum SizeType
  {
    MinimumSize, SizeHint
  };
  QSize calculate_size(SizeType sizeType) const;

  QList<ItemWrapper *> widgets_;
  int small_widget_size_;
};

}  // namespace orca_topside

#endif  // ORCA_TOPSIDE__TOPSIDE_LAYOUT_HPP_
