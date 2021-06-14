#include <QWidget>
#include <QBoxLayout>

// TODO namespace

class AspectRatioWidget : public QWidget
{
public:
  AspectRatioWidget(QWidget *widget, float width, float height, QWidget *parent = nullptr);
  void resizeEvent(QResizeEvent *event) override;

private:
  QBoxLayout *layout;
  float arWidth; // aspect ratio width
  float arHeight; // aspect ratio height
};
