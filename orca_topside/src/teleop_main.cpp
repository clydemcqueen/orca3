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

#include "orca_topside/node_spinner.hpp"
#include "orca_topside/teleop_node.hpp"
#include "orca_topside/topside_widget.hpp"

#include <QtWidgets/QApplication>
#include <QtWidgets/QDesktopWidget>

#define STYLESHEET \
  "QWidget {background-color : black;}" \
  "QLabel {background-color : black;color : white;font-family: sans-serif;font-size: 24px;text-align: center;}"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_topside::TeleopNode>();

  if (node->show_window()) {
    // Display, keyboard and joystick input
    QApplication app(argc, argv);
    QCoreApplication::setApplicationName("Orca Topside Control");
    QGuiApplication::setApplicationDisplayName(QCoreApplication::applicationName());
    QCoreApplication::setApplicationVersion(QT_VERSION_STR);

    app.setStyleSheet(STYLESHEET);

    auto topside_widget = new orca_topside::TopsideWidget(node);
    const QRect availableGeometry = QApplication::desktop()->availableGeometry(topside_widget);
    topside_widget->resize(availableGeometry.width() / 6, availableGeometry.height() / 4);
    topside_widget->show();

    node->set_view(topside_widget);

    // Spin the ROS node in the same thread as the Qt app
    orca_topside::NodeSpinner spinner(node);

    return QApplication::exec();
  } else {
    // Joystick input, no display
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  }
}
