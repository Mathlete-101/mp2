#include "moonpie_osamu/mission_control_panel.hpp"
#include <moonpie_osamu/msg/behavior_status.hpp>
#include <moonpie_osamu/msg/mission_command.hpp>
#include <QApplication>
#include <thread>
#include <QScrollBar>
#include <QTimer>
#include <chrono>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace moonpie_osamu
{

MissionControlPanel::MissionControlPanel(QWidget * parent)
: QMainWindow(parent),
  node_(std::make_shared<rclcpp::Node>("mission_control_panel")),
  connection_status_(ConnectionStatus::DISCONNECTED)
{
  // Set window properties
  setWindowTitle("Mission Control");
  resize(800, 600);  // Made window larger to accommodate camera feed

  // Create central widget and layout
  auto * central_widget = new QWidget(this);
  auto * main_layout = new QVBoxLayout(central_widget);

  // Create button layout
  auto * button_layout = new QHBoxLayout();

  // Create start button
  start_button_ = new QPushButton("Start Mission");
  start_button_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 10px; font-size: 14px; }");
  button_layout->addWidget(start_button_);

  // Create stop button
  stop_button_ = new QPushButton("Stop Mission");
  stop_button_->setStyleSheet("QPushButton { background-color: #f44336; color: white; padding: 10px; font-size: 14px; }");
  stop_button_->setEnabled(false);
  button_layout->addWidget(stop_button_);

  // Add buttons to main layout
  main_layout->addLayout(button_layout);

  // Create status label
  status_label_ = new QLabel("Status: Disconnected");
  status_label_->setStyleSheet("QLabel { color: red; font-weight: bold; }");
  status_label_->setAlignment(Qt::AlignCenter);
  main_layout->addWidget(status_label_);

  // Create camera display
  camera_display_ = new QLabel();
  camera_display_->setMinimumSize(640, 480);
  camera_display_->setStyleSheet("QLabel { background-color: black; }");
  camera_display_->setAlignment(Qt::AlignCenter);
  main_layout->addWidget(camera_display_);

  // Create log display
  log_display_ = new QTextEdit();
  log_display_->setReadOnly(true);
  log_display_->setStyleSheet("QTextEdit { background-color: #f5f5f5; font-family: monospace; }");
  main_layout->addWidget(log_display_);

  // Set the central widget
  setCentralWidget(central_widget);

  // Connect button signals to slots
  connect(start_button_, &QPushButton::clicked, this, &MissionControlPanel::onStartMission);
  connect(stop_button_, &QPushButton::clicked, this, &MissionControlPanel::onStopMission);

  // Create the ROS node
  node_ = std::make_shared<rclcpp::Node>("mission_control_panel_node");

  // Create command publisher
  cmd_pub_ = node_->create_publisher<moonpie_osamu::msg::MissionCommand>(
    "mission/cmd", 10);

  // Create subscriber for behavior status
  behavior_status_sub_ = node_->create_subscription<moonpie_osamu::msg::BehaviorStatus>(
    "mission/log", 10,
    std::bind(&MissionControlPanel::onBehaviorStatus, this, std::placeholders::_1));

  // Create subscriber for camera feed
  camera_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
    "/camera1/camera1/color/image_raw/compressed", 10,
    std::bind(&MissionControlPanel::onCameraImage, this, std::placeholders::_1));

  // Send initial test command
  QTimer::singleShot(1000, this, &MissionControlPanel::sendTestCommand);

  RCLCPP_INFO(node_->get_logger(), "Mission Control Panel initialized");
}

MissionControlPanel::~MissionControlPanel()
{
  // Clean up resources
}

void MissionControlPanel::setStatusLabel(ConnectionStatus status)
{
  switch (status) {
    case ConnectionStatus::DISCONNECTED:
      status_label_->setText("Status: Disconnected");
      status_label_->setStyleSheet("QLabel { color: red; font-weight: bold; }");
      break;
    case ConnectionStatus::READY:
      status_label_->setText("Status: Ready");
      status_label_->setStyleSheet("QLabel { color: #90ee90; font-weight: bold; }"); // light green
      break;
    case ConnectionStatus::RUNNING:
      status_label_->setText("Status: Running");
      status_label_->setStyleSheet("QLabel { color: #006400; font-weight: bold; }"); // dark green
      break;
  }
}

void MissionControlPanel::updateConnectionStatus(ConnectionStatus status)
{
  connection_status_ = status;
  setStatusLabel(status);
  if (status == ConnectionStatus::DISCONNECTED) {
    start_button_->setEnabled(false);
    stop_button_->setEnabled(false);
  } else if (status == ConnectionStatus::READY) {
    start_button_->setEnabled(true);
    stop_button_->setEnabled(true);
  } else if (status == ConnectionStatus::RUNNING) {
    start_button_->setEnabled(false);
    stop_button_->setEnabled(true);
  }
}

void MissionControlPanel::sendTestCommand()
{
  auto msg = std::make_unique<moonpie_osamu::msg::MissionCommand>();
  msg->command = "TEST";
  cmd_pub_->publish(std::move(msg));
  appendLog("Sent test command to mission control node...");
}

void MissionControlPanel::onStartMission()
{
  if (connection_status_ != ConnectionStatus::READY) {
    appendLog("Cannot start mission: Not connected to mission control node");
    return;
  }

  auto msg = std::make_unique<moonpie_osamu::msg::MissionCommand>();
  msg->command = "START";
  cmd_pub_->publish(std::move(msg));
  appendLog("Sent start command");
}

void MissionControlPanel::onStopMission()
{
  if (connection_status_ != ConnectionStatus::READY) {
    appendLog("Cannot stop mission: Not connected to mission control node");
    return;
  }

  auto msg = std::make_unique<moonpie_osamu::msg::MissionCommand>();
  msg->command = "STOP";
  cmd_pub_->publish(std::move(msg));
  appendLog("Sent stop command");
}

void MissionControlPanel::onBehaviorStatus(const moonpie_osamu::msg::BehaviorStatus::SharedPtr msg)
{
  QString timestamp = QString::fromStdString(
    std::to_string(msg->timestamp.sec) + "." +
    std::to_string(msg->timestamp.nanosec / 1000000).substr(0, 3)
  );

  QString log_message = QString("[%1] %2: %3")
    .arg(timestamp)
    .arg(QString::fromStdString(msg->current_node))
    .arg(QString::fromStdString(msg->details));

  appendLog(log_message);

  // Update status based on message
  if (msg->current_node == "mission_control") {
    if (msg->status == "NAVIGATING") {
      updateConnectionStatus(ConnectionStatus::RUNNING);
    } else if (msg->status == "READY") {
      updateConnectionStatus(ConnectionStatus::READY);
    } else if (msg->status == "IDLE") {
      updateConnectionStatus(ConnectionStatus::READY);
    }
  }
}

void MissionControlPanel::onCameraImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
  try {
    // Convert compressed image to cv::Mat
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to decode compressed image");
      return;
    }

    // Convert BGR to RGB
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

    // Convert to QImage
    QImage image(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
    
    // Scale image to fit display while maintaining aspect ratio
    QPixmap pixmap = QPixmap::fromImage(image);
    pixmap = pixmap.scaled(camera_display_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    
    // Update display
    camera_display_->setPixmap(pixmap);
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Error processing camera image: %s", e.what());
  }
}

void MissionControlPanel::appendLog(const QString& message)
{
  log_display_->moveCursor(QTextCursor::Start);
  log_display_->insertPlainText(message + "\n");
}

std::shared_ptr<rclcpp::Node> MissionControlPanel::getNode()
{
  return node_;
}

}  // namespace moonpie_osamu

int main(int argc, char ** argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Create Qt application
  QApplication app(argc, argv);

  // Create and show the mission control panel
  moonpie_osamu::MissionControlPanel panel;
  panel.show();

  // Create a timer to process ROS events
  QTimer ros_timer;
  ros_timer.setInterval(10);  // 10ms interval
  QObject::connect(&ros_timer, &QTimer::timeout, [&panel]() {
    rclcpp::spin_some(panel.getNode());
  });
  ros_timer.start();

  // Run the Qt event loop
  int result = app.exec();

  // Clean up
  rclcpp::shutdown();

  return result;
} 