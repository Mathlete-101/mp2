#include "moonpie_osamu/mission_control_panel.hpp"
#include <moonpie_osamu/msg/behavior_status.hpp>
#include <QApplication>
#include <thread>
#include <QScrollBar>
#include <QTimer>

namespace moonpie_osamu
{

MissionControlPanel::MissionControlPanel(QWidget * parent)
: QMainWindow(parent)
{
  // Set window properties
  setWindowTitle("Mission Control");
  resize(600, 400);

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
  status_label_ = new QLabel("Status: Ready");
  status_label_->setStyleSheet("QLabel { font-size: 14px; font-weight: bold; padding: 10px; }");
  status_label_->setAlignment(Qt::AlignCenter);
  main_layout->addWidget(status_label_);

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

  // Create publishers
  mission_start_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
    "mission/start", 10);
  mission_stop_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
    "mission/stop", 10);

  // Create subscriber for behavior status
  behavior_status_sub_ = node_->create_subscription<moonpie_osamu::msg::BehaviorStatus>(
    "behavior/status", 10,
    std::bind(&MissionControlPanel::onBehaviorStatus, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "Mission Control Panel initialized");
}

MissionControlPanel::~MissionControlPanel()
{
  // Clean up resources
}

void MissionControlPanel::onStartMission()
{
  // Publish start message
  auto msg = std::make_unique<std_msgs::msg::Empty>();
  mission_start_pub_->publish(std::move(msg));

  // Update UI
  start_button_->setEnabled(false);
  stop_button_->setEnabled(true);
  status_label_->setText("Status: Mission Running");
  status_label_->setStyleSheet("QLabel { font-size: 14px; font-weight: bold; padding: 10px; color: #4CAF50; }");
  
  // Clear log display
  log_display_->clear();
  appendLog("Mission started");
}

void MissionControlPanel::onStopMission()
{
  // Publish stop message
  auto msg = std::make_unique<std_msgs::msg::Empty>();
  mission_stop_pub_->publish(std::move(msg));

  // Update UI
  start_button_->setEnabled(true);
  stop_button_->setEnabled(false);
  status_label_->setText("Status: Mission Stopped");
  status_label_->setStyleSheet("QLabel { font-size: 14px; font-weight: bold; padding: 10px; color: #f44336; }");
  
  appendLog("Mission stopped");
}

void MissionControlPanel::onBehaviorStatus(const moonpie_osamu::msg::BehaviorStatus::SharedPtr msg)
{
  QString log_entry = QString("[%1] %2: %3 - %4")
    .arg(QDateTime::fromSecsSinceEpoch(msg->timestamp.sec).toString("hh:mm:ss"))
    .arg(QString::fromStdString(msg->current_node))
    .arg(QString::fromStdString(msg->status))
    .arg(QString::fromStdString(msg->details));
  
  appendLog(log_entry);
}

void MissionControlPanel::appendLog(const QString& message)
{
  log_display_->append(message);
  // Scroll to the bottom
  log_display_->verticalScrollBar()->setValue(log_display_->verticalScrollBar()->maximum());
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