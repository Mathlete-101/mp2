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
#include <QGroupBox>
#include <QJsonDocument>
#include <QJsonObject>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

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

  // Create central widget
  QWidget* centralWidget = new QWidget(this);
  setCentralWidget(centralWidget);

  // Create main layout
  QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);
  mainLayout->setSpacing(10);
  mainLayout->setContentsMargins(10, 10, 10, 10);

  // Create left panel for camera and status
  QVBoxLayout* leftPanel = new QVBoxLayout();
  leftPanel->setSpacing(10);

  // Create camera view
  cameraView = new QLabel(this);
  cameraView->setMinimumSize(640, 480);
  cameraView->setAlignment(Qt::AlignCenter);
  cameraView->setStyleSheet("QLabel { background-color: black; }");
  leftPanel->addWidget(cameraView);

  // Create status display
  statusDisplay = new QTextEdit(this);
  statusDisplay->setReadOnly(true);
  statusDisplay->setMaximumHeight(100);
  statusDisplay->setStyleSheet("QTextEdit { background-color: #f0f0f0; font-family: monospace; }");
  leftPanel->addWidget(statusDisplay);

  // Create log display
  logDisplay = new QTextEdit(this);
  logDisplay->setReadOnly(true);
  logDisplay->setStyleSheet("QTextEdit { background-color: #f5f5f5; font-family: monospace; }");
  leftPanel->addWidget(logDisplay);

  // Add left panel to main layout
  mainLayout->addLayout(leftPanel);

  // Create right panel for digging controls
  QVBoxLayout* rightPanel = new QVBoxLayout();
  rightPanel->setSpacing(10);

  // Create digging control group
  QGroupBox* diggingGroup = new QGroupBox("Digging Control", this);
  QVBoxLayout* diggingLayout = new QVBoxLayout(diggingGroup);
  diggingLayout->setSpacing(5);

  // Create digging status display
  diggingStatusLabel = new QLabel("Status: Idle", this);
  diggingStatusLabel->setStyleSheet("QLabel { font-weight: bold; padding: 5px; background-color: #e0e0e0; border-radius: 3px; }");
  diggingLayout->addWidget(diggingStatusLabel);

  // Create digging control buttons
  startDiggingBtn = new QPushButton("Start Digging", this);
  stopDiggingBtn = new QPushButton("Stop Digging", this);
  digAndDumpBtn = new QPushButton("Dig+Dump", this);

  // Create configuration controls
  QVBoxLayout* configLayout = new QVBoxLayout();
  
  // Dig time control
  QHBoxLayout* timeLayout = new QHBoxLayout();
  QLabel* digTimeLabel = new QLabel("Dig Time (0.1s):", this);
  digTimeSpinBox = new QSpinBox(this);
  digTimeSpinBox->setRange(10, 1000);  // 1.0s to 100.0s
  digTimeSpinBox->setValue(200);  // Default to 20.0s
  digTimeSpinBox->setSingleStep(1);
  timeLayout->addWidget(digTimeLabel);
  timeLayout->addWidget(digTimeSpinBox);

  // Travel time control
  QLabel* travelTimeLabel = new QLabel("Travel Time (0.1s):", this);
  travelTimeSpinBox = new QSpinBox(this);
  travelTimeSpinBox->setRange(10, 1000);  // 1.0s to 100.0s
  travelTimeSpinBox->setValue(30);  // Default to 3.0s
  travelTimeSpinBox->setSingleStep(1);
  timeLayout->addWidget(travelTimeLabel);
  timeLayout->addWidget(travelTimeSpinBox);
  configLayout->addLayout(timeLayout);

  // Drive & Dig Speed control
  QHBoxLayout* speedLayout = new QHBoxLayout();
  QLabel* driveAndDigSpeedLabel = new QLabel("Drive & Dig Speed (0.1 m/s):", this);
  driveAndDigSpeedSpinBox = new QSpinBox(this);
  driveAndDigSpeedSpinBox->setRange(1, 50);  // 0.1 to 5.0 m/s
  driveAndDigSpeedSpinBox->setValue(1);      // Default to 0.1 m/s (set to your default)
  driveAndDigSpeedSpinBox->setSingleStep(1);
  speedLayout->addWidget(driveAndDigSpeedLabel);
  speedLayout->addWidget(driveAndDigSpeedSpinBox);

  // Backward Travel Speed control
  QLabel* backwardTravelSpeedLabel = new QLabel("Backward Travel Speed (0.1 m/s):", this);
  backwardTravelSpeedSpinBox = new QSpinBox(this);
  backwardTravelSpeedSpinBox->setRange(1, 50);  // 0.1 to 5.0 m/s
  backwardTravelSpeedSpinBox->setValue(2);      // Default to 0.2 m/s (set to your default)
  backwardTravelSpeedSpinBox->setSingleStep(1);
  speedLayout->addWidget(backwardTravelSpeedLabel);
  speedLayout->addWidget(backwardTravelSpeedSpinBox);
  configLayout->addLayout(speedLayout);

  // Send config button
  sendConfigBtn = new QPushButton("Send Config", this);
  sendConfigBtn->setStyleSheet("QPushButton { background-color: #2196F3; color: white; padding: 10px; font-size: 14px; }");
  configLayout->addWidget(sendConfigBtn);

  // Style the buttons
  startDiggingBtn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 10px; font-size: 14px; }");
  stopDiggingBtn->setStyleSheet("QPushButton { background-color: #f44336; color: white; padding: 10px; font-size: 14px; }");
  digAndDumpBtn->setStyleSheet("QPushButton { background-color: #FF9800; color: white; padding: 10px; font-size: 14px; }");

  // Add buttons to digging layout
  diggingLayout->addLayout(configLayout);
  diggingLayout->addWidget(startDiggingBtn);
  diggingLayout->addWidget(digAndDumpBtn);
  diggingLayout->addWidget(stopDiggingBtn);
  diggingLayout->addStretch();

  // Add digging group to right panel
  rightPanel->addWidget(diggingGroup);

  // Create Arduino control display group
  QGroupBox* arduinoGroup = new QGroupBox("Arduino Control Status", this);
  QVBoxLayout* arduinoLayout = new QVBoxLayout(arduinoGroup);
  arduinoLayout->setSpacing(5);

  // Create grid layout for status values
  QGridLayout* statusGrid = new QGridLayout();
  statusGrid->setSpacing(5);

  // Create labels for each value
  int row = 0;
  
  // Command status
  statusGrid->addWidget(new QLabel("Command Active:"), row, 0);
  cmdStatusLabel = new QLabel("No");
  cmdStatusLabel->setStyleSheet("QLabel { font-weight: bold; }");
  statusGrid->addWidget(cmdStatusLabel, row++, 1);

  // Belt status
  statusGrid->addWidget(new QLabel("Dig Belt:"), row, 0);
  digBeltLabel = new QLabel("Stopped");
  digBeltLabel->setStyleSheet("QLabel { font-weight: bold; }");
  statusGrid->addWidget(digBeltLabel, row++, 1);

  statusGrid->addWidget(new QLabel("Dump Belt:"), row, 0);
  dumpBeltLabel = new QLabel("Stopped");
  dumpBeltLabel->setStyleSheet("QLabel { font-weight: bold; }");
  statusGrid->addWidget(dumpBeltLabel, row++, 1);

  // Actuator status
  statusGrid->addWidget(new QLabel("Actuator Extending:"), row, 0);
  actuatorExtendLabel = new QLabel("No");
  actuatorExtendLabel->setStyleSheet("QLabel { font-weight: bold; }");
  statusGrid->addWidget(actuatorExtendLabel, row++, 1);

  statusGrid->addWidget(new QLabel("Actuator Retracting:"), row, 0);
  actuatorRetractLabel = new QLabel("No");
  actuatorRetractLabel->setStyleSheet("QLabel { font-weight: bold; }");
  statusGrid->addWidget(actuatorRetractLabel, row++, 1);

  // D-pad values
  statusGrid->addWidget(new QLabel("D-pad X:"), row, 0);
  dpadXLabel = new QLabel("0");
  dpadXLabel->setStyleSheet("QLabel { font-weight: bold; }");
  statusGrid->addWidget(dpadXLabel, row++, 1);

  statusGrid->addWidget(new QLabel("D-pad Y:"), row, 0);
  dpadYLabel = new QLabel("0");
  dpadYLabel->setStyleSheet("QLabel { font-weight: bold; }");
  statusGrid->addWidget(dpadYLabel, row++, 1);

  // PID constants
  statusGrid->addWidget(new QLabel("Kp:"), row, 0);
  kpLabel = new QLabel("0.0");
  kpLabel->setStyleSheet("QLabel { font-weight: bold; }");
  statusGrid->addWidget(kpLabel, row++, 1);

  statusGrid->addWidget(new QLabel("Ki:"), row, 0);
  kiLabel = new QLabel("0.0");
  kiLabel->setStyleSheet("QLabel { font-weight: bold; }");
  statusGrid->addWidget(kiLabel, row++, 1);

  // Velocity values
  statusGrid->addWidget(new QLabel("Linear X:"), row, 0);
  linearXLabel = new QLabel("0.00 m/s");
  linearXLabel->setStyleSheet("QLabel { font-weight: bold; }");
  statusGrid->addWidget(linearXLabel, row++, 1);

  statusGrid->addWidget(new QLabel("Angular Z:"), row, 0);
  angularZLabel = new QLabel("0.00 rad/s");
  angularZLabel->setStyleSheet("QLabel { font-weight: bold; }");
  statusGrid->addWidget(angularZLabel, row++, 1);

  // Add the grid to the arduino layout
  arduinoLayout->addLayout(statusGrid);
  arduinoLayout->addStretch();

  // Add Arduino group to right panel
  rightPanel->addWidget(arduinoGroup);
  rightPanel->addStretch();

  // Add right panel to main layout
  mainLayout->addLayout(rightPanel);

  // Set the main layout
  centralWidget->setLayout(mainLayout);

  // Connect button signals to slots
  connect(startDiggingBtn, &QPushButton::clicked, this, &MissionControlPanel::onStartDiggingSequence);
  connect(stopDiggingBtn, &QPushButton::clicked, this, &MissionControlPanel::onStopDiggingSequence);
  connect(digAndDumpBtn, &QPushButton::clicked, this, &MissionControlPanel::onStartDigAndDumpSequence);
  connect(sendConfigBtn, &QPushButton::clicked, this, &MissionControlPanel::onSendConfig);

  // Create command publisher
  cmd_pub_ = node_->create_publisher<moonpie_osamu::msg::MissionCommand>(
    "mission/cmd", 10);

  // Create subscriber for behavior status
  behavior_status_sub_ = node_->create_subscription<moonpie_osamu::msg::BehaviorStatus>(
    "mission/log", 10,
    std::bind(&MissionControlPanel::onBehaviorStatus, this, std::placeholders::_1));

  // Create subscriber for camera feed - using D435 color image
  camera_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
    "/rs_d435/image_raw/compressed", 10,
    std::bind(&MissionControlPanel::onCameraImage, this, std::placeholders::_1));

  // Create subscriber for Arduino control messages
  arduino_control_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "arduino_command", 10,
    std::bind(&MissionControlPanel::onArduinoControl, this, std::placeholders::_1));

  // Create subscriber for Xbox controller (joy topic)
  joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10,
    std::bind(&MissionControlPanel::onJoy, this, std::placeholders::_1));

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
  QString statusText;
  QString styleSheet;
  
  switch (status) {
    case ConnectionStatus::DISCONNECTED:
      statusText = "Status: Disconnected";
      styleSheet = "QTextEdit { color: red; font-weight: bold; background-color: #f0f0f0; }";
      break;
    case ConnectionStatus::READY:
      statusText = "Status: Ready";
      styleSheet = "QTextEdit { color: #90ee90; font-weight: bold; background-color: #f0f0f0; }";
      break;
    case ConnectionStatus::RUNNING:
      statusText = "Status: Running";
      styleSheet = "QTextEdit { color: #006400; font-weight: bold; background-color: #f0f0f0; }";
      break;
  }
  
  statusDisplay->setText(statusText);
  statusDisplay->setStyleSheet(styleSheet);
}

void MissionControlPanel::updateConnectionStatus(ConnectionStatus status)
{
  connection_status_ = status;
  setStatusLabel(status);
  if (status == ConnectionStatus::DISCONNECTED) {
    startDiggingBtn->setEnabled(false);
    stopDiggingBtn->setEnabled(false);
  } else if (status == ConnectionStatus::READY) {
    startDiggingBtn->setEnabled(true);
    stopDiggingBtn->setEnabled(true);
  } else if (status == ConnectionStatus::RUNNING) {
    startDiggingBtn->setEnabled(false);
    stopDiggingBtn->setEnabled(true);
  }
}

void MissionControlPanel::appendLog(const QString& message)
{
  logDisplay->moveCursor(QTextCursor::Start);
  logDisplay->insertPlainText(message + "\n");
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
  if (msg->current_node == "mission_control" || msg->current_node == "digging_sequence") {
    if (msg->status == "DIGGING" || msg->status == "NAVIGATING") {
      updateConnectionStatus(ConnectionStatus::RUNNING);
    } else if (msg->status == "READY" || msg->status == "IDLE") {
      updateConnectionStatus(ConnectionStatus::READY);
    }
  }

  // Update the log display with the new status
  QString logText = QString::fromStdString(msg->status);
  logDisplay->append(logText);

  // Update digging status if it's a digging-related message
  if (msg->current_node == "digging_sequence") {
    QString statusText = QString::fromStdString(msg->details);
    diggingStatusLabel->setText("Status: " + statusText);
    
    // Update status label color based on the message
    if (statusText.contains("Starting", Qt::CaseInsensitive)) {
      diggingStatusLabel->setStyleSheet("QLabel { font-weight: bold; padding: 5px; background-color: #4CAF50; color: white; border-radius: 3px; }");
    } else if (statusText.contains("Stopping", Qt::CaseInsensitive)) {
      diggingStatusLabel->setStyleSheet("QLabel { font-weight: bold; padding: 5px; background-color: #f44336; color: white; border-radius: 3px; }");
    } else if (statusText.contains("Complete", Qt::CaseInsensitive)) {
      diggingStatusLabel->setStyleSheet("QLabel { font-weight: bold; padding: 5px; background-color: #2196F3; color: white; border-radius: 3px; }");
    } else {
      diggingStatusLabel->setStyleSheet("QLabel { font-weight: bold; padding: 5px; background-color: #FF9800; color: white; border-radius: 3px; }");
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
    pixmap = pixmap.scaled(cameraView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    
    // Update display
    cameraView->setPixmap(pixmap);
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Error processing camera image: %s", e.what());
  }
}

void MissionControlPanel::onArduinoControl(const std_msgs::msg::String::SharedPtr msg)
{
  try {
    // Parse the JSON message
    QJsonDocument doc = QJsonDocument::fromJson(QString::fromStdString(msg->data).toUtf8());
    if (doc.isObject()) {
      QJsonObject obj = doc.object();
      
      // Only update values that are present in the message
      if (obj.contains("cmd")) {
        cmdStatusLabel->setText(obj["cmd"].toBool() ? "Yes" : "No");
        cmdStatusLabel->setStyleSheet(QString("QLabel { font-weight: bold; color: %1; }")
          .arg(obj["cmd"].toBool() ? "#4CAF50" : "#f44336"));
      }
      
      if (obj.contains("dig_belt")) {
        digBeltLabel->setText(obj["dig_belt"].toInt() ? "Running" : "Stopped");
        digBeltLabel->setStyleSheet(QString("QLabel { font-weight: bold; color: %1; }")
          .arg(obj["dig_belt"].toInt() ? "#4CAF50" : "#f44336"));
      }
      
      if (obj.contains("dump_belt")) {
        dumpBeltLabel->setText(obj["dump_belt"].toInt() ? "Running" : "Stopped");
        dumpBeltLabel->setStyleSheet(QString("QLabel { font-weight: bold; color: %1; }")
          .arg(obj["dump_belt"].toInt() ? "#4CAF50" : "#f44336"));
      }
      
      if (obj.contains("actuator_extend")) {
        actuatorExtendLabel->setText(obj["actuator_extend"].toBool() ? "Yes" : "No");
        actuatorExtendLabel->setStyleSheet(QString("QLabel { font-weight: bold; color: %1; }")
          .arg(obj["actuator_extend"].toBool() ? "#4CAF50" : "#f44336"));
      }
      
      if (obj.contains("actuator_retract")) {
        actuatorRetractLabel->setText(obj["actuator_retract"].toBool() ? "Yes" : "No");
        actuatorRetractLabel->setStyleSheet(QString("QLabel { font-weight: bold; color: %1; }")
          .arg(obj["actuator_retract"].toBool() ? "#4CAF50" : "#f44336"));
      }
      
      if (obj.contains("dpad") && obj["dpad"].isObject()) {
        QJsonObject dpad = obj["dpad"].toObject();
        if (dpad.contains("x")) {
          dpadXLabel->setText(QString::number(dpad["x"].toInt()));
        }
        if (dpad.contains("y")) {
          dpadYLabel->setText(QString::number(dpad["y"].toInt()));
        }
      }
      
      if (obj.contains("Kp")) {
        kpLabel->setText(QString::number(obj["Kp"].toDouble(), 'f', 2));
      }
      
      if (obj.contains("Ki")) {
        kiLabel->setText(QString::number(obj["Ki"].toDouble(), 'f', 2));
      }

      // Handle velocity values from the JSON message
      if (obj.contains("linearx_mps")) {
        linearXLabel->setText(QString("%1 m/s").arg(obj["linearx_mps"].toDouble(), 0, 'f', 2));
      }
      
      if (obj.contains("angularz_rps")) {
        angularZLabel->setText(QString("%1 rad/s").arg(obj["angularz_rps"].toDouble(), 0, 'f', 2));
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Error parsing Arduino control message: %s", e.what());
  }
}

void MissionControlPanel::onSendConfig()
{
  auto msg = std::make_unique<moonpie_osamu::msg::MissionCommand>();
  msg->command = "CONFIG";
  msg->dig_time = digTimeSpinBox->value();
  msg->travel_time = travelTimeSpinBox->value();
  msg->drive_and_dig_speed_tenths = driveAndDigSpeedSpinBox->value();
  msg->backward_travel_speed_tenths = backwardTravelSpeedSpinBox->value();
  cmd_pub_->publish(std::move(msg));
  appendLog(QString("Sent configuration: dig_time=%1, travel_time=%2, drive_and_dig_speed=%3, backward_travel_speed=%4")
    .arg(digTimeSpinBox->value() / 10.0)
    .arg(travelTimeSpinBox->value() / 10.0)
    .arg(driveAndDigSpeedSpinBox->value())
    .arg(backwardTravelSpeedSpinBox->value()));
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

void MissionControlPanel::onStartDiggingSequence()
{
  // Digging sequence is independent of mission control node
  // Always send the command regardless of connection status
  auto msg = std::make_unique<moonpie_osamu::msg::MissionCommand>();
  msg->command = "START_DIG";
  cmd_pub_->publish(std::move(msg));
  appendLog("Sent start digging sequence command");
  updateConnectionStatus(ConnectionStatus::RUNNING);
}

void MissionControlPanel::onStopDiggingSequence()
{
  // Digging sequence is independent of mission control node
  // Always send the command regardless of connection status
  auto msg = std::make_unique<moonpie_osamu::msg::MissionCommand>();
  msg->command = "STOP_DIG";
  cmd_pub_->publish(std::move(msg));
  appendLog("Sent stop digging sequence command");
  updateConnectionStatus(ConnectionStatus::READY);
}

void MissionControlPanel::onStartDigAndDumpSequence()
{
  // Digging sequence is independent of mission control node
  // Always send the command regardless of connection status
  auto msg = std::make_unique<moonpie_osamu::msg::MissionCommand>();
  msg->command = "START_DIG_AND_DUMP";
  cmd_pub_->publish(std::move(msg));
  appendLog("Sent start dig and dump sequence command");
  updateConnectionStatus(ConnectionStatus::RUNNING);
}

void MissionControlPanel::sendTestCommand()
{
  auto msg = std::make_unique<moonpie_osamu::msg::MissionCommand>();
  msg->command = "TEST";
  cmd_pub_->publish(std::move(msg));
  appendLog("Sent test command to mission control node...");
}

void MissionControlPanel::onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Left bumper is usually button 4 on Xbox controllers
  bool left_bumper_pressed = (msg->buttons.size() > 4) ? (msg->buttons[4] == 1) : false;
  if (left_bumper_pressed && !prev_left_bumper_pressed_)
  {
    // Trigger dig and dump sequence
    auto cmd_msg = std::make_unique<moonpie_osamu::msg::MissionCommand>();
    cmd_msg->command = "START_DIG_AND_DUMP";
    cmd_pub_->publish(std::move(cmd_msg));
    appendLog("[JOY] Left bumper pressed: Sent start dig and dump sequence command");
    updateConnectionStatus(ConnectionStatus::RUNNING);
  }
  prev_left_bumper_pressed_ = left_bumper_pressed;
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