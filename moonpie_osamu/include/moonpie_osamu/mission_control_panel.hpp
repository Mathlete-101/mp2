#ifndef MOONPIE_OSAMU_MISSION_CONTROL_PANEL_HPP_
#define MOONPIE_OSAMU_MISSION_CONTROL_PANEL_HPP_

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QSpinBox>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moonpie_osamu/msg/behavior_status.hpp>
#include <moonpie_osamu/msg/mission_command.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace moonpie_osamu
{

class MissionControlPanel : public QMainWindow
{
  Q_OBJECT

public:
  enum class ConnectionStatus {
    DISCONNECTED,
    READY,
    RUNNING
  };

  explicit MissionControlPanel(QWidget * parent = nullptr);
  ~MissionControlPanel();

  std::shared_ptr<rclcpp::Node> getNode();

private slots:
  void onStartMission();
  void onStopMission();
  void onStartDiggingSequence();
  void onStopDiggingSequence();
  void onStartDigAndDumpSequence();
  void onStartDigAndDumpX3Sequence();
  void sendTestCommand();
  void onSendConfig();

private:
  void setStatusLabel(ConnectionStatus status);
  void updateConnectionStatus(ConnectionStatus status);
  void appendLog(const QString& message);
  void onBehaviorStatus(const moonpie_osamu::msg::BehaviorStatus::SharedPtr msg);
  void onCameraImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
  void onBrioCameraImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
  void onArduinoControl(const std_msgs::msg::String::SharedPtr msg);
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_;
  ConnectionStatus connection_status_;

  // UI Elements
  QLabel* cameraView;
  QLabel* brioCameraView;
  QTextEdit* statusDisplay;
  QTextEdit* logDisplay;
  QPushButton* startDiggingBtn;
  QPushButton* stopDiggingBtn;
  QPushButton* digAndDumpBtn;
  QPushButton* digAndDumpX3Btn;
  QPushButton* sendConfigBtn;
  QSpinBox* digTimeSpinBox;
  QSpinBox* travelTimeSpinBox;
  QSpinBox* driveAndDigSpeedSpinBox;
  QSpinBox* backwardTravelSpeedSpinBox;
  QLabel* diggingStatusLabel;

  // Arduino control status labels
  QLabel* cmdStatusLabel;
  QLabel* digBeltLabel;
  QLabel* dumpBeltLabel;
  QLabel* actuatorExtendLabel;
  QLabel* actuatorRetractLabel;
  QLabel* dpadXLabel;
  QLabel* dpadYLabel;
  QLabel* kpLabel;
  QLabel* kiLabel;
  QLabel* linearXLabel;
  QLabel* angularZLabel;

  // ROS publishers and subscribers
  rclcpp::Publisher<moonpie_osamu::msg::MissionCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<moonpie_osamu::msg::BehaviorStatus>::SharedPtr behavior_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr brio_camera_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arduino_control_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  bool prev_left_bumper_pressed_ = false;
};

}  // namespace moonpie_osamu

#endif  // MOONPIE_OSAMU_MISSION_CONTROL_PANEL_HPP_ 