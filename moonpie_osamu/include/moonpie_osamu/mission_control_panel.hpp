#ifndef MOONPIE_OSAMU_MISSION_CONTROL_PANEL_HPP_
#define MOONPIE_OSAMU_MISSION_CONTROL_PANEL_HPP_

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include <QTextEdit>
#include <QDateTime>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <moonpie_osamu/msg/behavior_status.hpp>

namespace moonpie_osamu
{

class MissionControlPanel : public QMainWindow
{
  Q_OBJECT

public:
  explicit MissionControlPanel(QWidget * parent = nullptr);
  ~MissionControlPanel();
  void onBehaviorStatus(const moonpie_osamu::msg::BehaviorStatus::SharedPtr msg);
  void appendLog(const QString& message);
  std::shared_ptr<rclcpp::Node> getNode();

private slots:
  void onStartMission();
  void onStopMission();

private:
  // ROS node and communication
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mission_start_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mission_stop_pub_;
  rclcpp::Subscription<moonpie_osamu::msg::BehaviorStatus>::SharedPtr behavior_status_sub_;

  QPushButton * start_button_;
  QPushButton * stop_button_;
  QLabel * status_label_;
  QTextEdit * log_display_;
};

}  // namespace moonpie_osamu

#endif  // MOONPIE_OSAMU_MISSION_CONTROL_PANEL_HPP_ 