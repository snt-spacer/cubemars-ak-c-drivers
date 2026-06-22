#pragma once

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cubemars_msgs/msg/mit_feedback.hpp>

#include <QLabel>
#include <QScrollArea>
#include <QVBoxLayout>

#include <thread>
#include <vector>

#include "motor_widget.hpp"

namespace cubemars_rviz_panel {

class CubemarsPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit CubemarsPanel(QWidget *parent = nullptr);
  ~CubemarsPanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config &config) override;

private slots:
  void onAddMotor();
  void onRemoveMotor(MotorWidget *widget);
  void onEnterAllMIT();
  void onExitAllMIT();

private:
  void addMotor(uint8_t motor_id);
  void updateMotorCount();

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;

  rclcpp::Subscription<cubemars_msgs::msg::MITFeedback>::SharedPtr
      feedback_sub_;

  QVBoxLayout *motors_layout_{nullptr};
  QLabel *motor_count_label_{nullptr};
  std::vector<MotorWidget *> motor_widgets_;
};

} // namespace cubemars_rviz_panel
