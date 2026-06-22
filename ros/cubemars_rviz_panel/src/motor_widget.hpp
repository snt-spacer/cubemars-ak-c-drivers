#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cubemars_msgs/msg/mit_command.hpp>
#include <cubemars_msgs/srv/enter_mit.hpp>
#include <cubemars_msgs/srv/exit_mit.hpp>
#include <cubemars_msgs/srv/set_zero.hpp>

#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>
#include <QWidget>

namespace cubemars_rviz_panel {

class MotorWidget : public QWidget {
  Q_OBJECT
public:
  explicit MotorWidget(uint8_t motor_id, rclcpp::Node::SharedPtr node,
                       QWidget *parent = nullptr);
  ~MotorWidget() override;

  uint8_t motorId() const { return motor_id_; }

  // Called from the panel's feedback subscription callback (any thread).
  // Posts to the Qt main thread internally.
  void dispatchFeedback(float position, float velocity, float torque);

signals:
  void removeRequested(MotorWidget *widget);

public slots:
  void onEnterMIT();
  void onExitMIT();

private slots:
  void onSetZero();
  void onToggleSending();
  void publishMITCommand();
  void applyFeedback(float position, float velocity, float torque);

private:
  uint8_t motor_id_;
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<cubemars_msgs::msg::MITCommand>::SharedPtr cmd_pub_;
  rclcpp::Client<cubemars_msgs::srv::EnterMIT>::SharedPtr enter_client_;
  rclcpp::Client<cubemars_msgs::srv::ExitMIT>::SharedPtr exit_client_;
  rclcpp::Client<cubemars_msgs::srv::SetZero>::SharedPtr zero_client_;

  // MIT command inputs
  QDoubleSpinBox *p_des_spin_;
  QDoubleSpinBox *v_des_spin_;
  QDoubleSpinBox *kp_spin_;
  QDoubleSpinBox *kd_spin_;
  QDoubleSpinBox *t_ff_spin_;
  QSpinBox *rate_spin_;
  QPushButton *send_once_btn_;
  QPushButton *send_toggle_btn_;
  QTimer *send_timer_;

  // Service buttons
  QPushButton *enter_btn_;
  QPushButton *set_zero_btn_;
  QPushButton *exit_btn_;

  // Feedback labels
  QLabel *pos_label_;
  QLabel *vel_label_;
  QLabel *torque_label_;

  bool mit_enabled_{false};

  void setSendEnabled(bool enabled);

  // Momentarily update a button's text then restore it
  static void flashButton(QPushButton *btn, const QString &flash_text,
                          const QString &restore_text);
};

} // namespace cubemars_rviz_panel
