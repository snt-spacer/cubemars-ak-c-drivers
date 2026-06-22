#include "motor_widget.hpp"

#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMetaObject>
#include <QVBoxLayout>

namespace cubemars_rviz_panel {

namespace {

// Button stylesheets — all follow the same pattern so hover/pressed states work
QString btnStyle(const char *base, const char *hover, const char *pressed,
                 const char *disabled_bg) {
  return QString(
             "QPushButton { background: %1; color: white; border: none; "
             "border-radius: 3px; padding: 3px 6px; }"
             "QPushButton:hover { background: %2; }"
             "QPushButton:pressed { background: %3; }"
             "QPushButton:disabled { background: %4; color: #888; }")
      .arg(base, hover, pressed, disabled_bg);
}

QString enterStyle() { return btnStyle("#388E3C", "#43A047", "#2E7D32", "#A5D6A7"); }
QString exitStyle()  { return btnStyle("#C62828", "#D32F2F", "#B71C1C", "#EF9A9A"); }
QString zeroStyle()  { return btnStyle("#757575", "#9E9E9E", "#616161", "#BDBDBD"); }
QString sendStartStyle() { return btnStyle("#00695C", "#00796B", "#004D40", "#80CBC4"); }
QString sendStopStyle()  { return btnStyle("#E65100", "#EF6C00", "#BF360C", "#FFCC80"); }

QString removeStyle() {
  return "QPushButton { color: #EF5350; background: transparent; "
         "border: 1px solid #EF5350; border-radius: 3px; padding: 2px 6px; }"
         "QPushButton:hover { background: rgba(239,83,80,0.15); }";
}

QString feedBoxStyle() {
  return "QFrame { background: #2d2d2d; border: 1px solid #505050; "
         "border-radius: 4px; }";
}

} // namespace

MotorWidget::MotorWidget(uint8_t motor_id, rclcpp::Node::SharedPtr node,
                         QWidget *parent)
    : QWidget(parent), motor_id_(motor_id), node_(node) {

  // ROS interfaces — all on the shared panel node
  const std::string ns = "/cubemars_controller_node/";
  cmd_pub_ = node_->create_publisher<cubemars_msgs::msg::MITCommand>(
      ns + "mit_command", 10);
  enter_client_ =
      node_->create_client<cubemars_msgs::srv::EnterMIT>(ns + "enter_mit");
  exit_client_ =
      node_->create_client<cubemars_msgs::srv::ExitMIT>(ns + "exit_mit");
  zero_client_ =
      node_->create_client<cubemars_msgs::srv::SetZero>(ns + "set_zero");

  send_timer_ = new QTimer(this);
  connect(send_timer_, &QTimer::timeout, this, &MotorWidget::publishMITCommand);

  // ── UI ──────────────────────────────────────────────────────────────
  auto *outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);

  auto *frame = new QFrame();
  frame->setFrameShape(QFrame::StyledPanel);
  auto *vlay = new QVBoxLayout(frame);
  vlay->setSpacing(6);
  outer->addWidget(frame);

  // ── Header ──────────────────────────────────────────────────────────
  auto *hdr = new QHBoxLayout();
  auto *title = new QLabel(QString("Motor %1").arg(motor_id_));
  QFont f = title->font();
  f.setBold(true);
  title->setFont(f);
  auto *rm_btn = new QPushButton("Remove");
  rm_btn->setStyleSheet(removeStyle());
  hdr->addWidget(title);
  hdr->addStretch();
  hdr->addWidget(rm_btn);
  vlay->addLayout(hdr);
  connect(rm_btn, &QPushButton::clicked, this,
          [this] { emit removeRequested(this); });

  // ── Service buttons ──────────────────────────────────────────────────
  auto *ctrl = new QHBoxLayout();
  enter_btn_ = new QPushButton("Enter MIT");
  set_zero_btn_ = new QPushButton("Set Zero");
  exit_btn_ = new QPushButton("Exit MIT");
  enter_btn_->setStyleSheet(enterStyle());
  set_zero_btn_->setStyleSheet(zeroStyle());
  exit_btn_->setStyleSheet(exitStyle());
  ctrl->addWidget(enter_btn_);
  ctrl->addWidget(set_zero_btn_);
  ctrl->addWidget(exit_btn_);
  vlay->addLayout(ctrl);
  connect(enter_btn_, &QPushButton::clicked, this, &MotorWidget::onEnterMIT);
  connect(set_zero_btn_, &QPushButton::clicked, this, &MotorWidget::onSetZero);
  connect(exit_btn_, &QPushButton::clicked, this, &MotorWidget::onExitMIT);

  // ── MIT Command — two-column grid ────────────────────────────────────
  auto *cmd_box = new QGroupBox("MIT Command");
  auto *grid = new QGridLayout(cmd_box);
  grid->setSpacing(4);
  grid->setColumnStretch(1, 1);
  grid->setColumnStretch(3, 1);

  // helper to create a compact spin row: returns {label, spinbox}
  auto addSpin = [&](int row, int col, const QString &lbl, QDoubleSpinBox *sb) {
    grid->addWidget(new QLabel(lbl), row, col);
    grid->addWidget(sb, row, col + 1);
  };

  p_des_spin_ = new QDoubleSpinBox();
  p_des_spin_->setRange(-12.566, 12.566);
  p_des_spin_->setDecimals(3);
  p_des_spin_->setSingleStep(0.01);
  p_des_spin_->setSuffix(" rad");

  v_des_spin_ = new QDoubleSpinBox();
  v_des_spin_->setRange(-30.0, 30.0);
  v_des_spin_->setDecimals(3);
  v_des_spin_->setSingleStep(0.1);
  v_des_spin_->setSuffix(" rad/s");

  kp_spin_ = new QDoubleSpinBox();
  kp_spin_->setRange(0.0, 500.0);
  kp_spin_->setDecimals(2);
  kp_spin_->setSingleStep(1.0);

  kd_spin_ = new QDoubleSpinBox();
  kd_spin_->setRange(0.0, 5.0);
  kd_spin_->setDecimals(3);
  kd_spin_->setSingleStep(0.01);

  t_ff_spin_ = new QDoubleSpinBox();
  t_ff_spin_->setRange(-18.0, 18.0);
  t_ff_spin_->setDecimals(3);
  t_ff_spin_->setSingleStep(0.1);
  t_ff_spin_->setSuffix(" Nm");

  rate_spin_ = new QSpinBox();
  rate_spin_->setRange(1, 1000);
  rate_spin_->setValue(10);
  rate_spin_->setSuffix(" Hz");

  // Row 0: p_des | v_des
  addSpin(0, 0, "p_des:", p_des_spin_);
  addSpin(0, 2, "v_des:", v_des_spin_);
  // Row 1: kp | kd
  addSpin(1, 0, "kp:", kp_spin_);
  addSpin(1, 2, "kd:", kd_spin_);
  // Row 2: t_ff | Rate
  addSpin(2, 0, "t_ff:", t_ff_spin_);
  grid->addWidget(new QLabel("Rate:"), 2, 2);
  grid->addWidget(rate_spin_, 2, 3);
  // Row 3: Send Once (left 2 cols) | Start/Stop toggle (right 2 cols)
  send_once_btn_ = new QPushButton("Send Once");
  send_once_btn_->setStyleSheet(zeroStyle());
  send_once_btn_->setEnabled(false);
  send_toggle_btn_ = new QPushButton("Start Sending");
  send_toggle_btn_->setStyleSheet(sendStartStyle());
  send_toggle_btn_->setEnabled(false);
  grid->addWidget(send_once_btn_, 3, 0, 1, 2);
  grid->addWidget(send_toggle_btn_, 3, 2, 1, 2);

  vlay->addWidget(cmd_box);
  connect(send_once_btn_, &QPushButton::clicked, this,
          &MotorWidget::publishMITCommand);
  connect(send_toggle_btn_, &QPushButton::clicked, this,
          &MotorWidget::onToggleSending);

  // ── Feedback — three boxes in one row ────────────────────────────────
  auto *fb_box = new QGroupBox("Feedback");
  auto *fb_row = new QHBoxLayout(fb_box);
  fb_row->setSpacing(6);

  auto makeFeedBox = [&](const QString &title, QLabel *&val_lbl) {
    auto *box = new QFrame();
    box->setFrameShape(QFrame::StyledPanel);
    box->setStyleSheet(feedBoxStyle());
    auto *bl = new QVBoxLayout(box);
    bl->setContentsMargins(6, 4, 6, 4);
    bl->setSpacing(1);
    auto *hdr_lbl = new QLabel(title);
    hdr_lbl->setStyleSheet("color: #9E9E9E; font-size: 10px; "
                           "background: transparent; border: none;");
    val_lbl = new QLabel("—");
    val_lbl->setStyleSheet("color: #E0E0E0; font-weight: bold; "
                           "background: transparent; border: none;");
    bl->addWidget(hdr_lbl);
    bl->addWidget(val_lbl);
    return box;
  };

  fb_row->addWidget(makeFeedBox("Position", pos_label_));
  fb_row->addWidget(makeFeedBox("Velocity", vel_label_));
  fb_row->addWidget(makeFeedBox("Torque", torque_label_));
  vlay->addWidget(fb_box);

  setLayout(outer);
}

MotorWidget::~MotorWidget() { send_timer_->stop(); }

void MotorWidget::dispatchFeedback(float position, float velocity,
                                   float torque) {
  QMetaObject::invokeMethod(
      this, "applyFeedback", Qt::QueuedConnection, Q_ARG(float, position),
      Q_ARG(float, velocity), Q_ARG(float, torque));
}

void MotorWidget::applyFeedback(float position, float velocity, float torque) {
  pos_label_->setText(QString::number(static_cast<double>(position), 'f', 4) +
                      " rad");
  vel_label_->setText(
      QString::number(static_cast<double>(velocity), 'f', 4) + " rad/s");
  torque_label_->setText(
      QString::number(static_cast<double>(torque), 'f', 4) + " Nm");
}

void MotorWidget::onToggleSending() {
  if (send_timer_->isActive()) {
    send_timer_->stop();
    send_toggle_btn_->setText("Start Sending");
    send_toggle_btn_->setStyleSheet(sendStartStyle());
  } else {
    send_timer_->start(1000 / rate_spin_->value());
    send_toggle_btn_->setText("Stop Sending");
    send_toggle_btn_->setStyleSheet(sendStopStyle());
  }
}

void MotorWidget::publishMITCommand() {
  cubemars_msgs::msg::MITCommand msg;
  msg.motor_id = motor_id_;
  msg.p_des = static_cast<float>(p_des_spin_->value());
  msg.v_des = static_cast<float>(v_des_spin_->value());
  msg.kp = static_cast<float>(kp_spin_->value());
  msg.kd = static_cast<float>(kd_spin_->value());
  msg.t_ff = static_cast<float>(t_ff_spin_->value());
  cmd_pub_->publish(msg);
}

void MotorWidget::flashButton(QPushButton *btn, const QString &flash_text,
                              const QString &restore_text) {
  btn->setText(flash_text);
  btn->setEnabled(true);
  QTimer::singleShot(1500, btn, [btn, restore_text] { btn->setText(restore_text); });
}

void MotorWidget::onEnterMIT() {
  auto req = std::make_shared<cubemars_msgs::srv::EnterMIT::Request>();
  req->motor_id = motor_id_;
  enter_btn_->setEnabled(false);
  auto *btn = enter_btn_;
  enter_client_->async_send_request(
      req, [this, btn](rclcpp::Client<cubemars_msgs::srv::EnterMIT>::SharedFuture f) {
        bool ok = f.get()->success;
        QMetaObject::invokeMethod(this, [this, btn, ok] {
          flashButton(btn, ok ? "✓ Enter MIT" : "✗ Enter MIT", "Enter MIT");
          if (ok) {
            mit_enabled_ = true;
            setSendEnabled(true);
          }
        }, Qt::QueuedConnection);
      });
}

void MotorWidget::onSetZero() {
  auto req = std::make_shared<cubemars_msgs::srv::SetZero::Request>();
  req->motor_id = motor_id_;
  set_zero_btn_->setEnabled(false);
  auto *btn = set_zero_btn_;
  zero_client_->async_send_request(
      req, [btn](rclcpp::Client<cubemars_msgs::srv::SetZero>::SharedFuture f) {
        bool ok = f.get()->success;
        QMetaObject::invokeMethod(
            btn, [btn, ok] { flashButton(btn, ok ? "✓ Set Zero" : "✗ Set Zero", "Set Zero"); },
            Qt::QueuedConnection);
      });
}

void MotorWidget::onExitMIT() {
  // Stop sending immediately — don't wait for the service response.
  mit_enabled_ = false;
  setSendEnabled(false);

  auto req = std::make_shared<cubemars_msgs::srv::ExitMIT::Request>();
  req->motor_id = motor_id_;
  exit_btn_->setEnabled(false);
  auto *btn = exit_btn_;
  exit_client_->async_send_request(
      req, [btn](rclcpp::Client<cubemars_msgs::srv::ExitMIT>::SharedFuture f) {
        bool ok = f.get()->success;
        QMetaObject::invokeMethod(
            btn, [btn, ok] { flashButton(btn, ok ? "✓ Exit MIT" : "✗ Exit MIT", "Exit MIT"); },
            Qt::QueuedConnection);
      });
}

void MotorWidget::setSendEnabled(bool enabled) {
  if (!enabled) {
    send_timer_->stop();
    send_toggle_btn_->setText("Start Sending");
    send_toggle_btn_->setStyleSheet(sendStartStyle());
  }
  send_once_btn_->setEnabled(enabled);
  send_toggle_btn_->setEnabled(enabled);
}

} // namespace cubemars_rviz_panel
