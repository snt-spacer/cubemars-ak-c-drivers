#include "cubemars_panel.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>

#include <pluginlib/class_list_macros.hpp>

namespace cubemars_rviz_panel {

CubemarsPanel::CubemarsPanel(QWidget *parent) : rviz_common::Panel(parent) {}

CubemarsPanel::~CubemarsPanel() {
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}

void CubemarsPanel::onInitialize() {
  node_ = std::make_shared<rclcpp::Node>("cubemars_rviz_panel_node");
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::thread([this] { executor_->spin(); });

  feedback_sub_ = node_->create_subscription<cubemars_msgs::msg::MITFeedback>(
      "/cubemars_controller_node/mit_feedback", 10,
      [this](cubemars_msgs::msg::MITFeedback::ConstSharedPtr msg) {
        uint8_t id = msg->motor_id;
        float pos = msg->position;
        float vel = msg->velocity;
        float torq = msg->torque;
        // Dispatch to each matching widget; dispatchFeedback() is thread-safe.
        for (auto *w : motor_widgets_) {
          if (w->motorId() == id) {
            w->dispatchFeedback(pos, vel, torq);
          }
        }
      });

  // ── UI ────────────────────────────────────────────────────────────
  auto *outer = new QVBoxLayout();
  outer->setContentsMargins(4, 4, 4, 4);
  outer->setSpacing(4);

  // ── General control section ───────────────────────────────────────
  auto *general_frame = new QFrame();
  general_frame->setFrameShape(QFrame::StyledPanel);
  auto *general_lay = new QVBoxLayout(general_frame);
  general_lay->setSpacing(4);

  motor_count_label_ = new QLabel("No motors added");
  QFont cf = motor_count_label_->font();
  cf.setBold(true);
  motor_count_label_->setFont(cf);

  auto *all_btn_row = new QHBoxLayout();
  auto *enter_all_btn = new QPushButton("Enter MIT (all)");
  auto *exit_all_btn = new QPushButton("Exit MIT (all)");
  enter_all_btn->setStyleSheet(
      "QPushButton { background: #388E3C; color: white; border: none; "
      "border-radius: 3px; padding: 3px 6px; }"
      "QPushButton:hover { background: #43A047; }"
      "QPushButton:pressed { background: #2E7D32; }");
  exit_all_btn->setStyleSheet(
      "QPushButton { background: #C62828; color: white; border: none; "
      "border-radius: 3px; padding: 3px 6px; }"
      "QPushButton:hover { background: #D32F2F; }"
      "QPushButton:pressed { background: #B71C1C; }");
  all_btn_row->addWidget(enter_all_btn);
  all_btn_row->addWidget(exit_all_btn);

  general_lay->addWidget(motor_count_label_);
  general_lay->addLayout(all_btn_row);
  outer->addWidget(general_frame);

  connect(enter_all_btn, &QPushButton::clicked, this, &CubemarsPanel::onEnterAllMIT);
  connect(exit_all_btn, &QPushButton::clicked, this, &CubemarsPanel::onExitAllMIT);

  // ── Add Motor button ──────────────────────────────────────────────
  auto *add_btn = new QPushButton("Add Motor");
  add_btn->setStyleSheet(
      "QPushButton { background: #1565C0; color: white; border: none; "
      "border-radius: 3px; padding: 4px 8px; }"
      "QPushButton:hover { background: #1976D2; }"
      "QPushButton:pressed { background: #0D47A1; }");
  outer->addWidget(add_btn);
  connect(add_btn, &QPushButton::clicked, this, &CubemarsPanel::onAddMotor);

  auto *scroll_area = new QScrollArea();
  scroll_area->setWidgetResizable(true);

  auto *scroll_content = new QWidget();
  motors_layout_ = new QVBoxLayout(scroll_content);
  motors_layout_->setAlignment(Qt::AlignTop);
  motors_layout_->setSpacing(4);
  scroll_area->setWidget(scroll_content);

  outer->addWidget(scroll_area);
  setLayout(outer);
}

void CubemarsPanel::onAddMotor() {
  bool ok = false;
  int id = QInputDialog::getInt(this, "Add Motor", "Motor ID (1–127):", 1, 1,
                                127, 1, &ok);
  if (!ok) {
    return;
  }
  uint8_t motor_id = static_cast<uint8_t>(id);
  for (auto *w : motor_widgets_) {
    if (w->motorId() == motor_id) {
      QMessageBox::warning(this, "Duplicate",
                           QString("Motor %1 is already in the panel.")
                               .arg(motor_id));
      return;
    }
  }
  addMotor(motor_id);
}

void CubemarsPanel::addMotor(uint8_t motor_id) {
  auto *w = new MotorWidget(motor_id, node_);
  motor_widgets_.push_back(w);
  motors_layout_->addWidget(w);
  connect(w, &MotorWidget::removeRequested, this,
          &CubemarsPanel::onRemoveMotor);
  updateMotorCount();
  Q_EMIT configChanged();
}

void CubemarsPanel::onRemoveMotor(MotorWidget *widget) {
  auto it = std::find(motor_widgets_.begin(), motor_widgets_.end(), widget);
  if (it != motor_widgets_.end()) {
    motor_widgets_.erase(it);
  }
  motors_layout_->removeWidget(widget);
  widget->deleteLater();
  updateMotorCount();
  Q_EMIT configChanged();
}

void CubemarsPanel::updateMotorCount() {
  int n = static_cast<int>(motor_widgets_.size());
  if (n == 0) {
    motor_count_label_->setText("No motors added");
  } else {
    motor_count_label_->setText(
        QString("%1 motor%2 active").arg(n).arg(n == 1 ? "" : "s"));
  }
}

void CubemarsPanel::onEnterAllMIT() {
  for (auto *w : motor_widgets_) {
    w->onEnterMIT();
  }
}

void CubemarsPanel::onExitAllMIT() {
  for (auto *w : motor_widgets_) {
    w->onExitMIT();
  }
}

void CubemarsPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  QString ids;
  for (size_t i = 0; i < motor_widgets_.size(); ++i) {
    if (i > 0) {
      ids += ',';
    }
    ids += QString::number(motor_widgets_[i]->motorId());
  }
  config.mapSetValue("motor_ids", ids);
}

void CubemarsPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
  QString ids;
  if (!config.mapGetString("motor_ids", &ids) || ids.isEmpty()) {
    return;
  }
  for (const QString &part : ids.split(',')) {
    bool ok = false;
    int id = part.trimmed().toInt(&ok);
    if (ok && id >= 1 && id <= 127) {
      addMotor(static_cast<uint8_t>(id));
    }
  }
}

} // namespace cubemars_rviz_panel

PLUGINLIB_EXPORT_CLASS(cubemars_rviz_panel::CubemarsPanel, rviz_common::Panel)
