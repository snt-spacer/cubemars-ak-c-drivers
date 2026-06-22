#include <cubemars_msgs/msg/mit_feedback.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ak_mit_adapter.h"

#include <atomic>
#include <cerrno>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <memory>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

class CubemarsFeedbackListenerNode : public rclcpp::Node {
public:
  CubemarsFeedbackListenerNode()
      : Node("cubemars_feedback_listener_node"), sock_(-1), running_(false) {
    declare_parameter("can_interface", "can0");
    auto iface = get_parameter("can_interface").as_string();

    sock_ = open_can_socket(iface.c_str());
    if (sock_ < 0) {
      RCLCPP_ERROR(
          get_logger(),
          "Failed to open CAN socket on %s — ensure the interface is up",
          iface.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "CAN listener socket open on %s", iface.c_str());
    feedback_pub_ = create_publisher<cubemars_msgs::msg::MITFeedback>(
        "/cubemars_controller_node/mit_feedback", 10);

    running_ = true;
    can_thread_ =
        std::thread(&CubemarsFeedbackListenerNode::can_read_loop, this);
  }

  ~CubemarsFeedbackListenerNode() {
    running_ = false;
    if (sock_ >= 0) {
      close(sock_);
      sock_ = -1;
    }
    if (can_thread_.joinable())
      can_thread_.join();
  }

private:
  int sock_;
  std::atomic<bool> running_;
  std::thread can_thread_;

  rclcpp::Publisher<cubemars_msgs::msg::MITFeedback>::SharedPtr feedback_pub_;

  int open_can_socket(const char *iface) {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0)
      return -1;

    struct ifreq ifr{};
    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
      close(s);
      return -1;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      close(s);
      return -1;
    }

    return s;
  }

  void can_read_loop() {
    while (running_) {
      if (sock_ < 0) {
        break;
      }

      struct pollfd pfd{};
      pfd.fd = sock_;
      pfd.events = POLLIN;

      int poll_rc = poll(&pfd, 1, 100);
      if (poll_rc == 0) {
        continue;
      }
      if (poll_rc < 0) {
        if (errno == EINTR) {
          continue;
        }
        break;
      }
      if ((pfd.revents & POLLIN) == 0) {
        continue;
      }

      struct can_frame frame{};
      ssize_t n = recv(sock_, &frame, sizeof(frame), 0);
      if (n != static_cast<ssize_t>(sizeof(frame))) {
        continue;
      }
      if (frame.can_id & CAN_EFF_FLAG) {
        continue;
      }
      if (frame.can_id & CAN_RTR_FLAG) {
        continue;
      }

      float pos, vel, torque;
      ak_decode_feedback(frame.data, &pos, &vel, &torque);

      auto msg = cubemars_msgs::msg::MITFeedback();
      msg.motor_id = static_cast<uint8_t>(frame.can_id & CAN_SFF_MASK);
      msg.position = pos;
      msg.velocity = vel;
      msg.torque = torque;
      feedback_pub_->publish(msg);
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CubemarsFeedbackListenerNode>());
  rclcpp::shutdown();
  return 0;
}
