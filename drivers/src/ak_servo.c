#include "ak_servo.h"
#include <stdint.h>

void buffer_append_int32(uint8_t *buf, int32_t number, int32_t *index) {
  buf[(*index)++] = (uint8_t)(number >> 24);
  buf[(*index)++] = (uint8_t)(number >> 16);
  buf[(*index)++] = (uint8_t)(number >> 8);
  buf[(*index)++] = (uint8_t)(number);
}

void buffer_append_int16(uint8_t *buf, int16_t number, int32_t *index) {
  buf[(*index)++] = (uint8_t)(number >> 8);
  buf[(*index)++] = (uint8_t)(number);
}

ServoCANFeedback decode_servo_can_feedback(const uint8_t data[8]) {
  int16_t pos_raw = (int16_t)((data[0] << 8) | data[1]);
  int16_t spd_raw = (int16_t)((data[2] << 8) | data[3]);
  int16_t cur_raw = (int16_t)((data[4] << 8) | data[5]);
  ServoCANFeedback fb;
  fb.position = pos_raw * 0.1f;
  fb.speed = spd_raw * 10.0f;
  fb.current = cur_raw * 0.01f;
  fb.temperature = (int8_t)data[6];
  fb.error = data[7];
  return fb;
}

uint32_t construct_extended_id(uint8_t id, CAN_PACKET_ID mode) {
  return (uint32_t)id | ((uint32_t)mode << 8);
}

AKMotorServoMessage construct_can_message(uint8_t id, CAN_PACKET_ID mode,
                                          const uint8_t *data, uint8_t len) {
  AKMotorServoMessage msg;
  msg.extended_id = construct_extended_id(id, mode);
  for (uint8_t i = 0; i < len && i < 8; ++i) {
    msg.data[i] = data[i];
  }
  msg.len = len;
  return msg;
}

AKMotorServoMessage generate_duty_message(uint8_t id, float duty) {
  uint8_t buf[4];
  int32_t idx = 0;
  buffer_append_int32(buf, (int32_t)(duty * 100000.0f), &idx);
  return construct_can_message(id, CAN_PACKET_SET_DUTY, buf, 4);
}

AKMotorServoMessage generate_current_message(uint8_t id, float current_A) {
  uint8_t buf[4];
  int32_t idx = 0;
  buffer_append_int32(buf, (int32_t)(current_A * 1000.0f), &idx);
  return construct_can_message(id, CAN_PACKET_SET_CURRENT, buf, 4);
}

AKMotorServoMessage generate_current_brake_message(uint8_t id,
                                                   float current_A) {
  uint8_t buf[4];
  int32_t idx = 0;
  buffer_append_int32(buf, (int32_t)(current_A * 1000.0f), &idx);
  return construct_can_message(id, CAN_PACKET_SET_CURRENT_BRAKE, buf, 4);
}

AKMotorServoMessage generate_rpm_message(uint8_t id, float erpm) {
  uint8_t buf[4];
  int32_t idx = 0;
  buffer_append_int32(buf, (int32_t)erpm, &idx);
  return construct_can_message(id, CAN_PACKET_SET_RPM, buf, 4);
}

AKMotorServoMessage generate_position_message(uint8_t id, float deg) {
  uint8_t buf[4];
  int32_t idx = 0;
  buffer_append_int32(buf, (int32_t)(deg * 10000.0f), &idx);
  return construct_can_message(id, CAN_PACKET_SET_POS, buf, 4);
}

AKMotorServoMessage generate_origin_message(uint8_t id, uint8_t mode) {
  return construct_can_message(id, CAN_PACKET_SET_ORIGIN_HERE, &mode, 1);
}

AKMotorServoMessage generate_pos_spd_message(uint8_t id, float pos_deg,
                                             int32_t spd_erpm,
                                             int32_t acc_erpm_s) {
  uint8_t buf[8];
  int32_t idx = 0;
  int32_t idx1 = 4;
  buffer_append_int32(buf, (int32_t)(pos_deg * 10000.0f), &idx);
  buffer_append_int16(buf, (int16_t)(spd_erpm / 10), &idx1);
  buffer_append_int16(buf, (int16_t)(acc_erpm_s / 10), &idx1);
  return construct_can_message(id, CAN_PACKET_SET_POS_SPD, buf, (uint8_t)idx1);
}