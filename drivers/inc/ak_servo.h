#pragma once

#include <stdint.h>

void buffer_append_int32(uint8_t* buf, int32_t number, int32_t* index);
void buffer_append_int16(uint8_t* buf, int16_t number, int32_t* index);

typedef struct {
    float position;    // degrees (-3200 to 3200)
    float speed;       // ERPM   (-320000 to 320000)
    float current;     // A      (-60 to 60)
    int   temperature; // °C     (-20 to 127)
    int   error;       // see error code table p.45
} ServoCANFeedback;

ServoCANFeedback decode_servo_can_feedback(const uint8_t data[8]);

typedef struct {
  uint32_t extended_id; // motor CAN node ID
  uint8_t data[8];
  uint8_t len;
} AKMotorServoMessage;


typedef enum {
    CAN_PACKET_SET_DUTY         = 0,   // Duty Cycle Mode
    CAN_PACKET_SET_CURRENT      = 1,   // Current Loop Mode
    CAN_PACKET_SET_CURRENT_BRAKE= 2,   // Current Brake Mode
    CAN_PACKET_SET_RPM          = 3,   // RPM / Velocity Mode
    CAN_PACKET_SET_POS          = 4,   // Position Mode
    CAN_PACKET_SET_ORIGIN_HERE  = 5,   // Set Origin Mode
    CAN_PACKET_SET_POS_SPD      = 6,   // Position-Velocity Loop Mode
    CAN_PACKET_SET_MIT          = 8,   // MIT Mode
} CAN_PACKET_ID;

uint32_t construct_extended_id(uint8_t id, CAN_PACKET_ID mode);

AKMotorServoMessage construct_can_message(uint8_t id, CAN_PACKET_ID mode, const uint8_t* data, uint8_t len);

// Duty-cycle mode: duty in [-1.0, 1.0]  (Manual p. 38)
AKMotorServoMessage generate_duty_message(uint8_t id, float duty);

// Current loop mode: current in Amperes  (Manual p. 38-39)
AKMotorServoMessage generate_current_message(uint8_t id, float current_A);

// Current brake mode: brake current in Amperes  (Manual p. 39)
AKMotorServoMessage generate_current_brake_message(uint8_t id, float current_A);

// Velocity mode: speed in ERPM  (Manual p. 40)
AKMotorServoMessage generate_rpm_message(uint8_t id, float erpm);

// Position mode: pos in degrees  (Manual p. 41)
AKMotorServoMessage generate_position_message(uint8_t id, float deg);

// Set origin mode: 0 = temporary, 1 = permanent (dual-encoder only)  (Manual p. 42)
AKMotorServoMessage generate_origin_message(uint8_t id, uint8_t mode);

// Position-velocity loop mode  (Manual p. 42-43)
// pos_deg:    target position in degrees
// spd_erpm:   speed limit in ERPM (range ±327680; wire unit = 10 ERPM)
// acc_erpm_s: acceleration in ERPM/s (range 0-327670; wire unit = 10 ERPM/s)
AKMotorServoMessage generate_pos_spd_message(uint8_t id, float pos_deg, int32_t spd_erpm, int32_t acc_erpm_s);