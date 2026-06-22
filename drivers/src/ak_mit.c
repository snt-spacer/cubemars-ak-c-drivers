#include "ak_mit.h"
#include <stdint.h>

float fminf(float a, float b) { return (a < b) ? a : b; }
float fmaxf(float a, float b) { return (a > b) ? a : b; }

void ak_set_model_idx(int idx) {
    if (idx >= 0 && idx < NUM_MODELS) ak_model_idx = idx;
}

inline int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (int)((x - x_min) * ((float)((1 << bits)) / span));
}

inline float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}

void decode_mit_fb(const uint8_t d[8],
                           float *pos, float *vel, float *torque)
{
    const MotorModel *m = &MOTOR_MODELS[ak_model_idx];
    int p_int = (d[1] << 8) | d[2];
    int v_int = (d[3] << 4) | (d[4] >> 4);
    int t_int = ((d[4] & 0x0F) << 8) | d[5];
    *pos    = uint_to_float(p_int, -12.5f,    12.5f,    16);
    *vel    = uint_to_float(v_int, -m->v_max,  m->v_max, 12);
    *torque = uint_to_float(t_int, -m->t_max,  m->t_max, 12);
}

inline void pack_mit_cmd(uint8_t out[8],
                         float p_des, float v_des,
                         float kp,    float kd, float t_ff)
{
    const float P_MIN  = -12.5f, P_MAX  = 12.5f;
    const float V_MAX  = MOTOR_MODELS[ak_model_idx].v_max;
    const float T_MAX  = MOTOR_MODELS[ak_model_idx].t_max;
    const float KP_MIN =   0.0f, KP_MAX = 500.0f;
    const float KD_MIN =   0.0f, KD_MAX =   5.0f;

    p_des = fminf(fmaxf(P_MIN,  p_des), P_MAX);
    v_des = fminf(fmaxf(-V_MAX, v_des), V_MAX);
    kp    = fminf(fmaxf(KP_MIN, kp),    KP_MAX);
    kd    = fminf(fmaxf(KD_MIN, kd),    KD_MAX);
    t_ff  = fminf(fmaxf(-T_MAX, t_ff),  T_MAX);

    int p_int  = float_to_uint(p_des, P_MIN,  P_MAX,  16);
    int v_int  = float_to_uint(v_des, -V_MAX, V_MAX,  12);
    int kp_int = float_to_uint(kp,    KP_MIN, KP_MAX, 12);
    int kd_int = float_to_uint(kd,    KD_MIN, KD_MAX, 12);
    int t_int  = float_to_uint(t_ff,  -T_MAX, T_MAX,  12);

    out[0] = (uint8_t)(p_int >> 8);
    out[1] = (uint8_t)(p_int & 0xFF);
    out[2] = (uint8_t)(v_int >> 4);
    out[3] = (uint8_t)(((v_int  & 0xF) << 4) | (kp_int >> 8));
    out[4] = (uint8_t)(kp_int & 0xFF);
    out[5] = (uint8_t)(kd_int >> 4);
    out[6] = (uint8_t)(((kd_int & 0xF) << 4) | (t_int >> 8));
    out[7] = (uint8_t)(t_int & 0xFF);
}

AKMotorMITMessage generate_mit_enter_message(uint8_t id) {
    uint8_t cmd[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
    AKMotorMITMessage msg;
    msg.standard_id = id;
    for (int i = 0; i < 8; ++i) {
      msg.data[i] = cmd[i];
    }
    msg.len = 8;
    return msg;
}

AKMotorMITMessage generate_mit_exit_message(uint8_t id) {
    uint8_t cmd[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
    AKMotorMITMessage msg;
    msg.standard_id = id;
    for (int i = 0; i < 8; ++i) {
      msg.data[i] = cmd[i];
    }
    msg.len = 8;
    return msg;
}

AKMotorMITMessage generate_mit_set_zero_message(uint8_t id) {
    uint8_t cmd[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
    AKMotorMITMessage msg;
    msg.standard_id = id;
    for (int i = 0; i < 8; ++i) {
        msg.data[i] = cmd[i];
    }
    msg.len = 8;
    return msg;
}

AKMotorMITMessage generate_mit_command_message(uint8_t id, float p_des, float v_des, float kp, float kd, float t_ff) {
    uint8_t data[8];
    pack_mit_cmd(data, p_des, v_des, kp, kd, t_ff);
    AKMotorMITMessage msg;
    msg.standard_id = id;
    for (int i = 0; i < 8; ++i) {
      msg.data[i] = data[i];
    }
    msg.len = 8;
    return msg;
}