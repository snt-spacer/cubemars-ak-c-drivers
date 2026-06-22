#include "ak_mit_adapter.h"
#include "ak_mit.h"
#include <string.h>

void ak_set_model_by_name(const char *name) {
    for (int i = 0; i < NUM_MODELS; i++) {
        if (strcmp(MOTOR_MODELS[i].name, name) == 0) {
            ak_set_model_idx(i);
            return;
        }
    }
    /* Unknown name — keep the default (AK40-10) */
}

MITFrame ak_enter_frame(unsigned char id) {
    AKMotorMITMessage m = generate_mit_enter_message(id);
    MITFrame f;
    f.id = m.standard_id;
    f.len = m.len;
    for (int i = 0; i < 8; i++) f.data[i] = m.data[i];
    return f;
}

MITFrame ak_exit_frame(unsigned char id) {
    AKMotorMITMessage m = generate_mit_exit_message(id);
    MITFrame f;
    f.id = m.standard_id;
    f.len = m.len;
    for (int i = 0; i < 8; i++) f.data[i] = m.data[i];
    return f;
}

MITFrame ak_set_zero_frame(unsigned char id) {
    AKMotorMITMessage m = generate_mit_set_zero_message(id);
    MITFrame f;
    f.id = m.standard_id;
    f.len = m.len;
    for (int i = 0; i < 8; i++) f.data[i] = m.data[i];
    return f;
}

MITFrame ak_command_frame(unsigned char id,
                          float p_des, float v_des,
                          float kp, float kd, float t_ff) {
    AKMotorMITMessage m = generate_mit_command_message(id, p_des, v_des, kp, kd, t_ff);
    MITFrame f;
    f.id = m.standard_id;
    f.len = m.len;
    for (int i = 0; i < 8; i++) f.data[i] = m.data[i];
    return f;
}

void ak_decode_feedback(const unsigned char d[8],
                        float *pos, float *vel, float *torque) {
    decode_mit_fb(d, pos, vel, torque);
}
