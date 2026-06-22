#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    unsigned char id;
    unsigned char data[8];
    unsigned char len;
} MITFrame;

void ak_set_model_by_name(const char *name);

MITFrame ak_enter_frame(unsigned char id);
MITFrame ak_exit_frame(unsigned char id);
MITFrame ak_set_zero_frame(unsigned char id);
MITFrame ak_command_frame(unsigned char id,
                          float p_des, float v_des,
                          float kp, float kd, float t_ff);
void ak_decode_feedback(const unsigned char d[8],
                        float *pos, float *vel, float *torque);

#ifdef __cplusplus
}
#endif
