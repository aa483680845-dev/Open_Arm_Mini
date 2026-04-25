#ifndef DM_MOTOR_H
#define DM_MOTOR_H
#include "pub_user.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#ifdef __cplusplus
extern "C" {
#endif

#define DM_MAX_MOTORS 32

typedef enum {
    DM_3507 = 0,
    DM_4310,
    DM_4340,
    DM_MOTOR_TYPE_COUNT
} dm_motor_type_t;

typedef struct {
    float q_max;
    float dq_max;
    float tau_max;
} dm_limit_param_t;

typedef struct {
    uint16_t can_id;
    uint16_t master_id;
    dm_motor_type_t motor_type;
    dm_limit_param_t limit;
    double state_q;
    double state_dq;
    double state_tau;
    int active;
} dm_motor_t;

/* 全局电机表 */
extern dm_motor_t g_motors[DM_MAX_MOTORS];
extern int g_motor_count;

/* 添加电机，返回 0 成功，-1 失败 */
int dm_motor_add(dm_motor_type_t type, uint16_t can_id, uint16_t master_id);

/* 通过 CAN ID 或 Master ID 查找电机 */
dm_motor_t* dm_motor_find(uint16_t id);

/* 初始化通道波特率 */
void dm_init_channel(device_handle *dev, uint8_t channel, bool canfd,
                     int bitrate, int dbitrate, float can_sp, float canfd_sp);

void dm_control_enable(device_handle *dev);
void dm_control_disable();

/* MIT 模式控制发送 */
void dm_control_mit(device_handle *dev, uint16_t can_id,
                    float kp, float kd, float q, float dq, float tau);

/* 接收回调，注册到 device_hook_to_rec */
void dm_receive_callback(usb_rx_frame_t *frame);

/* 线程安全地读取电机状态快照（idx 为 g_motors 数组下标） */
void dm_motor_snapshot(int idx, double *out_q, double *out_dq, double *out_tau);

#ifdef __cplusplus
}
#endif

#endif /* DM_MOTOR_HPP */
