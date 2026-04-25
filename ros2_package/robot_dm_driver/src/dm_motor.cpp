#include "robot_dm_driver/dm_motor.hpp"
#include <mutex>

/* 全局电机表 */
dm_motor_t g_motors[DM_MAX_MOTORS];
int g_motor_count = 0;

/* 保护 g_motors[*].state_q/dq/tau 的互斥锁，
   由 dm_receive_callback（CAN驱动线程）写，
   由 dm_motor_snapshot（ros2_control 控制循环线程）读 */
static std::mutex g_motors_state_mutex;

/* 内部保存设备句柄，供 dm_control_disable 使用 */
static device_handle *g_dev = NULL;

/* 各型号电机限制参数 */
static const dm_limit_param_t g_limit_params[DM_MOTOR_TYPE_COUNT] = {
    {12.566f,  50.0f,  5.0f},   /* DM3507 */
    {3.14159f, 45.0f, 54.0f},   /* DM4310 */
    {3.14159f, 45.0f, 54.0f},   /* DM4340 */
}; 

static uint16_t float_to_uint(float x, float xmin, float xmax, uint8_t bits)
{
    float span = xmax - xmin;
    float data_norm = (x - xmin) / span;
    uint16_t data_uint = (uint16_t)(data_norm * ((1 << bits) - 1));
    return data_uint;
}

static float uint_to_float(uint16_t x, float xmin, float xmax, uint8_t bits)
{
    float span = xmax - xmin;
    float data_norm = (float)x / ((1 << bits) - 1);
    return data_norm * span + xmin;
}

int dm_motor_add(dm_motor_type_t type, uint16_t can_id, uint16_t master_id)
{
    if (g_motor_count >= DM_MAX_MOTORS) {
        printf("[Error] motor table full\n");
        return -1;
    }
    dm_motor_t *m = &g_motors[g_motor_count];
    memset(m, 0, sizeof(dm_motor_t));
    m->can_id = can_id;
    m->master_id = master_id;
    m->motor_type = type;
    m->limit = g_limit_params[type];
    m->active = 1;
    g_motor_count++;
    printf("[Info] motor added: can_id=0x%02X master_id=0x%02X\n", can_id, master_id);
    return 0;
}

dm_motor_t* dm_motor_find(uint16_t id)
{
    for (int i = 0; i < g_motor_count; i++) {
        if (g_motors[i].active &&
            (g_motors[i].can_id == id || g_motors[i].master_id == id)) {
            return &g_motors[i];
        } 
    }
    return NULL;
}

void dm_init_channel(device_handle *dev, uint8_t channel, bool canfd,
                     int bitrate, int dbitrate, float can_sp, float canfd_sp)
{
    g_dev = dev;
    device_channel_set_baud_with_sp(dev, channel, canfd, bitrate, dbitrate, can_sp, canfd_sp);
    printf("**********DM Motor channel init successful**********\n");
}

void dm_control_enable(device_handle *dev)
{
    g_dev = dev;
    uint8_t payload[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    for (int i = 0; i < g_motor_count; i++) {
        if (g_motors[i].active) {
            device_channel_send_fast(dev, 0, g_motors[i].can_id, 1,
                                     false, true, true, 8, payload);
        }
    }
    printf("[Info] all motors enabled\n");
}

void dm_control_disable()
{
    if (g_dev == NULL) {
        printf("[Error] dm_control_disable: device not initialized\n");
        return;
    }
    uint8_t payload[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    for (int i = 0; i < g_motor_count; i++) {
        if (g_motors[i].active) {
            device_channel_send_fast(g_dev, 0, g_motors[i].can_id, 1,
                                     false, true, true, 8, payload);
        }
    }
    printf("[Info] all motors disabled\n");
}

void dm_control_mit(device_handle *dev, uint16_t can_id,
                    float kp, float kd, float q, float dq, float tau)
{
    dm_motor_t *m = dm_motor_find(can_id);
    if (m == NULL) {
        printf("[Error] dm_control_mit: motor id 0x%02X not found\n", can_id);
        return;
    }

    uint16_t kp_uint  = float_to_uint(kp,  0, 500, 12);
    uint16_t kd_uint  = float_to_uint(kd,  0, 5,   12);
    uint16_t q_uint   = float_to_uint(q,   -m->limit.q_max,   m->limit.q_max,   16);
    uint16_t dq_uint  = float_to_uint(dq,  -m->limit.dq_max,  m->limit.dq_max,  12);
    uint16_t tau_uint = float_to_uint(tau, -m->limit.tau_max, m->limit.tau_max, 12);

    uint8_t data[8];
    data[0] = (q_uint >> 8) & 0xff;
    data[1] = q_uint & 0xff;
    data[2] = dq_uint >> 4;
    data[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
    data[4] = kp_uint & 0xff;
    data[5] = kd_uint >> 4;
    data[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
    data[7] = tau_uint & 0xff;

    device_channel_send_fast(dev, 0, can_id, 1, false, true, true, sizeof(data), data);
}

void dm_receive_callback(usb_rx_frame_t *frame)
{
    uint16_t canID = (uint16_t)frame->head.can_id;

    dm_motor_t *m = dm_motor_find(canID);
    if (m == NULL) {
        return;
    }

    uint16_t q_uint   = ((uint16_t)frame->payload[1] << 8) | frame->payload[2];
    uint16_t dq_uint  = ((uint16_t)frame->payload[3] << 4) | (frame->payload[4] >> 4);
    uint16_t tau_uint = ((uint16_t)(frame->payload[4] & 0xf) << 8) | frame->payload[5];

    double new_q   = uint_to_float(q_uint,   -m->limit.q_max,   m->limit.q_max,   16);
    double new_dq  = uint_to_float(dq_uint,  -m->limit.dq_max,  m->limit.dq_max,  12);
    double new_tau = uint_to_float(tau_uint, -m->limit.tau_max, m->limit.tau_max, 12);

    {
        std::lock_guard<std::mutex> lock(g_motors_state_mutex);
        m->state_q   = new_q;
        m->state_dq  = new_dq;
        m->state_tau = new_tau;
    }
}

void dm_motor_snapshot(int idx, double *out_q, double *out_dq, double *out_tau)
{
    std::lock_guard<std::mutex> lock(g_motors_state_mutex);
    *out_q   = g_motors[idx].state_q;
    *out_dq  = g_motors[idx].state_dq;
    *out_tau = g_motors[idx].state_tau;
}
