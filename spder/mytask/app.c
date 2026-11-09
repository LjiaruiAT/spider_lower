#include "app.h"
#include "tim.h"
#include "usb_receiver.h"

TaskHandle_t leg_cl_Task_handle;
TaskHandle_t get_data_Task_handle;

/* 每路舵机对应一个 PWM 输出通道，并预留方向/零位修正 */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    int8_t direction;
    int16_t offset_ddeg;
} ServoMap_t;

#define SERVO_MIN_DDEG          0
#define SERVO_MAX_DDEG          1800
#define SERVO_CENTER_DDEG       900
#define SERVO_MIN_PULSE_US      500U
#define SERVO_MAX_PULSE_US      2500U
#define SERVO_PERIOD_COUNTS     1000U
#define SERVO_PERIOD_US         20000U
#define APP_TASK_PERIOD_TICKS   pdMS_TO_TICKS(10U)
#define BOOT_POSE_HOLD_TICKS    pdMS_TO_TICKS(1000U)

/*
 * 开机初始姿态（单位：0.1°）
 *
 * 当前先给出一个占位模板，方便你后面按实际机械结构填写。
 * 建议按“左前、右前、左中、右中、左后、右后”以及每条腿 3 个关节的顺序校准。
 *
 * 例如：
 *   Coxa  负责前后/左右展开
 *   Femur 负责抬腿
 *   Tibia 负责小腿弯曲
 *
 * 现在默认全部给 90.0°，保证编译可用。
 * 后续你只需要改这 18 个值，不需要改下面逻辑。
 */
static const int16_t kBootPose[USB_SERVO_COUNT] = {
    900, 900, 900,
    900, 900, 900,
    900, 900, 900,
    900, 900, 900,
    900, 900, 900,
    900, 900, 900
};

/* 18 路舵机按 main.c 中 PWM 启动顺序映射到 TIM 通道 */
static const ServoMap_t kServoMap[USB_SERVO_COUNT] = {
    {&htim1, TIM_CHANNEL_1,  1, 0},
    {&htim1, TIM_CHANNEL_2,  1, 0},
    {&htim1, TIM_CHANNEL_3,  1, 0},
    {&htim1, TIM_CHANNEL_4,  1, 0},
    {&htim2, TIM_CHANNEL_1,  1, 0},
    {&htim2, TIM_CHANNEL_2,  1, 0},
    {&htim2, TIM_CHANNEL_3,  1, 0},
    {&htim2, TIM_CHANNEL_4,  1, 0},
    {&htim3, TIM_CHANNEL_1,  1, 0},
    {&htim3, TIM_CHANNEL_2,  1, 0},
    {&htim3, TIM_CHANNEL_3,  1, 0},
    {&htim3, TIM_CHANNEL_4,  1, 0},
    {&htim4, TIM_CHANNEL_1,  1, 0},
    {&htim4, TIM_CHANNEL_2,  1, 0},
    {&htim4, TIM_CHANNEL_3,  1, 0},
    {&htim4, TIM_CHANNEL_4,  1, 0},
    {&htim5, TIM_CHANNEL_1,  1, 0},
    {&htim5, TIM_CHANNEL_2,  1, 0},
};

static UsbServoFrame_t g_servo_frame;
static uint8_t g_usb_receiver_ready = 0U;
static TickType_t g_boot_pose_ready_tick = 0U;

/* 协议角度限幅到 0.0°~180.0° */
static int16_t ClampAngleDdeg(int32_t angle_ddeg)
{
    if (angle_ddeg < SERVO_MIN_DDEG) {
        return SERVO_MIN_DDEG;
    }
    if (angle_ddeg > SERVO_MAX_DDEG) {
        return SERVO_MAX_DDEG;
    }
    return (int16_t)angle_ddeg;
}

static int16_t ApplyServoCalibration(uint16_t index, int16_t angle_ddeg)
{
    int32_t corrected = angle_ddeg;

    /* direction=-1 时按 180° 对称翻转，再叠加零位偏移 */
    if (kServoMap[index].direction < 0) {
        corrected = SERVO_MAX_DDEG - corrected;
    }

    corrected += kServoMap[index].offset_ddeg;
    return ClampAngleDdeg(corrected);
}

static uint32_t AngleDdegToCompare(int16_t angle_ddeg)
{
    uint32_t pulse_us;

    /* 线性映射：0°->500us, 180°->2500us */
    pulse_us = SERVO_MIN_PULSE_US +
               (((uint32_t)angle_ddeg * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) +
                (SERVO_MAX_DDEG / 2)) / SERVO_MAX_DDEG;

    return (pulse_us * SERVO_PERIOD_COUNTS) / SERVO_PERIOD_US;
}

/* 把 18 路角度依次写到 18 路 PWM，完成一次整机舵机刷新 */
static void ApplyServoAngles(const int16_t *angles)
{
    for (uint16_t i = 0; i < USB_SERVO_COUNT; i++) {
        int16_t corrected = ApplyServoCalibration(i, ClampAngleDdeg(angles[i]));
        uint32_t compare = AngleDdegToCompare(corrected);

        /* PWM 已在 main.c 启动，这里只更新 CCR */
        __HAL_TIM_SET_COMPARE(kServoMap[i].htim, kServoMap[i].channel, compare);
    }
}

void Task_Init(void)
{
    xTaskCreate(get_data, "get_data_Task", 128 * 4, NULL, 4, &get_data_Task_handle);
    xTaskCreate(leg_cl, "leg_cl_Task", 128 * 4, NULL, 4, &leg_cl_Task_handle);
}

void get_data(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    UsbServoFrame_t frame;
    (void)pvParameters;

    /* 只初始化一次 USB 接收层，并把默认姿态设为开机初始姿态
       这样即使上位机还没发第一帧，蜘蛛也会先进入一个固定姿态等待。 */
    if (g_usb_receiver_ready == 0U) {
        UsbReceiver_Init();
        for (uint16_t i = 0; i < USB_SERVO_COUNT; i++) {
            g_servo_frame.angle[i] = kBootPose[i];
        }
        g_servo_frame.valid = 1U;
        g_boot_pose_ready_tick = xTaskGetTickCount() + BOOT_POSE_HOLD_TICKS;
        g_usb_receiver_ready = 1U;
    }

    for (;;) {
        /* 先给蜘蛛 1 秒时间保持开机姿态，等初始化动作完成后再接收上位机数据。 */
        if (xTaskGetTickCount() < g_boot_pose_ready_tick) {
            vTaskDelayUntil(&xLastWakeTime, APP_TASK_PERIOD_TICKS);
            continue;
        }

        /* 有新协议帧到达时，刷新共享角度缓存。
           get_data 相当于“通信接收任务”，只负责把最新目标角度拿到手。 */
        if ((g_usb_servo_frame_sem != NULL) &&
            (xSemaphoreTake(g_usb_servo_frame_sem, 0) == pdTRUE) &&
            (UsbReceiver_GetLatest(&frame) == pdTRUE)) {
            taskENTER_CRITICAL();
            g_servo_frame = frame;
            taskEXIT_CRITICAL();
        }

        vTaskDelayUntil(&xLastWakeTime, APP_TASK_PERIOD_TICKS);
    }
}

void leg_cl(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    UsbServoFrame_t frame;
    (void)pvParameters;

    /* leg_cl 相当于“舵机执行任务”：
       周期读取最新角度，把它们转换成 PWM compare，然后下发到 18 路 TIM。 */
    for (;;) {
        /* 再次读取最新帧，避免输出任务长期使用旧缓存 */
        if (UsbReceiver_GetLatest(&frame) == pdTRUE) {
            taskENTER_CRITICAL();
            g_servo_frame = frame;
            taskEXIT_CRITICAL();
        }

        taskENTER_CRITICAL();
        frame = g_servo_frame;
        taskEXIT_CRITICAL();

        /* 只有收到过至少一帧合法数据，才真的去刷新舵机输出 */
        if (frame.valid != 0U) {
            ApplyServoAngles(frame.angle);
        }

        /* 第一版策略：如果暂时没新帧，就保持最后一次有效角度。
           也就是说：开机先走 kBootPose，保持 1 秒初始化姿态，之后再持续跟随上位机最新目标角度。 */
        vTaskDelayUntil(&xLastWakeTime, APP_TASK_PERIOD_TICKS);
    }
}
