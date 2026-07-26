/**
 * @file    usb_receiver.c
 * @brief   USB 舵机角度帧解析层实现
 */

#include "usb_receiver.h"
#include "usb_trans.h"
#include <string.h>

static UsbServoFrame_t g_latest_frame;
SemaphoreHandle_t g_usb_servo_frame_sem = NULL;

/* 线性缓冲：USB 收到的字节先拼到这里，再按 42 字节协议帧解析 */
static uint8_t ring_buf[USB_FRAME_TOTAL_LEN * 2U];
static uint16_t ring_idx = 0U;

static void RingBuf_Align(void)
{
    uint16_t head_pos = ring_idx;

    /* 对齐双字节帧头 AA 55，丢弃前面的无效字节 */
    for (uint16_t i = 0; i + 1U < ring_idx; i++) {
        if ((ring_buf[i] == USB_FRAME_HEAD1) &&
            (ring_buf[i + 1U] == USB_FRAME_HEAD2)) {
            head_pos = i;
            break;
        }
    }

    if (head_pos > 0U && head_pos < ring_idx) {
        memmove(ring_buf, ring_buf + head_pos, ring_idx - head_pos);
        ring_idx -= head_pos;
    } else if (head_pos == ring_idx) {
        ring_idx = 0U;
    }
}

static uint8_t CalcCheck(const uint8_t *buf)
{
    uint8_t check = 0U;

    /* CHECK 字节 = Byte0~Byte39 逐字节异或 */
    for (uint16_t i = 0; i < (USB_FRAME_TOTAL_LEN - 2U); i++) {
        check ^= buf[i];
    }

    return check;
}

void UsbRecvCallback(uint8_t *src, uint16_t size)
{
    /* 先把本次收到的原始字节拼进缓冲区 */
    for (uint16_t i = 0; i < size; i++) {
        if (ring_idx < sizeof(ring_buf)) {
            ring_buf[ring_idx++] = src[i];
        } else {
            ring_idx = 0U;
            ring_buf[ring_idx++] = src[i];
        }
    }

    while (ring_idx >= USB_FRAME_TOTAL_LEN) {
        UsbServoFrame_t frame;

        RingBuf_Align();
        if (ring_idx < USB_FRAME_TOTAL_LEN) {
            break;
        }

        if ((ring_buf[0] != USB_FRAME_HEAD1) ||
            (ring_buf[1] != USB_FRAME_HEAD2)) {
            ring_buf[0] = 0x00U;
            continue;
        }

        /* 帧头、长度、帧尾、异或校验全部通过后，才认为是一帧有效角度数据 */
        if ((ring_buf[2] != USB_FRAME_PAYLOAD_LEN) ||
            (ring_buf[USB_FRAME_TOTAL_LEN - 1U] != USB_FRAME_TAIL) ||
            (CalcCheck(ring_buf) != ring_buf[USB_FRAME_TOTAL_LEN - 2U])) {
            memmove(ring_buf, ring_buf + 1, ring_idx - 1U);
            ring_idx -= 1U;
            continue;
        }

        frame.seq = ring_buf[3];
        frame.valid = 1U;

        /* 从 Byte4 开始，每 2 字节还原一路舵机角度，格式是 int16_t 小端 */
        for (uint16_t i = 0; i < USB_SERVO_COUNT; i++) {
            uint16_t offset = 4U + (i * 2U);
            frame.angle[i] = (int16_t)((uint16_t)ring_buf[offset] |
                             ((uint16_t)ring_buf[offset + 1U] << 8));
        }

        taskENTER_CRITICAL();
        g_latest_frame = frame;
        taskEXIT_CRITICAL();

        if (g_usb_servo_frame_sem != NULL) {
            xSemaphoreGive(g_usb_servo_frame_sem);
        }

        memmove(ring_buf, ring_buf + USB_FRAME_TOTAL_LEN, ring_idx - USB_FRAME_TOTAL_LEN);
        ring_idx -= USB_FRAME_TOTAL_LEN;
    }
}

void UsbReceiver_Init(void)
{
    if (g_usb_servo_frame_sem == NULL) {
        g_usb_servo_frame_sem = xSemaphoreCreateBinary();
    }

    /* 启动前先清空状态，避免旧数据残留影响第一帧 */
    g_latest_frame.valid = 0U;
    ring_idx = 0U;
    USB_CDC_Init(UsbRecvCallback, NULL, NULL);
}

BaseType_t UsbReceiver_GetLatest(UsbServoFrame_t *frame)
{
    if (frame == NULL) {
        return pdFALSE;
    }

    taskENTER_CRITICAL();
    *frame = g_latest_frame;
    taskEXIT_CRITICAL();

    return (frame->valid != 0U) ? pdTRUE : pdFALSE;
}
