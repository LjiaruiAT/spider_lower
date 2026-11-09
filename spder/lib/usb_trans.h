/**
 * @file    usb_trans.h
 * @brief   USB CDC 传输层 —— HAL 适配 + 分包累积 + 回调投递
 *
 * 本模块是雷达链路的最底层，直接操作 STM32F4 USB FS 外设。
 * 上层是 usb_receiver（帧解析层），本模块不关心帧协议内容。
 *
 * 工作模式：
 *   USB CDC 以最大 64 字节/包收发。上位机发送的数据可能超过 64 字节，
 *   本模块在 USB_RecvTask 中循环收包，将连续的分片拼成一帧再回调上层。
 *
 *                USB 硬件 ISR
 *                    │
 *                    ▼
 *         CDC_RecvCplt_Handler (ISR)
 *              ┊  xQueueSendFromISR
 *              ┊  kUsbRecvQueue ← 每包长度
 *              ▼
 *         USB_RecvTask (后台任务, 优先级 4)
 *              ┊  循环: SetRxBuffer → ReceivePacket → QueueReceive
 *              ┊  累积到非 64B 包或溢出时调用 Recv_Finished_Cb
 *              ▼
 *         UsbRecvCallback (usb_receiver.c)
 */

#ifndef __USB_TRANS_H__
#define __USB_TRANS_H__

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/** @brief USB CDC 接收缓冲区大小 (字节)，与 CubeMX 生成的 APP_RX_DATA_SIZE 保持一致 */
#define USB_CDC_RECV_BUFFER_SIZE    512

/* 前向声明 */
typedef struct CDC_SendReq_t  CDC_SendReq_t;
typedef struct CDC_RecvReq_t  CDC_RecvReq_t;

/**
 * @brief 数据包接收完成回调
 * @param src  完整帧数据的起始地址（指向 UserRxBufferFS）
 * @param size 累计收到的字节总数
 * @note  回调在 USB_RecvTask 上下文中执行，不是 ISR
 */
typedef void (*Recv_finished_cb_t)(uint8_t *src, uint16_t size);

/**
 * @brief 接收缓冲区溢出回调
 * @param user_data 用户数据指针（初始化时传入）
 * @note  当连续 64 字节分片累积快超出 USB_CDC_RECV_BUFFER_SIZE 时触发，
 *        本项目中未使用（传入 NULL），usb_trans 内部自行回退缓冲区指针处理
 */
typedef void (*RecvBufferOverflow_cb_t)(void *user_data);

/**
 * @brief USB 接收队列句柄
 *
 * CDC_RecvCplt_Handler(ISR) 向此队列发送每包长度，
 * USB_RecvTask 从中取出并累加 buffer_index。
 */
extern QueueHandle_t kUsbRecvQueue;

/** @brief (保留) 发送完成信号量队列，当前版本未使用 */
extern QueueHandle_t kUsbSendsemphr;

/** @brief (保留) 发送请求队列，当前版本未使用 */
extern QueueHandle_t kUsbSendReqQueue;

/**
 * @brief USB CDC 应用层初始化
 *
 * @param recv_cb           接收完成回调（必须提供，本项目中 = UsbRecvCallback）
 * @param recv_overflow_cb  溢出回调（可选，本项目中 = NULL）
 * @param user_data         透传给回调的用户数据（本项目中 = NULL）
 *
 * 1. 保存回调指针到模块静态变量
 * 2. 创建 kUsbRecvQueue（长度 2）
 * 3. 创建 USB_RecvTask（栈 128 字，优先级 4）
 */
void USB_CDC_Init(Recv_finished_cb_t recv_cb,
                  RecvBufferOverflow_cb_t recv_overflow_cb,
                  void *user_data);

/**
 * @brief USB CDC 接收完成中断回调（由 STM32 HAL 库调用）
 *
 * @param Buf 当前 USB 接收缓冲区指针（UserRxBufferFS + 偏移）
 * @param Len 本次收到的字节数
 *
 * 在 USB CDC 接收中断上下文中执行。将 Len 通过队列发给 USB_RecvTask。
 * 本项目中由 CubeMX 生成的 usbd_cdc_if.c 中 CDC_Receive_FS 回调触发。
 */
void CDC_RecvCplt_Handler(uint8_t *Buf, uint32_t *Len);

#endif /* __USB_TRANS_H__ */
