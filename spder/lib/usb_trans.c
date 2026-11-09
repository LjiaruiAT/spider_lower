/**
 * @file    usb_trans.c
 * @brief   USB CDC 传输层实现
 *
 * ======== 核心机制：USB CDC 分包累积 ========
 *
 * USB CDC 批量传输最大包长 64 字节。上位机发送一帧雷达数据时，
 * USB 协议栈可能把它拆成多个 64 字节分片。
 *
 * 判断帧结束的规则（USB CDC 标准）：
 *   - 收到 < 64 字节的包 → 帧结束
 *   - 收到 == 64 字节的包 → 帧未完，继续收下一包
 *
 * USB_RecvTask 循环收包，动态移动接收缓冲区指针，直到收到非 64 字节包后，
 * 将累积的完整帧通过 Recv_Finished_Cb 回调给上层 (usb_receiver)。
 *
 * ======== 溢出保护 ========
 *
 * 如果连续 64 字节分片超过 USB_CDC_RECV_BUFFER_SIZE - 64：
 *   1. 将下一次 DMA 目标地址回退到上一包的起始位置（覆盖上一包）
 *   2. buffer_index 置为 -1 作为哨兵
 *   3. 若注册了 Recv_Overflow_cb 则回调
 *
 * ======== 任务间通信 ========
 *
 *   CDC_RecvCplt_Handler (ISR)          USB_RecvTask (任务)
 *          │                                  │
 *     xQueueSendFromISR ─── kUsbRecvQueue ──→ xQueueReceive (阻塞等待)
 *     (写入包长度 Len)                        (取出包长度，累加 buffer_index)
 */

#include "usb_trans.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

/* ---------- 外部依赖 (CubeMX 生成) ---------- */

/** @brief USB 设备句柄，CubeMX 生成于 usb_device.c */
extern USBD_HandleTypeDef hUsbDeviceFS;

/** @brief USB CDC 接收缓冲区，CubeMX 生成于 usbd_cdc_if.c，大小 = APP_RX_DATA_SIZE */
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/* ---------- 队列与任务句柄 ---------- */

/** @brief 接收通知队列：ISR 向任务传递每包字节数，长度 2 */
QueueHandle_t kUsbRecvQueue;

/** @brief (保留) 接收信号量队列 */
QueueHandle_t kUsbRecvSemphr;

/** @brief USB 接收任务句柄 */
TaskHandle_t kUsbRecvTaskHandle;

/* ---------- 模块静态变量 ---------- */

/** @brief 上层注册的接收完成回调（指向 UsbRecvCallback） */
static Recv_finished_cb_t Recv_Finished_Cb;

/** @brief 上层注册的溢出回调（本项目未使用） */
static RecvBufferOverflow_cb_t Recv_Overflow_cb;

/** @brief 透传给回调的用户数据 */
static void *kUserData;

/**
 * @brief 当前帧在 UserRxBufferFS 中的累积写位置
 *
 * 每次 USB_RecvTask 循环开始时归零。
 * ISR 每收到一包数据，USB_RecvTask 取出包长度后累加。
 * 溢出时置为 -1 作为哨兵。
 */
int32_t buffer_index = 0;

/**
 * @brief 最新一包的字节数（从 kUsbRecvQueue 中取出）
 *
 * 用作 do-while 循环的终止条件：current_cdc_pack_size < 64 表示帧结束。
 */
uint32_t current_cdc_pack_size = 0;

/**
 * @brief USB CDC 接收任务
 *
 * @param param 未使用
 *
 * ======== 工作流程 ========
 *
 * while(1):
 *   1. buffer_index 归零（新帧开始）
 *   2. 设置下一次 DMA 接收目标 = UserRxBufferFS[0]
 *   3. do-while 循环收包：
 *      a. USBD_CDC_ReceivePacket() 触发 USB 硬件收一包
 *      b. xQueueReceive 阻塞等待 ISR 通知（portMAX_DELAY）
 *      c. 如果下一包会溢出：回退指针到上一包起始，buffer_index = -1
 *      d. 否则：重设 RxBuffer = UserRxBufferFS + 新偏移
 *      e. 直到收到非 64 字节包或溢出，退出循环
 *   4. 如果 buffer_index > 0：回调 Recv_Finished_Cb 投递完整帧
 *   5. 回到步骤 1
 *
 * ======== 关键设计细节 ========
 *
 * - buffer_index = -1 哨兵：溢出后的包被舍弃，但循环仍会继续收包。
 *   正常包可能 < 64 字节，会使 loop 退出但 buffer_index ≤ 0，
 *   于是 Recv_Finished_Cb 不会被调用，数据静默丢弃。
 *
 * - 64 字节判断：USB CDC 批量端点最大包长固定 64，但 USB 协议并不保证
 *   ≤63 才是帧结束——如果上位机恰好发送 64 字节对齐的帧，需要应用层协议
 *   自带帧长信息。本项目的雷达协议使用固定 22 字节帧，不存在此问题。
 */
static void USB_RecvTask(void *param)
{
    while (1)
    {
        buffer_index = 0;

        // 每帧开始：从 UserRxBufferFS[0] 开始接收
        USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);

        do {
            // 触发一次 USB 硬件接收
            USBD_CDC_ReceivePacket(&hUsbDeviceFS);

            // 阻塞等待 ISR 通知（CDC_RecvCplt_Handler 写入队列）
            xQueueReceive(kUsbRecvQueue, &current_cdc_pack_size, portMAX_DELAY);

            // 溢出检查：下次再收 64 字节是否越界
            if (buffer_index + 64 > USB_CDC_RECV_BUFFER_SIZE) {
                // 回退到上一包起始位置，下一包会覆盖此区域（丢弃上一包数据）
                USBD_CDC_SetRxBuffer(&hUsbDeviceFS,
                    UserRxBufferFS + buffer_index - current_cdc_pack_size);
                buffer_index = -1;   // 哨兵：标记溢出状态

                if (Recv_Overflow_cb)
                    Recv_Overflow_cb(kUserData);
            } else {
                // 正常情况：移动接收指针到下个空闲位置
                USBD_CDC_SetRxBuffer(&hUsbDeviceFS,
                    UserRxBufferFS + buffer_index);
            }

            buffer_index = buffer_index + current_cdc_pack_size;

        } while (current_cdc_pack_size == 64);   // 64 字节包 = 未完待续

        // 帧结束（收到 < 64 字节包），且无溢出 → 投递给上层
        if (buffer_index > 0)
            Recv_Finished_Cb(UserRxBufferFS, buffer_index);
    }
}

/**
 * @brief USB CDC 应用层初始化
 *
 * 调用链：Task_Init.c → Remote_Analysis() → UsbReceiver_Init() → USB_CDC_Init()
 *
 * @param recv_cb           接收完成回调（UsbRecvCallback）
 * @param recv_overflow_cb  溢出回调（本项目 = NULL）
 * @param user_data         透传数据（本项目 = NULL）
 */
void USB_CDC_Init(Recv_finished_cb_t recv_cb,
                  RecvBufferOverflow_cb_t recv_overflow_cb,
                  void *user_data)
{
    // 保存回调
    Recv_Finished_Cb  = recv_cb;
    Recv_Overflow_cb  = recv_overflow_cb;
    kUserData         = user_data;

    // 创建 ISR → Task 通知队列（只传包长度，所以队列元素 = uint32_t）
    kUsbRecvQueue = xQueueCreate(2, sizeof(uint32_t));

    // 创建接收任务：栈 128 字，优先级 4
    xTaskCreate(USB_RecvTask, "usb_cdc_recv", 128, NULL, 4, &kUsbRecvTaskHandle);
}

/**
 * @brief USB CDC 接收完成中断回调
 *
 * 调用链：
 *   USB FS ISR → CDC_Rec eive_FS (usbd_cdc_if.c) → CDC_RecvCplt_Handler (本函数)
 *
 * @param Buf 当前接收目标地址（由 USBD_CDC_SetRxBuffer 设定）
 * @param Len 本次收到的字节数（最大 64）
 *
 * 在 ISR 上下文中执行，因此必须使用 FromISR 版本的 FreeRTOS API。
 * 只做最小工作：将包长度投递到队列，唤醒 USB_RecvTask。
 */
void CDC_RecvCplt_Handler(uint8_t *Buf, uint32_t *Len)
{
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

    if (kUsbRecvQueue)
        xQueueSendFromISR(kUsbRecvQueue, Len, &pxHigherPriorityTaskWoken);

    // 如果写入队列导致更高优先级任务就绪，触发上下文切换
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}
