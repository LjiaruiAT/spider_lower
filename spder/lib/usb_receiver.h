#ifndef __USB_RECEIVER_H__
#define __USB_RECEIVER_H__

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdint.h>

#define USB_SERVO_COUNT        18U
#define USB_FRAME_HEAD1        0xAAU
#define USB_FRAME_HEAD2        0x55U
#define USB_FRAME_PAYLOAD_LEN  36U
#define USB_FRAME_TOTAL_LEN    42U
#define USB_FRAME_TAIL         0xBBU

typedef struct {
    int16_t angle[USB_SERVO_COUNT];
    uint8_t seq;
    uint8_t valid;
} UsbServoFrame_t;

extern SemaphoreHandle_t g_usb_servo_frame_sem;

void UsbReceiver_Init(void);
BaseType_t UsbReceiver_GetLatest(UsbServoFrame_t *frame);

#endif /* __USB_RECEIVER_H__ */
