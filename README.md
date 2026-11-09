# Spider Lower — 六足机器人下位机

18 路舵机 PWM 驱动 · STM32F407 + FreeRTOS · USB CDC 接收上位机帧
**Status: 🟡 单腿实机验证通过，整机装配与 18 路标定进行中**

> 六足机器人下半部分（执行层）。上位机（ROS 2 + 六腿 IK）见 [spider_upper](https://github.com/LjiaruiAT/spider_upper)。

---

## 1. 作用

上位机算出 18 个关节角（6 条腿 × coxa / femur / tibia），通过 USB CDC 打包下发；本固件负责把角度换算成 PWM 比较值，输出到 18 路舵机，并处理校准与限位。

上下位机分工：**所有运动学在 ROS 2 侧**，STM32 侧只做「角度 → 脉宽」的确定性映射，不做插值或规划——这样谁出问题一眼能定位。

---

## 2. 实现

| 项 | 值 | 位置 |
|---|---|---|
| 主控 | STM32F407VETx | `spder/spder.ioc`（CubeMX） |
| 实时内核 | FreeRTOS，10 ms 任务周期 | `Core/Src/freertos.c` |
| 舵机路数 | 18（TIM1/2/3/4/5 全部 4 通道 + TIM5 前 2 路） | `mytask/app.c` |
| 角度分辨率 | 0.1°（0 ~ 1800） | `SERVO_MIN_DDEG` / `SERVO_MAX_DDEG` |
| 脉宽范围 | 500 ~ 2500 µs，周期 20 ms | `SERVO_MIN_PULSE_US` / `SERVO_PERIOD_US` |
| 通信 | USB CDC，接收 `UsbServoFrame_t` 帧 | `usb_receiver.h` |
| 开机姿态 | 上电输出 `kBootPose[18]`，保持 1 s 后进入正常接收 | `BOOT_POSE_HOLD_TICKS` |

### 校准机制

每路舵机在 `kServoMap[]` 中独立配置：

```c
{ &htim1, TIM_CHANNEL_1, direction, offset_ddeg }
```

- `direction = -1`：按 180° 对称翻转（舵机反装）
- `offset_ddeg`：零位偏移（±0.1°）
- 校准后统一限幅到 0 ~ 180°

这层配置让"换一个装反的舵机"不需要改任何逻辑代码。

---

## 3. 文件

```
spder/
├── Core/          CubeMX：TIM（18 路 PWM）/ USB / DMA / FreeRTOS
├── mytask/
│   ├── app.c      舵机映射、校准、PWM 输出、开机姿态
│   ├── app.h
│   └── config.h   参数
├── lib/           usb_receiver.c（CDC 帧解析）/ usb_trans.c（发送）
├── USB_DEVICE/    CubeMX 生成的 CDC 设备层
└── MDK-ARM/       Keil 工程（EIDE 配置见 .eide/）
```

---

## 4. 状态与限制

**已完成**
- 18 路 PWM 输出与通道映射
- 方向翻转 + 零位偏移校准结构
- USB CDC 帧接收与角度限幅
- 开机姿态保持

**未完成 / 限制**
- 仅单腿（3 路）完成实机验证，**整机 18 路尚未全部装配与标定**
- `kBootPose[]` 目前是占位值（全部 90.0°），需按实际机械结构填写
- 无单元测试；无断线保护（上位机断连后保持最后一帧角度）
- 仓库目录名 `spder` 是笔误（应为 `spider`），保留原样以免破坏已有引用

---

## 5. 编译

Keil MDK 打开 `MDK-ARM/spder.uvprojx`，或使用 `MDK-ARM/.eide/` 的 EIDE 配置。
新增源文件需同时加入工程分组，否则不参与编译。
