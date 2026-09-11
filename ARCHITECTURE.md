# ARCHITECTURE.md

STM32F407 AGV firmware — 完整架构参考文档。按需查阅，不随每次会话自动加载。

> **提示**：在日常编码中不需要逐页阅读本文。遇到以下情况时查阅对应章节：
> -   修改 GPIO / 外设 / 中断 → §1 Hardware Pins → §2 ISR
> -   修改循迹控制 → §3 Line-Follow
> -   修改视觉/OpenMV → §4 OpenMV → §5 Visual Servo → §6 Vision → §7 AI Pipeline
> -   修改机械臂动作 → §11.3 Servo Power → §11.4 ArmSM
> -   修改串口/CSV → §9 CSV Protocol
> -   调试启动 → §12 Startup → §13 Main Loop
> -   定位 Bug → §15 Bug Tracking

---

## 1. Hardware Pin Mapping (Full)

### DC Motors (4 differential-drive wheels, left/right pairs)

| ID | Name | PWM (Timer) | IN1 | IN2 |
|----|------|-------------|-----|-----|
| 0 | 左前 | PA7 (TIM3_CH2) | PE3 | PE4 |
| 1 | 右前 | PA6 (TIM3_CH1) | PE5 | PE6 |
| 2 | 左后 | PB1 (TIM3_CH4) | PC8 | PC4 |
| 3 | 右后 | PB15 (TIM12_CH2) | PD3 | PD4 |

Motor PWM: **1kHz** (ARR=999, PSC=83, 84MHz base), duty 0–999.
Speed range: ±1000 (mapped to duty via `speed * 999 / 1000`).

### Servos (4-DOF arm)

| User ID | Joint | PWM (Timer) | Angle range |
|---------|-------|-------------|-------------|
| 0 | 腰座 (waist) | PA8 (TIM1_CH1) | 0–180° |
| 1 | 大臂 (shoulder) | PA9 (TIM1_CH2) | 0–180° |
| 2 | 小臂 (elbow) | PA2 (TIM9_CH1) | 0–180° |
| 3 | 夹爪 (gripper) | PA11 (TIM1_CH4) | 0–180° |

Servo PWM: 50Hz (ARR=19999, PSC=167, 168MHz base), pulse 500–2500.
Note: `pwm.h` defines both `SERVO_ELBOW` (TIM1_CH3/PA10, unused zombie channel) and `SERVO_ELBOW_PA2` (TIM9_CH1/PA2, active). The elbow was remapped from PA10 to PA2 but the old define remains.

### Quadrature Encoders

| ID | Timer | CH1 | CH2 |
|----|-------|-----|-----|
| 0 (左前) | TIM5 | PA0 | PA1 |
| 1 (右前) | TIM2 | PA15 | PB3 |
| 2 (左后) | TIM4 | PB6 | PB7 |
| 3 (右后) | TIM8 | PC6 | PC7 |

### Communication & Sensors

| Interface | Peripheral | Pins | Notes |
|-----------|-----------|------|-------|
| Debug UART | UART5 (115200) | PC12(TX), PD2(RX) | Primary command + CSV telemetry |
| Bluetooth | USART2 (9600) | PD5(TX), PD6(RX) | **Currently disabled** — implemented in `hc_sr40.c/h` (misnamed, NOT ultrasonic). `BT_Init()` commented out, ISR `#if 0`'d. USART2 cannot coexist with UART5 at current NVIC priorities (both would need prio 0,0 to beat TIM6) |
| Line sensor ×5 | GPIO input | PC0, PC1, PC2, PC3, PA4 | Active-low, pull-up, weighted error |
| OLED | I2C | (see `OLEDIIC.c`) | Currently disconnected |
| OpenMV | USART3 (115200) | PC10(TX), PC11(RX) | Camera vision data (`$TAG` packets) |
| CountSensor | EXTI1 on PB1 | PB1 | External interrupt counter; ISR exists but body is empty — hardware is wired but unused |

---

## 2. ISR Priority Layout (Full)

NVIC priorities (lower number = higher priority; preemption priority, subpriority format):

| ISR | Peripheral | File | Priority (pre,sub) | Role |
|-----|-----------|------|-------|------|
| `PVD_IRQHandler` | PVD | `Timer.c` | 0,0 | Brownout: immediate motor brake |
| `UART5_IRQHandler` | UART5 | `uart.c` | 0,0 | Debug UART RX ring buffer (tied with PVD as highest) |
| `TIM6_DAC_IRQHandler` | TIM6 | `Timer.c` | 1,0 | 100Hz control heartbeat |
| `USART2_IRQHandler` | USART2 | `hc_sr40.c` | 2,0 | Bluetooth RX **(disabled via `#if 0`)** |
| `USART3_IRQHandler` | USART3 | `commend_openmv.c` | 2,0 | OpenMV camera RX ring buffer |
| `EXTI1_IRQHandler` | EXTI1 | `CountSensor.c` | 3,3 | Counter input **(empty handler)** |

**Key implication**: UART5 at prio 0,0 can preempt TIM6 (prio 1,0). During heavy CSV telemetry, UART5 ISR steals cycles from the 100Hz control loop. USART2 (BT) is disabled because it would need prio 0,0 to not lose data at 9600 baud, but that conflicts with UART5. If BT is re-enabled, the NVIC priorities must be rebalanced.

---

## 3. Control System Design (Detailed)

**Timer interrupt**: TIM6 fires at **100Hz (10ms)** — the central control heartbeat (`User/Timer.c:TIM6_DAC_IRQHandler`).

**Four operating modes** (`work_mode`):
- `MODE_MANUAL (0)` — Serial commands directly set `target_speed[]`
- `MODE_LINE_FOLLOW (1)` — Line sensor PID → `target_speed[]` per 10ms. Supports timed stop via `line_time_ticks`: when `line_time_ticks > 0 && g_sys_tick >= line_time_ticks`, motors are stopped and mode switches to MANUAL. The `line_time <sec>` serial command sets this (argument in seconds, internally converted to absolute tick).
- `MODE_POSITION (2)` — Position PID outer loop → `target_speed[]` per 20ms
- `MODE_VISUAL_SERVO (3)` — OpenMV vision-guided approach (`#define MODE_VISUAL_SERVO 3` at `visual_servo.h:7`; fully wired into the TIM6 ISR dispatch at `Timer.c:95`)

**Nested PID loops**:
1. **Position loop** (20ms, positional PID `pid_positon[]`): Target position → speed command (±600)
2. **Speed loop** (10ms, incremental PID `pid_motor[]`): Speed command → motor PWM output (±1000)
3. **Line-follow loop** (10ms, positional PID in `line_follow`): Sensor error → turn correction → modifies `target_speed[]`

### 3.1 Line-Follow PID

**Line sensor array** (`User/line_sensor.h`): 5-channel IR on PC0–PC3 + PA4. GPIO reads compared `!= 0` so `states[i]==1` means line detected (header says active-low). Weighted error `{-2,-1,0,1,2}` normalized by detection count → error ±2.

**Line-follow PID** (`User/line_follow.c`): Features sensor debounce (3-sample majority vote in `LineSensor_Read`), deadband on error (±0.1 → zeroed), EMA on derivative (70% old + 30% new), integral separation at ±1.5 error threshold, trapezoidal integration, and turn output clamping at ±600. Output speeds clamped to [0, 1000] (forward-only; inner wheel stops but doesn't reverse on sharp turns). Turn is added to left wheels and subtracted from right wheels (differential steering).

**Line-loss behavior**: When no sensor detects the line, motors coast at 60% of last known speed (±30 min) for 50 ticks (500ms), then integrals zeroed and motors stop. Prevents nuisance stops from brief dropouts.

**Curve detection**: Built-in curve counter (`curve_count`, `curve_target`, `curve_ready` fields in `Line_follow_Handle`). A "curve" is detected when ≤4 of 5 sensors see the line for 3 consecutive frames. After `curve_target` curves + `curve_delay_ms` delay, `curve_ready` is set to 1. This can be used to trigger actions after passing a known number of curves on the track.

**Motor 3 right-rear compensation**: Motor 3 encoder is broken → open-loop control. In line-follow, `right - 20` is applied to motor 3 speed to compensate for its tendency to run faster without encoder feedback. This hardware-specific hack is only in the line-follow path; other modes use `encoder_max[3] = 0` open-loop fallback in the TIM6 ISR.

**Motor direction polarity**: `motor_dir[] = {1, -1, 1, -1}` corrects for motor mounting orientation so positive `target_speed` always means forward.

---

## 4. OpenMV Vision Subsystem

**OpenMV Vision Communication** (`User/commend_openmv.c/h`): USART3 (PC10=TX, PC11=RX, 115200 baud) dedicated to the OpenMV camera. Uses the same ring-buffer ISR + polling pattern as the debug UART. Parses data packets at ~10–50Hz. Three data structures are maintained:

- **`OpenMV_Data`** — AprilTag/color blob tracking: `tag_id`, `cx`, `cy`, `distance_cm`, `angle_deg`, `pixel_width`. Used by visual servo and vision_task.
- **`OpenMV_QRData`** — QR code decoded text (up to 64 chars). Used by the AI auto-scan main-loop pipeline.
- **`OpenMV_CLSData`** — AI classification result: `class_id` (-1=uncertain) and `confidence` (0–100%). Used by the AI auto-scan main-loop pipeline.

All three use a `fresh` flag pattern for ISR-to-main-loop producer-consumer sync. ISR-safe `OpenMV_TagSeenRecently()` queries tag presence by timestamp without consuming data. `OpenMV_PollLine()` drains the ring buffer in the main loop. OpenMV mode is set via `OpenMV_SendCmd("MODE,<mode>")` where mode is `APRILTAG`, `COLOR`, `QRCODE`, or `AI`.

**Serial packet formats** (USART3, 115200 baud):
- `$TAG,<id>,<cx>,<cy>,<dist_cm>,<angle_deg>,<width>` — AprilTag/color blob detection
- `$QR,<payload>` — QR code decoded text
- `$CLS,<class_id>,<confidence>` — AI classification result (class_id=-1 means uncertain)
- `$HB` — Heartbeat (aliveness check via `OpenMV_IsAlive()`)

Commands are sent as `$CMD,<command>\r\n` (e.g. `$CMD,MODE,AI\r\n`).

**OpenMV firmware**: `OpenMV_scripts/main.py` (source backup). The active firmware is on the H7 Plus internal flash (H: drive when USB-connected). Utilities: `train.py`, `capture.py`, `test_*.py`. Models: `model_mid.tflite` (active), `model.tflite`, `model_small.tflite`. Legacy: `vision_main.py` (5-state-machine, not active).

**Model loading**: H7 Plus has 32MB internal flash — no SD card needed. The active model `model_mid.tflite` (~630KB, MobileNetV2 α=0.35 INT8) is loaded via `ml.Model(load_to_fb=True)` from `/flash/`. The model is lazy-loaded on first `MODE,AI` command to avoid blocking startup.

**QR debounce**: QR codes require 3 consecutive frame confirmations; on loss, output is held for 5 frames before clearing. This adds ~150–250ms latency to QR detection/loss transitions.

---

## 5. Visual Servo

**Visual Servo** (`User/visual_servo.c/h`): Implements `MODE_VISUAL_SERVO (3)` — dual PID for autonomous AprilTag docking:
- **Lateral PID**: Steers to center the tag horizontally (target `cx=160`, 320×240 image). Output clamped to ±turn_limit (default 400).
- **Longitudinal PID**: Controls forward/backward speed to reach the target distance. Output clamped to ±base_speed_max (default 150).
- Outputs combined via differential steering: `left = forward + turn`, `right = forward - turn`.
- Features integral separation on both axes, IIR low-pass filter on derivatives (70% new + 30% old), trapezoidal integration, loss-of-tag handling (stops motors after configurable timeout, default 200ms), and hysteresis-based alignment detection (N consecutive frames within tolerances, default 5 frames).
- Follows the same handle-pattern convention as `Line_follow_Handle` — `VisualServo_Task()` is called at 100Hz from the TIM6 ISR.

---

## 6. Vision Task Mission Orchestrator

**Vision Task** (`User/vision_task.c/h`): A high-level, non-blocking state machine that coordinates OpenMV vision data with line-following and arm actions. Three mission types:
- **Color search** (`VIS_COLOR_SEARCH → VIS_COLOR_APPROACH → VIS_COLOR_GRASP → VIS_DONE`): Line-follow until target color blob detected → approach using centroid-based steering → grasp at close range.
- **QR code search** (`VIS_QR_SEARCH → VIS_QR_APPROACH → VIS_QR_EXECUTE → VIS_DONE`): Line-follow until QR code detected → approach → execute decoded command (supports `GRASP` and `PLACE` keywords; defaults to grasp if no command).
- **AI classification scan** (`VIS_AI_SCAN → VIS_AI_APPROACH → VIS_AI_EXECUTE → VIS_DONE`): Stop and classify objects via OpenMV's onboard model → approach detected target → execute action based on class ID.

Each mission type sets the OpenMV mode (`OMV_MODE_COLOR`, `OMV_MODE_QRCODE`, `OMV_MODE_AI`) and switches to line-follow or manual mode as needed. Missions have configurable approach/grasp distance thresholds and 1s target-loss timeout (100 ticks). `VisionTask_Process()` is called from `main()` and throttles to 50ms intervals. State queries via `VisionTask_IsBusy()` / `VisionTask_GetState()`.

---

## 7. Main-Loop AI Pipeline (Primary Autonomous Mission, QR-triggered)

Located inline in `main.c` (`qr_phase` 0–11 state machine, main loop). This is the **active** autonomous mission orchestrator — it replaced the older `#if 0`'d timed auto-mission. **QR 触发**: the pipeline starts by scanning a QR code, approaches to 15cm, then runs AI classification / color alignment / grasp / place. It only activates after docking completes (`dock_state == 5`, `main.c:615`), so it never races the dock state machine. Two stations:

```
Station 0 (抓取放从车): QR 触发 → approach 15cm → 识别(AI 分类, 腰座±20° 扫描) → QuickAlign 色块对准 → 抓取 → 放置到从车(180°) → 下一件
Station 1 (从车取件交付): QR→AI→COLOR 识别从车上物件 → 取件 → 交付
任务序列: mission_idx 0→2, mission_class[] = {0,2,1} (红→黄→绿); 两站跑完 mission_done
关键 phase: 0=QR触发+approach, 1=识别(腰座±20°扫描), 10=QuickAlign 色块对准, 3=抓取,
           11=放置(180°), 7=AprilTag 定位, 9=AI(skip_pick), 8=Station0 下一件, 4/5/6=Station1 取件交付
每阶段独立超时兜底 + `restart`/`run` 一键恢复
```

The pipeline coexists with `VisionTask_Process()` but operates independently. `mission_idx`/`station_idx` are globals (`main.c:94-95`, both start at 0). Lateral alignment uses waist servo rotation + body forward/back; AI classification filters by `mission_class[mission_idx]`.

**class_id → color mapping** (OpenMV model labels → vision_task color IDs):

| class_id | Label | Color ID | OpenMV Mode |
|:--:|------|:--:|------|
| 0 | red_hexagon | 1 (Red) | COLOR |
| 1 | green_circle | 2 (Green) | COLOR |
| 2 | yellow_rect | 3 (Yellow) | COLOR |

---

## 8. TaskQueue System (Mission Sequencer)

`User/state_machine.c/h` defines a **TaskQueue** — a scriptable multi-step mission sequencer (separate from the legacy `TaskNav` and the AI auto-scan pipeline). Up to 16 steps (`TASK_QUEUE_MAX`), each step is a `TaskStep`:

```c
typedef struct {
    uint8_t  travel_mode;    // 0=encoder mileage, 1=line-follow
    int32_t  encoder_dist;   // target encoder counts for this leg
    int16_t  tag_id;         // AprilTag ID for docking (-1 = skip docking)
    int16_t  dock_distance;  // visual servo stop distance (cm)
    uint8_t  action;         // ACTION_GRASP / ACTION_PLACE / ACTION_HOOK / ACTION_UNHOOK / ACTION_NONE
    int16_t  action_param;   // e.g. place waist angle
} TaskStep;
```

TaskQueue state machine: `TQ_TRAVEL → TQ_DOCK → TQ_ACTION → (next step) → TQ_DONE`. During TRAVEL, two triggers can advance to DOCK: (1) encoder mileage reached, or (2) target AprilTag spotted within 100ms (auto-transition to visual servo docking). Per-step timeouts: TRAVEL=30s (3000 ticks), DOCK=10s (1000 ticks), ACTION=8s (800 ticks). Hook/unhook actions (`ACTION_HOOK`/`ACTION_UNHOOK`) call `Action_HookTrailer()`/`Action_UnhookTrailer()` — placeholders for trailer coupling control.

`TaskQueue_Process()` is called from `main()` in the main loop. It is NOT ISR-safe (blocking arm actions use `Delay_ms`).

---

## 9. Debug Interface

Serial commands via **UART5** (wired USB-UART, PC12/PD2) and **Bluetooth USART2** (PD5/PD6, 9600 baud). Both feed into `Commend_Parse()`. `fputc` redirects `printf` to either UART5 or BT based on `uart_set_bt_output()`. Format: `<command> <args>`. Full list via `help` command. Key categories:

- **Motion**: `spd`, `m f|b|l|r|s`, `mode <0-3>`, `stop`, `reset`, `enable <0|1>`, `line_time <sec>`
- **PID tuning**: `kp/ki/kd`, `pos_kp/pos_ki/pos_kd`, `lkp/lki/lkd`, `base_speed`, `PID`, `SET KP:...`, `SETPOINT:`
- **Encoder**: `enc_pos`, `enc_speed`, `enc_max`, `enc_dir`, `enc_clear`, `enc_raw`, `enc_speed_all`, `pid_dbg`
- **Servo**: `servo`, `s_smooth`, `servo_on/off`, `a_set`, `t_set`, `show_act`
- **Arm**: `grasp`, `place <waist>`, `reset_arm`, `show`
- **OpenMV**: `vmode`, `vcolor`, `vtag`, `vqr`, `vfind`, `vqrgo`, `vfind_ai`, `vstop`, `vdata`, `vqrtext`, `vcls`, `vstat`
- **Task**: `task_start`, `task_stop`, `task_status`
- **System**: `save/load` (placeholder), `BT`

---

## 10. CSV Data Protocol & LLM PID Tuner

The main loop sends PID telemetry over UART5 every 50ms (5 `g_sys_tick` units) via `sendPIDDataToUART()` in `User/command.c`. **Currently `sendPIDDataToUART()` is commented out at `main.c:1125` — uncomment it to enable the CSV stream for the LLM PID Tuner.** Each print produces 4 lines (one per motor):

```
<motor_id>,<tick>,<setpoint>,<speed>,<output>,<error>,<Kp>,<Ki>,<Kd>\r\n
```

Note the order: `motor_id` (int) comes first, then `tick` (uint32_t, formatted as `%lu`).

**`llm-pid-tuner-dev/`** is a companion Python project that reads this CSV stream over serial and uses an LLM (OpenAI/Claude/Ollama/etc.) to auto-tune motor PID gains. See its `README.md` for setup. Key files:
- `tuner.py` — real-hardware tuning entry point
- `simulator.py` — local thermal system simulation (no hardware needed)
- `pid_safety.py` — parameter guardrails, rollback to best-known-stable params
- `doctor.py` — environment diagnostics (API connectivity, serial port check)
- `firmware.cpp` — reference Arduino/ESP32 firmware implementing the same CSV protocol

The tuner sends PID updates as serial text commands the existing command parser already handles: `kp/ki/kd <id> <val>` and bulk `SET KP:<p> KI:<i> KD:<d> [M:<id>]`.

---

## 11. Key Architecture Details

### 11.1 Global Tick
`volatile uint32_t g_sys_tick` increments at 100Hz in the TIM6 ISR (`User/Timer.c`). Used by `main()` for periodic tasks (LED heartbeat every ~500k loop iterations, CSV reporting every 5 ticks).

### 11.2 Volatile Shared Variables (ISR ↔ main loop)

These are read/written by both ISR and main-loop code — never assume they stay constant across multiple main-loop statements.

| Variable | Location | Writer | Readers |
|----------|----------|--------|---------|
| `g_sys_tick` | `Timer.c` | TIM6 ISR | main loop, line_follow, vision_task |
| `target_speed[4]` | `main.c` | TIM6 ISR (speed PID output), main loop (manual/vision) | TIM6 ISR (PWM output) |
| `work_mode` | `main.c` | main loop (serial cmd) | TIM6 ISR (dispatch) |
| `motor_enable` | `main.c` | main loop, PVD ISR | TIM6 ISR (gates PWM output) |
| `line_time_ticks` | `Timer.c` | main loop (`line_time` cmd) | TIM6 ISR (line-follow stop check) |
| `g_sys_ms` | `DELAY.c` | SysTick ISR (1ms) | `Delay_ms()`, `Delay_Start()` |
| `rx_buf[]/rx_head/rx_tail` | `uart.c` | UART5 ISR (write head) | main loop (read tail) |
| `omv_rx_buf[]/head/tail` | `commend_openmv.c` | USART3 ISR (write head) | main loop (`OpenMV_PollLine()`) |
| `omv_qr_fresh/buf` | `commend_openmv.c` | USART3 ISR | main loop (AI pipeline) |
| `omv_cls_fresh/data` | `commend_openmv.c` | USART3 ISR | main loop (AI pipeline) |
| `omv_ai_ready` | `commend_openmv.c` | USART3 ISR (`$OK,BOOT`) | main loop (AI init wait) |
| `omv_model_error` | `commend_openmv.c` | USART3 ISR (`$ERR`) | main loop (AI error check) |
| `omv_last_hb_tick` | `commend_openmv.c` | USART3 ISR (`$HB`) | main loop (`OpenMV_IsAlive()`) |

**Rule**: In main-loop code, take a local snapshot of any ISR-shared variable you check more than once (e.g. `uint32_t tick = g_sys_tick;` then use `tick`). The compiler may re-read the `volatile` from memory each time, giving inconsistent values.

### 11.3 PID Features
Both position and incremental PID include integral separation (integral zeroed when |error| > threshold), back-calculation anti-windup (stop integrating when output saturated in same direction as error), trapezoidal integration, and a first-order IIR low-pass filter on the derivative term. Default speed PID gains (incremental): motors 0,1 → Kp=5.5 Ki=0.16 Kd=0; motors 2,3 → Kp=5.0 Ki=0.14 Kd=0. Default position PID: Kp=0.5 Ki=0.001 Kd=0. Default line-follow: base_speed=180, Kp=180.0 Ki=2.0 Kd=2.0.

### 11.4 Motor & Encoder Polarity
`motor_dir[] = {1, -1, 1, -1}` and `encoder_dir[] = {-1, -1, -1, -1}` (all-negative default) correct for physical mounting orientation. Sensor data is multiplied by `encoder_dir[i]` before speed PID; speed commands are multiplied by `motor_dir[i]` before PWM output. A positive `target_speed` always means "forward" at the application level.

### 11.5 Hardware Limitations
Motor 3 (右后) encoder is physically broken — `encoder_max[3] = 0` in main.c, which triggers open-loop fallback in the TIM6 ISR (speed command mapped directly to PWM without PID feedback). The OLED display is physically disconnected — `OLED_Init()` is commented out to avoid I2C timeout stalling startup. Navigation encoder averaging in `TaskNav_Process()` and `TaskQueue_Process()` uses only motors 0–2, excluding motor 3.

### 11.6 Smooth Servo
(`User/smooth_servo.c`): Uses quintic (5th-order) polynomial smoothstep interpolation for jerk-limited servo motion. Called from TIM6 ISR at 100Hz. Includes automatic speed limiting — if the requested motion exceeds `MAX_ANGULAR_VEL` (180°/s peak), the duration is automatically extended.

### 11.7 Servo Power Management
(`smooth_servo.c:168-177`): After reaching target position, each servo has custom power-off behavior (reduces heat and power draw): (1) **Shoulder (ID 1) + Elbow (ID 2)** — always stays powered on (holds against gravity); (2) **Waist (ID 0)** — powered off permanently after reaching position; (3) **Gripper (ID 3)** — cycle mode (hold 500ms → off 5s → re-power 500ms → repeat). If modifying servo behavior, check the `HOLD_TICKS`/`OFF_TICKS`/`ON_TICKS` timing constants.

### 11.8 Arm State Machine
(`User/state_machine.h`): Tracks `ArmState` enum (IDLE/GRASPING/PLACING/RESETTING/ESTOP). Action sequences in `action_group.c` check state before executing and reject overlapping commands with "Arm busy". Features 50ms debounce on state transitions (DEBOUNCE_TICKS=5, checked at 100Hz in `ArmSM_Tick10ms()`) and per-operation timeouts via `ArmSM_Tick10ms()` (30s grasp, 30s place, 30s reset → auto-ESTOP). Overlapping requests return 0 (rejected); callers should check `ArmSM_IsBusy()` before requesting.

### 11.9 Legacy AGV Task Navigation
(`User/state_machine.c`): The `TaskNav` state machine executes full autonomous missions: `TASK_IDLE → TASK_GO_TO_A (line-follow to pickup) → TASK_GRASP (arm grasp) → TASK_GO_TO_B (line-follow to delivery) → TASK_PLACE (arm place) → TASK_DONE`. Progress is measured via encoder averaging (motors 0–2 only; motor 3 encoder is broken). `TaskNav_Process()` is called from main loop at ~100Hz but throttles distance checks to 50ms intervals. Arm actions within it are blocking (use `Delay_ms`), which is safe in the main loop but NOT in ISR context. Each travel leg has a 30s timeout.

### 11.10 PVD Brownout Protection
(`User/Timer.c:PVD_Init`): Configures PVD interrupt at 2.9V threshold (PWR_PVDLevel_7). When VDD drops below 2.9V, `PVD_IRQHandler` immediately brakes all motors and sets `motor_enable = 0`. Priority is NVIC priority 0,0 — the highest in the system.

### 11.11 Encoder Safety
(`User/enconder.c`): IIR low-pass filter (70% prev + 30% new) on speed. Fault detection: delta >10× max → 10 consecutive = force-resync. 16-bit counter wraparound handled.

### 11.12 JTAG Release
`Encoder_Init()` remaps PA15/PB3 from JTAG to GPIO_AF_TIM2, keeping only SWD (PA13/PA14). `DBGMCU->CR` trace pins (PE2–PE6) disabled in `main()`.

### 11.13 Stack/Heap Constraints
(from `startup_stm32f40xx.s`): Stack size = **1024 bytes** (`0x400`), Heap size = **512 bytes** (`0x200`). These are very small — avoid deep call stacks, large local arrays, and dynamic allocation (`malloc`/`new`). A stack overflow will silently corrupt `.data` or heap. No MPU is configured to catch overflows. If adding recursive functions, deeply nested calls, or large local buffers (>256 bytes), increase the stack size in the startup file.

### 11.14 Ring Buffer Pattern
Both UART5 (debug) and USART3 (OpenMV) use the same ring-buffer ISR + main-loop polling pattern. ISR pushes bytes into a circular buffer; the main loop drains them with `uart_rb_getchar()` (UART5) or `OpenMV_PollLine()` (OpenMV). During `printf()` bursts, ISR silently captures bytes to prevent data loss. Never poll RXNE directly from the main loop — it races with the ISR.

### 11.15 `__weak` Parameter Hooks
`Save_parameters()` and `Load_parameters()` are declared `__weak` in `command.c`. They're currently empty placeholders but can be overridden in another translation unit to implement persistent PID parameter storage (e.g., in Flash or EEPROM). The `save`/`load` serial commands call these.

### 11.16 Mode Define Duplication
`MODE_MANUAL`/`MODE_LINE_FOLLOW`/`MODE_POSITION` are in both `line_follow.h` and `main.c`; `MODE_VISUAL_SERVO` only in `visual_servo.h`. Check both locations when modifying mode constants.

---

## 12. Startup Sequence

(`User/main.c`):
1. `Motor_EmergencyBrake()` — immediately grounds all motor IN pins before any slow peripheral init (prevents floating-IN startup runaway)
2. SystemInit → SysTick (1ms) → NVIC config → disable DBGMCU trace pins (PE2–PE6)
3. LED_Init → UART5 init → print "a\r\n" (power-on alive signal) → OpenMV USART3 init (Bluetooth `BT_Init()` is commented out — USART2 is currently unused)
4. Motor + PWM init → Delay_ms(200) → Servo + SmoothServo + Arm state machine + Action param init + VisionTask init
5. VisualServo init (any tag, 15cm stop distance, default PID) + TaskQueue init
6. Encoder + line sensor init
7. PID + line-follow init with default gains
8. TIM6 100Hz ISR start + PVD enable
9. All PID states zeroed, `motor_enable = 0`, target_speed[] cleared, motors stopped
10. `motor_enable = 1; work_mode = MODE_LINE_FOLLOW` — auto-enter line-following (上电直接进循迹)
11. `Action_Reset()` → wait for idle → LED blinks 3 times (MCU-alive heartbeat)
12. Enter `while(1)` — serial command processing + AI auto-scan pipeline + periodic tasks

Note: The **timed auto-mission** (grasp at 5s, place at ~22s) is **disabled** via `#if 0` at `main.c:228`. It has been replaced by the AI auto-scan pipeline and the TaskQueue system.

Note: **开跑整流程 (auto-start run flow)** — boot sets `cross_dock_enable=1` (crossroad auto-dock ON) and prints the `AGV RUN FLOW READY` banner; the robot then runs: line-follow → 十字路口(5路全踩) → 右拐脱线对准 → AprilTag 倒车对接挂钩 → Station0(抓3件放从车) → Station1(从车取件交付) → LED 三闪完成. Serial `run`/`restart` performs a full-flow reset (AI pipeline `qr_phase`/mission/station counters + dock state machine + `dock_done_once`), returning to line-following.

Note: `MotorSelfTest()` (sequential ±400 pulse per motor) is defined as a static function in `main.c` but is **not called** in the current startup sequence — it exists for manual debugging if needed.

---

## 13. Main Loop Detail

(`User/main.c`): After init, the robot is already in `MODE_LINE_FOLLOW` — line-following runs autonomously in the TIM6 ISR. The `while(1)` loop handles:
1. UART5 serial command input (ring-buffer drain only — **no direct RXNE polling**; ISR handles all RX to avoid hardware register races)
   - **Startup silent period**: For the first 300 ticks (3s) after entering the main loop, incoming UART chars are parsed silently — no echo, but commands still execute. After 3s, printable ASCII chars are echoed back. This suppresses USB-TTL noise during power-up.
   - **Error filtering**: UART bytes with Frame Error (FE) or Noise Error (NE) flags are silently discarded (by ISR) — prevents garbage from polluting the command buffer during power transients.
   - Ring buffer ISR captures all bytes; main loop drains them via `uart_rb_getchar()`. Direct RXNE polling from main loop would race with the ISR and is explicitly avoided (see `main.c:222-223`).
2. AI pipeline (`qr_phase` 0–11, QR-triggered; gated on `dock_state==5`) + dock 状态机 (`dock_state` 0–7, 从车对接)
3. LED heartbeat (~500k loop iterations per toggle)
4. CSV PID telemetry placeholder (currently commented out)
5. `OpenMV_PollLine()` for vision data ring-buffer drain
6. `TaskNav_Process()` (legacy AGV navigation)
7. `VisionTask_Process()` (color/QR/AI mission orchestrator)
8. `TaskQueue_Process()` (scriptable multi-step mission sequencer)
9. Bluetooth command input via `BT_GetLine()`

UART bytes arriving during `printf()` bursts are captured by ISR into a ring buffer and replayed when the main loop drains it — preventing data loss during CSV telemetry output.

---

## 14. 从车拖挂 + 视觉伺服对接

从车是纯机械拖斗（无电无MCU），前面贴 AprilTag 供对接。设计决策：箱子抓取用巡线+腰座比例转向（±5cm）；从车对接用视觉伺服双PID（需±1cm硬连接精度）。`vision_task.c` 管箱子抓取，`visual_servo.c` 管对接 PID，`main.c` 内联 `dock_state` 0–7 状态机编排整条对接链（十字路口→右转→找码→倒车→挂钩→回轨）。对接流程：巡线到十字路口（5路全踩）→原地右转90°脱线→找 AprilTag ID=1→`VisualServo_TaskReverse` 双PID精确倒车（横向差速转向+纵向线性减速，右后/右前开环补偿）→到位停车挂钩→回轨切回 `MODE_LINE_FOLLOW`。丢码 1s（`miss>100`）刹车回找码；找码 30s 总超时放弃对接恢复巡线。调参见 `READY.md`「从车对接实机验证 (2026-08-24)」。

---

## 15. Bug Tracking (Bug#1–#7)

Active design issues/hazards tagged as `Bug#N` in comments throughout the codebase. These are known limitations, not undiscovered bugs:

| Bug | Description | Location |
|:---|:---|:---|
| **Bug#1** | Phase 2 hard timeout uses a separate timer not reset by frame refreshes | `main.c:287,421,584` |
| **Bug#2** | Cross-task scan state (`nodata_start`, `scan_step`, `scan_tick`) must be manually reset when transitioning tasks | `main.c:288,414,597` |
| **Bug#3** | ArmSM return value must be checked before calling `Action_Grasp()`/`Action_Place()` | `main.c:623`, `state_machine.c:271,322` |
| **Bug#4** | Phase 1 waist servo scans ±20° around center after 2s to increase detection range | `main.c:293,373` |
| **Bug#5** | OpenMV `$ERR` packets are parsed but main-loop error handling is not fully wired | `commend_openmv.c:29,264` |
| **Bug#6** | 50ms delay after OpenMV mode switch prevents stale first-frame data | `main.c:408` |
| **Bug#7** | UART ring-buffer peeking for "stop" during blocking arm actions enables software emergency stop | `action_group.c:63,67` |

When editing code near any Bug# tag, read the surrounding comments for context.

---

## 16. State Machines Summary (7 total)

| State Machine | Location | Context | Trigger | Purpose |
|:---|:---|:---|:---|:---|
| `ArmSM` | `state_machine.c` | TIM6 ISR (10ms tick) | Serial cmd / TaskNav / TaskQueue / QR pipeline | Arm busy/lockout + timeout guard |
| `TaskNav` | `state_machine.c` | main loop (50ms throttle) | `task_start` serial cmd | Legacy 2-leg go-grasp-go-place mission |
| `TaskQueue` | `state_machine.c` | main loop | API call (`TaskQueue_AddStep()` + `TaskQueue_Start()`) | Scriptable N-step mission with visual docking |
| `VisionTask` | `vision_task.c` | main loop (50ms throttle) | `vfind`/`vqrgo`/`vfind_ai` cmd | Color/QR/AI search-approach-execute |
| AI pipeline | `main.c` (inline, `qr_phase` 0–11) | main loop | QR 触发 + `dock_state==5` | QR→AI/color align→grasp/place (Station0/1) |
| dock 状态机 | `main.c` (inline, `dock_state` 0–7) | main loop | 十字路口 5路全踩 / `crossdock` 命令 | 从车对接: 停车→原地右转→找 AprilTag→倒车→挂钩 |
| `VisualServo` | `visual_servo.c` | TIM6 ISR (100Hz) | `work_mode == 3` | Low-level dual-PID docking control |

**Rule**: Any code that calls `Action_Grasp()`/`Action_Place()` must first check `ArmSM_IsBusy()` or use `ArmSM_RequestGrasp()`/`ArmSM_RequestPlace()` which return 0 if the arm is busy. Call `ArmSM_NotifyComplete()` after the action finishes.

---

## 17. Key Tunable Constants

| Constant | File | Default | Purpose |
|----------|------|---------|---------|
| `MAX_ANGULAR_VEL` | `smooth_servo.c` | 180°/s | Servo speed cap |
| `RX_BUF_SIZE` | `uart.c` | 512 | Debug UART ring buffer |
| `OMV_BUF_SIZE` | `commend_openmv.h` | 256 | OpenMV ring buffer |
| `TASK_QUEUE_MAX` | `state_machine.h` | 16 | Max mission steps |
| `TIMEOUT_GRASP/PLACE/RESET` | `state_machine.c` | 3000 ticks (30s) | Arm action timeout |
| `DEBOUNCE_TICKS` | `state_machine.c` | 5 (50ms) | ArmSM state debounce |
| `DEFAULT_APPROACH_DIST` | `vision_task.c` | 15cm | Vision approach trigger |
| `DEFAULT_GRASP_DIST` | `vision_task.c` | 8cm | Vision grasp trigger |
| `DEFAULT_LOST_TIMEOUT` | `vision_task.c` | 100 ticks (1s) | Target loss timeout |
| `APPROACH_BASE_SPEED` | `vision_task.c` | 100 | Vision approach base speed |
| `WAIST_ADJUST_GAIN` | `vision_task.c` | 0.12°/px | Waist steering gain |
| `APPROACH_SPEED_SLOW` | `vision_task.c` | 60 | AI approach slow speed |
| `PID_DEFAULT_DT` | `pid.c` | 0.01 (10ms) | PID sample time |

---

## 18. Karpathy 编码指南

行为准则：编码前思考（明确假设、呈现权衡）、简洁优先（最少代码、不过度推测）、精准修改（只碰必须碰的、匹配现有风格）、目标驱动执行（定义成功标准、循环验证）。详见用户记忆。
