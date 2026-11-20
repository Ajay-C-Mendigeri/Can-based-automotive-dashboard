# CAN Protocol Based Node Communication for Automotive

A three-node automotive network built on **PIC18F4580** microcontrollers. Two sensor ECUs read vehicle signals and broadcast them over a **CAN bus**; a third ECU acts as the **instrument cluster**, receiving every message and showing it on a character LCD with blinking turn indicators.

> Firmware: Embedded C · Toolchain: MPLAB X + XC8 · MCU: PIC18F4580 (built-in ECAN module)

---

## Table of Contents
1. [System Overview](#1-system-overview)
2. [CAN Protocol Basics](#2-can-protocol-basics)
3. [Standard Data Frame (108 bits)](#3-standard-data-frame-108-bits)
4. [Message Map Used in This Project](#4-message-map-used-in-this-project)
5. [ECU1 – Speed and Gear Node](#5-ecu1--speed-and-gear-node)
6. [ECU2 – Engine and Indicator Node](#6-ecu2--engine-and-indicator-node)
7. [ECU3 – Dashboard / Cluster Node](#7-ecu3--dashboard--cluster-node)
8. [CAN Driver Walkthrough](#8-can-driver-walkthrough)


---

## 1. System Overview

```
 ┌──────────────────────┐        ┌──────────────────────┐        ┌──────────────────────┐
 │        ECU1          │        │        ECU2          │        │        ECU3          │
 │  Speed  + Gear       │        │  RPM + Temp +        │        │  Dashboard (CLCD)    │
 │  (ADC + keypad)      │        │  Indicator           │        │  + indicator LEDs    │
 │  TX: 0x10, 0x20      │        │  TX: 0x30,0x40,0x50  │        │  RX: all IDs         │
 └──────────┬───────────┘        └──────────┬───────────┘        └──────────┬───────────┘
            │ CAN_H / CAN_L                 │                               │
 ═══════════╧═══════════════════════════════╧═══════════════════════════════╧═══════════
                                   CAN BUS (twisted pair, 120 Ω at both ends)
```

| Node | Role | Inputs | Outputs | Sends | Receives |
|------|------|--------|---------|-------|----------|
| **ECU1** | Speed and gear sensor | Potentiometer (ADC ch. 4), 4-key keypad | UART debug log | `0x10`, `0x20` | – |
| **ECU2** | Engine and indicator sensor | Potentiometers (ADC ch. 4 and 6), 4-key keypad | Indicator LEDs, UART debug log | `0x30`, `0x40`, `0x50` | – |
| **ECU3** | Instrument cluster | CAN bus | 16x2 CLCD, indicator LEDs | – | `0x10`–`0x50` |

---

## 2. CAN Protocol Basics

**Controller Area Network (CAN)** is a serial, multi-master, message-based bus designed by Bosch for vehicles. It is used because it is robust against electrical noise, needs only two wires, and lets many ECUs share one bus without a central master.

Key properties:

- **Message-based, not address-based.** Frames carry a *message ID* describing the content (for example "speed"), not the address of a destination node. Every node sees every frame and decides whether to use it.
- **Differential signalling** on CAN_H / CAN_L. A *dominant* bit is logical `0`, a *recessive* bit is logical `1`. A dominant bit always overrides a recessive one on the bus.
- **Arbitration by ID.** If two nodes start transmitting together, they compare the ID bit by bit. The node sending a recessive bit while it reads a dominant one loses and backs off, with no data lost. **The lower ID wins**, so lower IDs mean higher priority.
- **Built-in error handling:** CRC, bit stuffing, ACK check, form check and automatic retransmission.
- **Bit stuffing:** after five consecutive identical bits the transmitter inserts one opposite bit to keep the receivers synchronised. This is why real frames on the wire are slightly longer than the nominal length.

---

## 3. Standard Data Frame (108 bits)

A standard (11-bit identifier) data frame is at most **108 bits** long (1 + 12 + 6 + 64 + 16 + 2 + 7).

```
┌─────┬───────────────────┬───────────────┬──────────────┬───────────┬───────────┬─────────┐
│ SOF │ Arbitration field │ Control field │  Data field  │ CRC field │ ACK field │   EOF   │
│1 bit│      12 bits      │    6 bits     │ 0 – 64 bits  │  16 bits  │  2 bits   │ 7 bits  │
└─────┴───────────────────┴───────────────┴──────────────┴───────────┴───────────┴─────────┘
```

| # | Field | Size | Description |
|---|-------|------|-------------|
| 1 | **SOF** (Start of Frame) | 1 bit | A single dominant bit (`0`). Marks the start of a frame and synchronises all nodes. |
| 2 | **Arbitration field** | 12 bits | **Message ID** (11 bits) + **RTR** (1 bit). Decides which node wins the bus. |
| | Message ID | 11 bits | Identifies the content and sets the priority. Lower value means higher priority. |
| | RTR (Remote Transmission Request) | 1 bit | `0` = data frame (dominant), `1` = remote request frame (recessive). |
| 3 | **Control field** | 6 bits | **IDE** + **reserved bit** + **DLC**. |
| | IDE (Identifier Extension) | 1 bit | `0` = standard 11-bit ID, `1` = extended 29-bit ID. |
| | Reserved bit | 1 bit | Reserved for future use. Sent dominant. |
| | DLC (Data Length Code) | 4 bits | Number of data bytes: `0000` = 0 bytes, `0001` = 1 byte, … `1000` = 8 bytes. |
| 4 | **Data field** | 0 – 64 bits | The payload, 0 to 8 bytes. |
| 5 | **CRC field** | 16 bits | **CRC value** (15 bits) + **CRC delimiter** (1 recessive bit). The receiver recomputes the CRC to validate the frame. |
| 6 | **ACK field** | 2 bits | **ACK slot** (1 bit) + **ACK delimiter** (1 bit). The transmitter sends the slot recessive. Any node that received the frame correctly overwrites it with a dominant bit. |
| 7 | **EOF** (End of Frame) | 7 bits | Seven recessive bits (`1111111`). Marks the end of the frame. |

A 3-bit *Inter-Frame Space* follows every frame before the next one can start.



### How the frames in this project look

All frames here use a **standard ID, data frame (RTR = 0, IDE = 0)**:

| Payload | Bits before stuffing |
|---------|----------------------|
| 1 data byte (gear, indicator) | 1 + 12 + 6 + 8 + 16 + 2 + 7 = **52 bits** |
| 2 data bytes (speed, RPM, temperature) | 1 + 12 + 6 + 16 + 16 + 2 + 7 = **60 bits** |

---

## 4. Message Map Used in This Project

Defined in `msg_id.h`, identical in all three ECUs:

| Message | CAN ID | DLC | Payload | Source | Range |
|---------|--------|-----|---------|--------|-------|
| Speed | `0x10` | 2 | `[0]` = low byte, `[1]` = high byte | ECU1 | 0 – 100 |
| Gear | `0x20` | 1 | ASCII: `'N'`, `'1'`–`'5'`, `'R'` | ECU1 | – |
| RPM | `0x30` | 2 | little-endian 16-bit | ECU2 | 0 – 2000 |
| Engine temperature | `0x40` | 2 | little-endian 16-bit | ECU2 | 0 – 500 (°C) |
| Indicator | `0x50` | 1 | `0` off, `1` left, `2` right, `3` hazard | ECU2 | – |

Because lower IDs win arbitration, **speed is the highest-priority message** and indicator the lowest. Multi-byte values are sent **little-endian** (low byte first).

---

## 5. ECU1 – Speed and Gear Node

**Files:** `ecu1_main.c`, `ecu1_sensor.c/.h`, `adc.c/.h`, `digital_keypad.c/.h`, `can.c/.h`, `uart.c/.h`, `msg_id.h`

ECU1 measures the vehicle speed and tracks the selected gear, then publishes both on the CAN bus.

**Speed**
- `get_speed()` reads **ADC channel 4** (10-bit, 0–1023) from a potentiometer and scales it to **0–100**:
  `speed = adc × 100 / 1023`.
- The value is split into two bytes and transmitted with ID `0x10`, DLC 2.

**Gear**
- `get_gear_pos()` is a small state machine driven by the keypad (PORTC, edge-detected so one press equals one event):

| Key | Action |
|-----|--------|
| SWITCH1 | Gear up: `N → 1 → 2 → 3 → 4 → 5` |
| SWITCH2 | Gear down: `5 → … → 1 → N` |
| SWITCH3 | Reverse (`R`) |
| SWITCH4 | Neutral (`N`) |

- The gear character is transmitted with ID `0x20`, DLC 1.

**Debug output:** every loop iteration prints `speed: 042   |   Gear : 3` over UART.

---

## 6. ECU2 – Engine and Indicator Node

**Files:** `ecu2_main.c`, `ecu2_sensor.c/.h`, `adc.c/.h`, `digital_keypad.c/.h`, `can.c/.h`, `uart.c/.h`, `msg_id.h`

ECU2 monitors the engine and owns the turn-indicator switch.

**RPM:** `get_rpm()` reads ADC channel 4 and scales it to **0–2000**. Sent with ID `0x30`, DLC 2.

**Engine temperature:** `get_engine_temp()` reads ADC channel 6 and scales it to **0–500**. Sent with ID `0x40`, DLC 2.

**Indicator:** `process_indicator()` is driven by the keypad:

| Key | Indicator state | Local LEDs (PORTB) |
|-----|-----------------|--------------------|
| SWITCH1 | Left (`1`) | `RB0`, `RB1` on |
| SWITCH2 | Right (`2`) | `RB6`, `RB7` on |
| SWITCH3 | Hazard (`3`) | Both sides on |
| SWITCH4 | Off (`0`) | All off |

The state is sent with ID `0x50`, DLC 1. ECU2 only reports the *state*. The **blinking** is done by the dashboard (ECU3).

**Debug output:** `rpm : 1234   |   indicator : LEFT   |   temperature : 087 C` over UART.

---

## 7. ECU3 – Dashboard / Cluster Node

**Files:** `main.c`, `meassge_handler.c`, `message_handler.h`, `clcd.c/.h`, `timer0.c/.h`, `can.c/.h`, `uart.c/.h`, `msg_id.h`

ECU3 is a **receive-only** node. It never transmits. It acts as the instrument cluster.

**Main loop**
```c
init_config();                 // CLCD, LEDs, CAN
while (1) {
    process_canbus_data();     // poll CAN RX buffer and dispatch by ID
    blink_indicator_led();     // software blink timing
}
```

**Message dispatch:** `process_canbus_data()` calls `can_receive()`, then switches on the message ID:

| ID | Handler | Shown on the CLCD |
|----|---------|-------------------|
| `0x10` | `handle_speed_data()` | Line 1, col 0: `S:042` |
| `0x20` | `handle_gear_data()` | Line 1, col 6: `G:3` |
| `0x30` | `handle_rpm_data()` | Line 1, col 9: `R:1234` |
| `0x40` | `handle_engine_temp_data()` | Line 2, col 0: `T:087` |
| `0x50` | `handle_indicator_data()` | Line 2, col 6: `I: L` / `I: R` / `I: B` / `I: O` |

Each handler checks that the DLC matches what it expects, rebuilds the 16-bit value from the two bytes, and **only redraws the LCD when the value changes**, which avoids flicker and saves LCD bus time.

**Indicator blinking:** `blink_indicator_led()` toggles a flag every 250 loop passes and drives the LEDs:
- Left: left LEDs blink
- Right: right LEDs blink
- Hazard: both blink together
- Off: all LEDs off

Example LCD layout (16x2):
```
S:042 G:3 R:1234
T:087  I: L
```

---

## 8. CAN Driver Walkthrough

All three ECUs share essentially the same `can.c`.

**Pins:** `RB2` = CAN TX, `RB3` = CAN RX (connect to a CAN transceiver such as MCP2551/TJA1050).

**Initialisation (`init_can`)**
1. Put the ECAN module in **configuration mode** and wait for `CANSTAT == 0x80`.
2. Select ECAN **Mode 0** (`ECANCON = 0`).
3. Set bit timing with `BRGCON1 = 0xE1`, `BRGCON2 = 0x1B`, `BRGCON3 = 0x03` (13 time quanta per bit; the resulting bit rate depends on the oscillator frequency, and the source comment assumes 8 MHz). **All nodes must use the same timing.**
4. Clear the acceptance filters (`RXFCON0 = 0`).
5. Switch to **normal mode**.
6. Configure receive buffer 0 (`RXB0CON`).

**Transmit (`can_transmit(id, data, len)`)**
1. Reject `len > 8`.
2. Wait until the previous frame has gone out (`TXB0REQ == 0`).
3. Write the 11-bit ID into `TXB0SIDH/SIDL`.
4. Write the DLC and copy the data into `TXB0D0…D7`.
5. Set `TXB0REQ = 1`. The hardware handles arbitration, CRC, bit stuffing and retries.

**Receive (`can_receive(&id, data, &len)`)**
1. If `RXB0FUL` is set, rebuild the ID from `RXB0SIDH/SIDL`, read the DLC and copy the data out.
2. Clear `RXB0FUL` and `RXB0IF` so the buffer can accept the next frame.
3. If nothing is pending, `len = 0`. **Callers must check `len` first.**

The CRC, ACK, stuffing and EOF fields from section 3 are all generated and checked **by the CAN peripheral in hardware**. The firmware only supplies the ID, DLC and data.

---