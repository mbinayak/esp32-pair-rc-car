# ESP32 RC Car with Joystick Control

A fun personal project designed for my toddler to enjoy a wireless RC car, while I explore embedded systems and motor control using ESP32.

---

## 🔧 Initial Setup & Wiring

* **Microcontroller**: ESP32 (x2)
* **Motor Driver**: L298N (H-Bridge)
* **Battery**: 7.4V Li-ion
* **Motors**: 4 DC motors (tank-style: 2 per side)

**Basic Wiring**:

* L298N `IN1/IN2/IN3/IN4` → ESP32 motor control pins
* L298N `ENA/ENB` (PWM) → ESP32 PWM pins
* L298N `VCC` → Battery 7.4V
* ESP32 powered via L298N 5V out (or separate regulator if needed)

---

## 🎮 WiFi Joystick Controller (Legacy)

Originally, the remote used a physical joystick connected to one ESP32 which sent values via ESP-NOW to the car ESP32. This version is still available under:

```
code/
├── wifi_joystick_controller/       # Sends joystick input
└── wifi_rc_car_controller/         # Receives input and drives motors
```

---

## 🎮 Bluetooth Gamepad Upgrade

To make it more intuitive and fun for my toddler, the ESP32 now connects to a Bluetooth gamepad (e.g., PS/Xbox) using [Bluepad32](https://github.com/deltaco/Bluepad32).

### Controls:

* `throttle` → Move forward
* `brake` → Move backward
* `left X axis` → Turn left (slows left motors)
* `right X axis` → Turn right (slows right motors)

New code for this setup lives in:

```
code/
└── ble-joystic-receiver-rc-car/
    └── ble-joystic-receiver-rc-car.ino
```

---

## 📁 Folder Structure

```
.
├── assets/
│   └── wiring_diagrams/             # Circuit diagrams
├── code/
│   ├── ble-joystic-receiver-rc-car/
│   ├── shared/
│   ├── wifi_joystick_controller/
│   └── wifi_rc_car_controller/
└── README.md
```

---

## ✅ Next Plans

* Add servo-mounted distance sensor for wall detection
* Add LED indicators (front/rear)
* Upgrade motor driver to TB6612FNG or DRV8833

---

Made with love ❤️ for my curious little driver 🏋️‍♂️🚗
