# ESP32+PS5 Controller Quadcopter 

##  Overview
This project implements a dual-loop PID-based flight controller for a quadcopter using an **ESP32** microcontroller and an **MPU-6050** IMU sensor.  
A **PS5 DualSense controller** is connected via Bluetooth to provide pilot input (throttle, roll, pitch, yaw, and arming).

The system uses both **Angle** and **Rate PID loops** for stable and responsive flight control through sensor fusion and real-time motor correction.


##  Hardware Requirements
- ESP32 Development Board  
- MPU-6050 IMU Sensor (I2C interface)  
- 4 × Brushless DC Motors (A2212 or similar)  
- 4 × ESCs (Electronic Speed Controllers)  
- 4S Li-Po Battery  
- Propellers (2 CW + 2 CCW)  
- PS5 DualSense Controller (Bluetooth)  
- Optional: LED indicator on GPIO 2  


##  Pin Configuration

| Component      | ESP32 Pin | Description   |
|----------------|------------|----------------|
| Motor 1 (FR)   | 25         | Front Right    |
| Motor 2 (RR)   | 14         | Rear Right     |
| Motor 3 (RL)   | 4          | Rear Left      |
| Motor 4 (FL)   | 13         | Front Left     |
| LED Indicator  | 2          | Connection LED |
| MPU-6050 SDA   | 21         | I2C Data       |
| MPU-6050 SCL   | 22         | I2C Clock      |


##  PS5 Controller Mapping

| Action          | Control           | Description             |
|-----------------|------------------|-------------------------|
| Throttle        | Left Stick (Y)   | Up/Down — motor power   |
| Roll            | Right Stick (X)  | Tilt left/right         |
| Pitch           | Right Stick (Y)  | Tilt forward/backward   |
| Yaw             | L2 / R2          | Rotate left/right       |
| Arm Motors      | Circle (O)       | Enables motor output    |
| Disarm Motors   | Cross (X)        | Cuts motor output       |



##  Control System Description

### 1️Sensor Processing
- Reads accelerometer and gyroscope data from MPU-6050 via I2C.  
- Applies calibration offsets.  
- Uses a complementary filter to combine gyro + accelerometer data for smooth angle estimation.  

### 2️ Dual PID Control Loops
- **Outer Loop (Angle PID):**  
  Converts desired stick angles into desired rotational rates.  
- **Inner Loop (Rate PID):**  
  Converts rate errors into motor speed adjustments.  

### 3️ Motor Mixing Formula

M1 (FR) = Throttle - Roll - Pitch - Yaw
M2 (RR) = Throttle - Roll + Pitch + Yaw
M3 (RL) = Throttle + Roll + Pitch - Yaw
M4 (FL) = Throttle + Roll - Pitch + Yaw


## ESC & Motor Configuration
- PWM frequency: **450 Hz**  
- Minimum throttle (cutoff): **1000 µs**  
- Idle throttle: **1002 µs**  
- Maximum throttle: **1900 µs**  
- ESC initialization delay: **2 seconds**

---

##  Software Setup

### Required Libraries
- `ps5Controller.h`
- `Wire.h`
- `ESP32Servo.h`
- `math.h`

### Steps
1. Open the code in **Arduino IDE 2.3.3**.  
2. Select **ESP32 Dev Module** as the board.  
3. Upload the sketch to the ESP32.  
4. Pair your PS5 controller by updating its MAC address.
5. Power the quadcopter and connect the PS5 controller.  
6. Wait for the LED on pin 2 to light (Bluetooth connected).  


##  Safety Notes
- Always **remove propellers during testing**.  
- Verify all motor rotation directions before flight.  
- Calibrate IMU on a **flat, level surface**.  
- Keep a safety switch accessible during tuning.  
- Use a tether or safety cage during initial testing.  

##  Future Enhancements
- Add barometer for altitude hold  
- Integrate GPS for waypoint navigation  
- Include blackbox logging for PID performance  
- Connect Raspberry Pi for AI vision and autonomous flight  


