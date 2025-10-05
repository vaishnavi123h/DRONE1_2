#include <ps5Controller.h>
#include <Wire.h>
#include <ESP32Servo.h> 
#include <math.h> // Include for atan/sqrt/atan2 if not automatically included

const int ESCfreq = 450;
const int mot1_pin = 25;
const int mot2_pin = 14;
const int mot3_pin = 4;
const int mot4_pin = 13;
const float LOOP_TIME_S = 0.004; // 4ms loop time (250Hz) 
const int THROTTLE_CUTOFF = 1000;
const int THROTTLE_IDLE = 1002;

// Angle PID (Outer Loop: Angle Error -> Desired Rate)
const float PAngleRoll = 2.0; 
const float IAngleRoll = 0.5; 
const float DAngleRoll = 0.007; 
const float PAnglePitch = PAngleRoll;
const float IAnglePitch = IAngleRoll;
const float DAnglePitch = DAngleRoll;
const float MAX_ANGLE_I_TERM = 400.0; // Max I-term output
const float MAX_ANGLE_PID_OUT = 400.0; // Max output for Desired Rate

// Rate PID (Inner Loop: Rate Error -> Motor Input)
const float PRateRoll = 0.625; 
const float IRateRoll = 0.7; 
const float DRateRoll = 0.0005; 
const float PRatePitch = PRateRoll;
const float IRatePitch = IRateRoll;
const float DRatePitch = DRateRoll;

const float PRateYaw = 4.0; 
const float IRateYaw = 3.0; 
const float DRateYaw = 0.0; 
const float MAX_RATE_PID_OUT = 450.0;


// IMU CALIBRATION & FILTER CONSTANTS

float RateCalibrationRoll = -2.82; 
float RateCalibrationPitch = 0.435; 
float RateCalibrationYaw = -0.25;
float AccXCalibration = 0.01; 
float AccYCalibration = -0.01; 
float AccZCalibration = 0.05;

// Complementary Filter Constants
const float COMP_ALPHA = 0.991; // Gyro weight (0.991)
const float ACC_WEIGHT = 1.0 - COMP_ALPHA; // Accelerometer weight (0.009)
const float MAX_COMP_ANGLE = 20.0; // Max angle clamp (for sanity)

//GLOBAL VARIABLES

Servo mot1, mot2, mot3, mot4;
uint32_t LoopTimer;
bool motorsArmed = false;

// Sensor Readings
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch; // Calculated from Accel
volatile float RateRoll, RatePitch, RateYaw; // Calibrated Gyro Rates

// Filtered Angles
volatile float complementaryAngleRoll = 0.0f;
volatile float complementaryAnglePitch = 0.0f;

// RC Input Variables
int RightStickX, LeftStickX, RightStickY, LeftStickY, L2_Value, R2_Value;
float InputThrottle;
volatile float DesiredAngleRoll, DesiredAnglePitch, DesiredRateYaw; 

// PID Error & Term Variables (Roll, Pitch, Yaw are reused)
// Angle Loop
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll = 0.0, PrevItermAngleRoll = 0.0;
volatile float PrevErrorAnglePitch = 0.0, PrevItermAnglePitch = 0.0;
volatile float DesiredRateRoll, DesiredRatePitch; // Output of Angle PID

// Rate Loop
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float PrevErrorRateRoll = 0.0, PrevItermRateRoll = 0.0;
volatile float PrevErrorRatePitch = 0.0, PrevItermRatePitch = 0.0;
volatile float PrevErrorRateYaw = 0.0, PrevItermRateYaw = 0.0;
volatile float PtermRoll, ItermRoll, DtermRoll, InputRoll;
volatile float PtermPitch, ItermPitch, DtermPitch, InputPitch;
volatile float PtermYaw, ItermYaw, DtermYaw, InputYaw;

// Motor Outputs
volatile int MotorInput1, MotorInput2, MotorInput3, MotorInput4;

//GYRO SIGNAL FUNCTION

void gyro_signal() {
    // I2C Communication for MPU-6050
    // Set digital low-pass filter (0x05 for 10Hz/20ms delay) and Accel config (0x10 for +/-8g)
    Wire.beginTransmission(0x68);
    Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();

    // Read Accelerometer (6 bytes from 0x3B)
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();

    // Set Gyro config (0x08 for +/-500 deg/s)
    Wire.beginTransmission(0x68);
    Wire.write(0x1B); Wire.write(0x8); Wire.endTransmission();

    // Read Gyroscope (6 bytes from 0x43)
    Wire.beginTransmission(0x68);
    Wire.write(0x43); Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();

    // Convert to Engineering Units
    // Accel: MPU-6050 with +/-8g scale is 4096 LSB/g
    AccX = (float)AccXLSB / 4096.0;
    AccY = (float)AccYLSB / 4096.0;
    AccZ = (float)AccZLSB / 4096.0;
    // Gyro: MPU-6050 with +/-500 deg/s scale is 65.5 LSB/(deg/s)
    RateRoll = (float)GyroX / 65.5;
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;

    // Apply Calibrations
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;
    AccX -= AccXCalibration;
    AccY -= AccYCalibration;
    AccZ -= AccZCalibration;

    // Calculate Angle from Accelerometer
    AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.295779513; // * (180/PI)
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.295779513;

    // Complementary Filter
    complementaryAngleRoll = COMP_ALPHA * (complementaryAngleRoll + RateRoll * LOOP_TIME_S) + ACC_WEIGHT * AngleRoll;
    complementaryAnglePitch = COMP_ALPHA * (complementaryAnglePitch + RatePitch * LOOP_TIME_S) + ACC_WEIGHT * AnglePitch;
    
    // Clamp filtered angle to prevent integration errors and instability
    complementaryAngleRoll = constrain(complementaryAngleRoll, -MAX_COMP_ANGLE, MAX_COMP_ANGLE);
    complementaryAnglePitch = constrain(complementaryAnglePitch, -MAX_COMP_ANGLE, MAX_COMP_ANGLE);
}

void setup(void) {
    Serial.begin(115200);
    ps5.begin("58:10:31:2f:c8:b5");
    pinMode(2, OUTPUT);

    // MPU-6050 Initialization
    Wire.setClock(400000);
    Wire.begin();
    delay(150);
    Wire.beginTransmission(0x68); 
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(100);

    // ESC Setup
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    mot1.attach(mot1_pin, THROTTLE_CUTOFF, 2000); // min/max pulse width for ESCs
    mot2.attach(mot2_pin, THROTTLE_CUTOFF, 2000);
    mot3.attach(mot3_pin, THROTTLE_CUTOFF, 2000);
    mot4.attach(mot4_pin, THROTTLE_CUTOFF, 2000);

    mot1.setPeriodHertz(ESCfreq); // Set PWM frequency
    mot2.setPeriodHertz(ESCfreq);
    mot3.setPeriodHertz(ESCfreq);
    mot4.setPeriodHertz(ESCfreq);

    mot1.writeMicroseconds(THROTTLE_CUTOFF); // Initial disarmed state
    mot2.writeMicroseconds(THROTTLE_CUTOFF);
    mot3.writeMicroseconds(THROTTLE_CUTOFF);
    mot4.writeMicroseconds(THROTTLE_CUTOFF);

    // Wait for ESCs to arm (usually a few seconds)
    delay(2000);

    LoopTimer = micros();
}

void loop(void) {
    // Controller Input and Failsafe
     if (PS5.isConnected()) {
    digitalWrite(2, HIGH);
    RightStickX = (abs(PS5.RStickX()) < 15) ? 0 : PS5.RStickX();
    RightStickY = (abs(PS5.RStickY()) < 15) ? 0 : PS5.RStickY();
    LeftStickX = PS5.LStickX();
    LeftStickY = PS5.LStickY();
    L2_Value = PS5.L2Value();
    R2_Value = PS5.R2Value();

    if (PS5.Circle()) motorsArmed = true;
    if (PS5.Cross()) motorsArmed = false;
  } else {
    digitalWrite(2, LOW);
    RightStickX = RightStickY = LeftStickX = 0;
    LeftStickY = -127;
  }

    // Read IMU and update filter
    gyro_signal();

    // Roll/Pitch sticks map to Desired Angle (Max 25 degrees)
    DesiredAngleRoll = map(RightStickX, -128, 127, -25, 25); 
    DesiredAnglePitch = map(RightStickY, -128, 127, -25, 25);
    
    // Left stick Y maps to Throttle
    InputThrottle = map(LeftStickY, -128, 127, THROTTLE_CUTOFF, 2000);
    InputThrottle = constrain(InputThrottle, THROTTLE_CUTOFF, 1800);

    // L2/R2 map to Desired Yaw Rate (Max +/- 25 deg/s)
    if ((R2_Value > 0 && L2_Value > 0) || (R2_Value == 0 && L2_Value == 0)) {
        DesiredRateYaw = 0;
    } else {
        if (R2_Value > 0) { DesiredRateYaw = map(R2_Value, 0, 255, 0, 25); } 
        if (L2_Value > 0) { DesiredRateYaw = map(L2_Value, 0, 255, 0, -25); } 
    }


    // 1. Roll Angle PID
    ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
    PtermRoll = PAngleRoll * ErrorAngleRoll;
    // Trapezoidal Integration: Iterm += I * (Error_current + Error_previous) * dt/2
    ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (LOOP_TIME_S / 2.0));
    ItermRoll = constrain(ItermRoll, -MAX_ANGLE_I_TERM, MAX_ANGLE_I_TERM);
    DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / LOOP_TIME_S);
    DesiredRateRoll = PtermRoll + ItermRoll + DtermRoll;
    DesiredRateRoll = constrain(DesiredRateRoll, -MAX_ANGLE_PID_OUT, MAX_ANGLE_PID_OUT);
    PrevErrorAngleRoll = ErrorAngleRoll; // FIX: Store positive error
    PrevItermAngleRoll = ItermRoll;

    // 2. Pitch Angle PID
    ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
    PtermPitch = PAnglePitch * ErrorAnglePitch;
    ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (LOOP_TIME_S / 2.0));
    ItermPitch = constrain(ItermPitch, -MAX_ANGLE_I_TERM, MAX_ANGLE_I_TERM);
    DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / LOOP_TIME_S);
    DesiredRatePitch = PtermPitch + ItermPitch + DtermPitch;
    DesiredRatePitch = constrain(DesiredRatePitch, -MAX_ANGLE_PID_OUT, MAX_ANGLE_PID_OUT);
    PrevErrorAnglePitch = ErrorAnglePitch; // FIX: Store positive error
    PrevItermAnglePitch = ItermPitch;


    //Rate PID (Inner Loop: Rate Error -> Motor Adjustment)

    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw - RateYaw;

    // 1. Roll Rate PID
    PtermRoll = PRateRoll * ErrorRateRoll;
    ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (LOOP_TIME_S / 2.0));
    ItermRoll = constrain(ItermRoll, -MAX_RATE_PID_OUT, MAX_RATE_PID_OUT);
    DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / LOOP_TIME_S);
    InputRoll = PtermRoll + ItermRoll + DtermRoll;
    InputRoll = constrain(InputRoll, -MAX_RATE_PID_OUT, MAX_RATE_PID_OUT);
    PrevErrorRateRoll = ErrorRateRoll; // FIX: Store positive error
    PrevItermRateRoll = ItermRoll;

    // 2. Pitch Rate PID
    PtermPitch = PRatePitch * ErrorRatePitch;
    ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (LOOP_TIME_S / 2.0));
    ItermPitch = constrain(ItermPitch, -MAX_RATE_PID_OUT, MAX_RATE_PID_OUT);
    DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / LOOP_TIME_S);
    InputPitch = PtermPitch + ItermPitch + DtermPitch;
    InputPitch = constrain(InputPitch, -MAX_RATE_PID_OUT, MAX_RATE_PID_OUT);
    PrevErrorRatePitch = ErrorRatePitch; // FIX: Store positive error
    PrevItermRatePitch = ItermPitch;

    // 3. Yaw Rate PID
    PtermYaw = PRateYaw * ErrorRateYaw;
    ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (LOOP_TIME_S / 2.0));
    ItermYaw = constrain(ItermYaw, -MAX_RATE_PID_OUT, MAX_RATE_PID_OUT);
    DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / LOOP_TIME_S);
    InputYaw = PtermYaw + ItermYaw + DtermYaw;
    InputYaw = constrain(InputYaw, -MAX_RATE_PID_OUT, MAX_RATE_PID_OUT);
    PrevErrorRateYaw = ErrorRateYaw; // FIX: Store positive error
    PrevItermRateYaw = ItermYaw;


    // Motor Mixing

    // Motor Mixing: Throttle +/- (Roll Pitch Yaw) adjustments
    // M1 (FR) = THR - Roll - Pitch - Yaw
    // M2 (RR) = THR - Roll + Pitch + Yaw
    // M3 (RL) = THR + Roll + Pitch - Yaw
    // M4 (FL) = THR + Roll - Pitch + Yaw

    MotorInput1 = round(InputThrottle - InputRoll - InputPitch - InputYaw);
    MotorInput2 = round(InputThrottle - InputRoll + InputPitch + InputYaw);
    MotorInput3 = round(InputThrottle + InputRoll + InputPitch - InputYaw);
    MotorInput4 = round(InputThrottle + InputRoll - InputPitch + InputYaw);

    // Output Constraining (Limit max/min)
    MotorInput1 = constrain(MotorInput1, THROTTLE_IDLE, 1900);
    MotorInput2 = constrain(MotorInput2, THROTTLE_IDLE, 1900);
    MotorInput3 = constrain(MotorInput3, THROTTLE_IDLE, 1900);
    MotorInput4 = constrain(MotorInput4, THROTTLE_IDLE, 1900);

    //Motor Arming/Disarming
        if (!motorsArmed) { 
        // Force motors to cut-off and reset I-terms on disarm
        MotorInput1 = THROTTLE_CUTOFF;
        MotorInput2 = THROTTLE_CUTOFF;
        MotorInput3 = THROTTLE_CUTOFF;
        MotorInput4 = THROTTLE_CUTOFF;
        
        PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevItermRateYaw = 0;
        PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
    }
    
    //Output to ESCs and Serial Monitoring 
    mot1.writeMicroseconds(MotorInput1);
    mot2.writeMicroseconds(MotorInput2);
    mot3.writeMicroseconds(MotorInput3);
    mot4.writeMicroseconds(MotorInput4);

    Serial.print("Throttle: "); Serial.print(InputThrottle);
    Serial.print(" | AngleR/P: "); Serial.print(complementaryAngleRoll); Serial.print(" / "); Serial.print(complementaryAnglePitch);
    Serial.print(" | RateR/P/Y: "); Serial.print(RateRoll); Serial.print(" / "); Serial.print(RatePitch); Serial.print(" / "); Serial.print(RateYaw);
    Serial.print(" | Motors: "); Serial.print(MotorInput1); Serial.print(" "); Serial.print(MotorInput2); Serial.print(" "); Serial.print(MotorInput3); Serial.print(" "); Serial.println(MotorInput4);

    // Loop Timing
    while (micros() - LoopTimer < (LOOP_TIME_S * 1000000))
        ; // Wait until 4ms has passed
    LoopTimer = micros();
}