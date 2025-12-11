✅ FINAL WIRING TABLE — Opportunity Rover Six-Wheel Drive (BTS7960)

Each wheel uses one BTS7960 board.

Legend

Arduino Mega Pin = digital pin from your microcontroller

BTS7960 Pins = L_EN, R_EN, LPWM, RPWM

PCA9685 Channels = 0–15 (4096-step PWM)

🛞 Wheel #1 — Front Left (FL)
Function	Arduino Mega	PCA9685 Channel	BTS7960 Pin
Enable (L_EN + R_EN tied together)	23	—	L_EN + R_EN
Forward PWM	—	0	RPWM
Reverse PWM	—	1	LPWM
🛞 Wheel #2 — Front Right (FR)
Function	Arduino Mega	PCA9685 Channel	BTS7960 Pin
Enable	25	—	L_EN + R_EN
Forward PWM	—	2	RPWM
Reverse PWM	—	3	LPWM
🛞 Wheel #3 — Center Left (CL)
Function	Arduino Mega	PCA9685 Channel	BTS7960 Pin
Enable	27	—	L_EN + R_EN
Forward PWM	—	4	RPWM
Reverse PWM	—	5	LPWM
🛞 Wheel #4 — Center Right (CR)
Function	Arduino Mega	PCA9685 Channel	BTS7960 Pin
Enable	29	—	L_EN + R_EN
Forward PWM	—	6	RPWM
Reverse PWM	—	7	LPWM
🛞 Wheel #5 — Rear Left (RL)
Function	Arduino Mega	PCA9685 Channel	BTS7960 Pin
Enable	31	—	L_EN + R_EN
Forward PWM	—	8	RPWM
Reverse PWM	—	9	LPWM
🛞 Wheel #6 — Rear Right (RR)
Function	Arduino Mega	PCA9685 Channel	BTS7960 Pin
Enable	33	—	L_EN + R_EN
Forward PWM	—	10	RPWM
Reverse PWM	—	11	LPWM
🎯 PWM FREQUENCY

Set PCA9685 to 1 kHz (pwm.setPWMFreq(1000)) — ideal for BTS7960 and keeps noise low.

⚙️ STEERING SERVOS

You reserved channels 12–15:

Servo	PCA9685 CH
FL steering	12
FR steering	13
RL steering	14
RR steering	15

These remain unaffected by the BTS upgrade.

🔌 ENCODERS — Reminder

Per your firmware:

Wheel	ENC A	ENC B
FL	18	34
FR	19	35
CL	20	36
CR	21	37
RL	2	38
RR	3	39

Correct and fully compatible with the BTS conversion.

🧰 Power Wiring (BE CAREFUL HERE)
BTS7960 Power

+12V to VCC motor terminal

GND to battery negative

Arduino GND must connect to BTS7960 GND

PCA9685 GND must also connect to Arduino GND

This ensures reference unity for PWM.

⚠️ IMPORTANT: If your BTS7960 EN pins are tied to +5V

Some boards ship with L_EN and R_EN pre-tied HIGH.

If EN is tied to 5V:

You do not need the Arduino EN pins defined

You MUST change firmware to ignore EN pins

Replace in firmware:

digitalWrite(MOTOR_EN_PINS[i], HIGH);


with:

// EN pinned to 5V — EN control disabled


Tell me if this is your configuration and I will auto-patch your firmware.

🔧 If your wiring differs, I can auto-regenerate the tables

Tell me:

1. Do you want EN controlled by Arduino?

Yes → Use tables above

No, EN tied to 5V → I will remove EN control logic

2. Do your PCA9685 channels need re-assignment?

I can remap arbitrarily.

3. Do you want automatic generation of:

Updated Arduino firmware

Wiring diagrams (ASCII / PDF)

ROS parameter YAML for motor mapping

Gazebo plugin mapping for each wheel

Jetson/RPi pinout overlay
