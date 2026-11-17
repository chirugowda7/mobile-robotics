/*
  =============================================================================
  6-Axis Robot Arm Controller Firmware with PID Smooth Motion Engine
  =============================================================================
  Safe Home Position:
    - J1 (Base):         90.0°  (Range: 0° to 180°)
    - J2 (Shoulder):     80.0°  (Range: 0° to 180°)
    - J3 (Elbow):        108.0° (Range: 0° to 180°)
    - J4 (Wrist Pitch):  38.0°  (Range: 0° to 180°)
    - J5 (Wrist Roll):   96.0°  (Range: 0° to 180°)
    - J6 (Gripper):      45.0°  (Range: 40° to 100° strictly locked)

  Features:
    - Active PID trajectory smoothing & critically damped deceleration
    - Velocity and acceleration limiting (protects J2 shoulder and J6 gripper)
    - Sub-degree 2000-step microsecond resolution (zero stepping chatter)
    - Compatible with: axis.py
    - Baud rate: 115200
  =============================================================================
*/

#include <Servo.h>

const int NUM_JOINTS = 6;
const int SERVO_PINS[NUM_JOINTS] = {3, 5, 6, 9, 10, 11};

// Servo objects
Servo joints[NUM_JOINTS];

// Microsecond pulse calibration (500us - 2500us for full range)
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// Individual joint angle limits: J6 is clamped to 40° - 100°
const float JOINT_MIN_DEG[NUM_JOINTS] = {0.0,   0.0,   0.0,   0.0,   0.0,   40.0};
const float JOINT_MAX_DEG[NUM_JOINTS] = {180.0, 180.0, 180.0, 180.0, 180.0, 100.0};

// Safe Home Position per Joint: [J1, J2, J3, J4, J5, J6 (45.0°)]
const float HOME_POS_DEG[NUM_JOINTS] = {90.0f, 80.0f, 108.0f, 38.0f, 96.0f, 45.0f};

// PID Controller Structure per Joint
struct PIDController {
  float target;         // Setpoint from GUI (degrees)
  float current;        // Current filtered output (degrees)
  float velocity;       // Current velocity (deg/sec)
  float integral;       // Error integral
  float Kp;             // Proportional gain (pull towards target)
  float Ki;             // Integral gain (steady state accuracy)
  float Kd;             // Derivative gain (damping / anti-overshoot)
  float baseMaxVel;     // Base maximum velocity (deg/sec)
  float baseMaxAccel;   // Base maximum acceleration (deg/sec^2)
  float maxVel;         // Active maximum velocity (deg/sec)
  float maxAccel;       // Active maximum acceleration (deg/sec^2)
};

PIDController pid[NUM_JOINTS];

// Speed multiplier (0.1 to 1.0)
float speedScale = 0.65f; // 65% default (smooth & gentle)

unsigned long lastPIDUpdate = 0;
bool emergencyStopped = false;

// Serial receive buffer
const int BUF_SIZE = 128;
char rxBuffer[BUF_SIZE];
int rxIndex = 0;

void setSpeedScale(float scale) {
  speedScale = constrain(scale, 0.1f, 1.0f);
  for (int i = 0; i < NUM_JOINTS; i++) {
    pid[i].maxVel = pid[i].baseMaxVel * speedScale;
    pid[i].maxAccel = pid[i].baseMaxAccel * speedScale;
  }
}

void initPID() {
  // Base parameters: {Kp, Ki, Kd, baseMaxVel, baseMaxAccel}
  float params[NUM_JOINTS][5] = {
    // Kp,   Ki,   Kd,   maxVel, maxAccel
    {12.0,  0.2,  6.0,  120.0,  300.0},  // J1: Base Turntable
    { 9.0,  0.3,  6.5,   70.0,  160.0},  // J2: Shoulder (Heavy load - ultra smooth)
    {10.0,  0.2,  5.5,   90.0,  220.0},  // J3: Elbow
    {12.0,  0.2,  5.0,  140.0,  350.0},  // J4: Wrist Pitch
    {12.0,  0.2,  5.0,  140.0,  350.0},  // J5: Wrist Roll
    {14.0,  0.2,  5.0,  120.0,  350.0}   // J6: Gripper (Safe 40°–100°)
  };

  for (int i = 0; i < NUM_JOINTS; i++) {
    pid[i].target       = HOME_POS_DEG[i];  // Start at Safe Home Position
    pid[i].current      = HOME_POS_DEG[i];
    pid[i].velocity     = 0.0f;
    pid[i].integral     = 0.0f;
    pid[i].Kp           = params[i][0];
    pid[i].Ki           = params[i][1];
    pid[i].Kd           = params[i][2];
    pid[i].baseMaxVel   = params[i][3];
    pid[i].baseMaxAccel = params[i][4];
  }
  setSpeedScale(speedScale);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for native USB if using Leonardo/Micro
  }

  initPID();

  // Attach servos with extended pulse width and start at Safe Home Position
  for (int i = 0; i < NUM_JOINTS; i++) {
    joints[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
    float ratio = HOME_POS_DEG[i] / 180.0f;
    int pulseUs = (int)round(SERVO_MIN_US + ratio * (SERVO_MAX_US - SERVO_MIN_US));
    joints[i].writeMicroseconds(pulseUs);
  }

  lastPIDUpdate = millis();
  Serial.println("READY: 6-Axis Arm Connected (J6 Home at 45 deg)");
}

void loop() {
  // 1. Process serial commands
  readSerialData();

  // 2. Continuous 100Hz PID Motion Smoothing Loop
  updatePIDMotion();
}

void readSerialData() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      if (rxIndex > 0) {
        rxBuffer[rxIndex] = '\0';
        handleCommand(rxBuffer);
        rxIndex = 0;
      }
    } else {
      if (rxIndex < BUF_SIZE - 1) {
        rxBuffer[rxIndex++] = c;
      } else {
        rxIndex = 0;
      }
    }
  }
}

void handleCommand(char* cmd) {
  while (*cmd == ' ') cmd++; // Trim leading whitespace

  // 1. HOME COMMAND: Smoothly glides all joints to Safe Home Position
  if (strncmp(cmd, "HOME", 4) == 0) {
    emergencyStopped = false;
    for (int i = 0; i < NUM_JOINTS; i++) {
      pid[i].target = HOME_POS_DEG[i];
    }
    Serial.println("OK: Gliding to Safe Home position.");
    return;
  }

  // 2. SPEED COMMAND: "SPEED <10-100>"
  if (strncmp(cmd, "SPEED", 5) == 0) {
    int spd = atoi(cmd + 5);
    if (spd >= 10 && spd <= 100) {
      setSpeedScale(spd / 100.0f);
      Serial.print("OK: Speed set to ");
      Serial.print(spd);
      Serial.println("%");
    }
    return;
  }

  // 3. EMERGENCY STOP
  if (strncmp(cmd, "STOP", 4) == 0) {
    emergencyStopped = true;
    for (int i = 0; i < NUM_JOINTS; i++) {
      pid[i].target = pid[i].current;
      pid[i].velocity = 0.0f;
      pid[i].integral = 0.0f;
    }
    Serial.println("STOP: Arm halted.");
    return;
  }

  // 4. MOVE COMMAND: "MOVE j1,j2,j3,j4,j5,j6"
  if (strncmp(cmd, "MOVE", 4) == 0) {
    if (emergencyStopped) {
      emergencyStopped = false;
    }

    char* data = cmd + 4;
    while (*data == ' ') data++;

    float parsedAngle;
    int count = 0;

    char* token = strtok(data, ",");
    while (token != NULL && count < NUM_JOINTS) {
      parsedAngle = atof(token);

      // Clamp target within safe mechanical limits (J6 strictly clamped 40°–100°)
      if (parsedAngle < JOINT_MIN_DEG[count]) parsedAngle = JOINT_MIN_DEG[count];
      if (parsedAngle > JOINT_MAX_DEG[count]) parsedAngle = JOINT_MAX_DEG[count];

      // Update PID setpoint
      pid[count].target = parsedAngle;

      count++;
      token = strtok(NULL, ",");
    }

    return;
  }

  Serial.print("ERR: Unknown command: ");
  Serial.println(cmd);
}

// 100 Hz PID Trajectory Smoothing & Deceleration Control
void updatePIDMotion() {
  if (emergencyStopped) return;

  unsigned long now = millis();
  unsigned long elapsedMs = now - lastPIDUpdate;

  // Run at fixed 10 ms interval (100 Hz)
  if (elapsedMs < 10) return;

  float dt = elapsedMs / 1000.0f;
  lastPIDUpdate = now;

  for (int i = 0; i < NUM_JOINTS; i++) {
    float error = pid[i].target - pid[i].current;

    // Deadband threshold: if within 0.1 degree and nearly stopped, freeze
    if (fabs(error) < 0.10f && fabs(pid[i].velocity) < 0.4f) {
      pid[i].velocity = 0.0f;
      pid[i].integral = 0.0f;
      continue;
    }

    // Accumulate integral error with anti-windup clamping
    pid[i].integral += error * dt;
    pid[i].integral = constrain(pid[i].integral, -12.0f, 12.0f);

    // Derivative on measurement: -velocity dampens oscillations cleanly
    float derivative = -pid[i].velocity;

    // Calculate desired acceleration from PID
    float desiredAccel = (pid[i].Kp * error) + (pid[i].Ki * pid[i].integral) + (pid[i].Kd * derivative);

    // Limit acceleration to prevent sudden torque spikes on servos
    desiredAccel = constrain(desiredAccel, -pid[i].maxAccel, pid[i].maxAccel);

    // Update velocity
    pid[i].velocity += desiredAccel * dt;
    pid[i].velocity = constrain(pid[i].velocity, -pid[i].maxVel, pid[i].maxVel);

    // Update position
    pid[i].current += pid[i].velocity * dt;
    pid[i].current = constrain(pid[i].current, JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);

    // Convert angle to microsecond pulse with sub-degree micro-stepping
    float ratio = pid[i].current / 180.0f;
    int pulseUs = (int)round(SERVO_MIN_US + ratio * (SERVO_MAX_US - SERVO_MIN_US));
    joints[i].writeMicroseconds(pulseUs);
  }
}
