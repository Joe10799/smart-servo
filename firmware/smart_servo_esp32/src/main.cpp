#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>

// -------------------- Configuration --------------------

static constexpr float GEAR_RATIO = 15.0f / 40.0f;
static constexpr float TORQUE_THRESHOLD = 80.0f;

static constexpr int SERVO_PIN = 5;
static constexpr int PROXY_DIR_PIN = 18;
static constexpr int PROXY_STEP_PIN = 19;
static constexpr float PROXY_STEPS_PER_REVOLUTION = 200.0f;

const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";

static constexpr uint16_t TCP_PORT = 3333;

// -------------------- Servo / Door State --------------------

Servo visual_servo;

float servo_angle = 0.0f;      // continuous servo angle, can exceed 360
float door_angle = 0.0f;       // door angle after gearbox
float speed_dps = 40.0f;       // servo speed in deg/sec
int direction = 0;             // -1 left, 0 stop, +1 right

// The standard Wokwi servo is limited to 180 degrees. A stepper is connected
// as a display-only proxy so the unbounded servo_angle is visible through full
// rotations. It is not part of the real door actuator.
long proxy_step_position = 0;

// Hidden randomized real endpoints
float closed_real = -20.0f;
float open_real = 40.0f;

// Detected endpoints
float closed_detected = 0.0f;
float open_detected = 0.0f;

bool calibrated = false;
bool detecting = false;
bool detect_armed = false;
int detect_stage = 0;          // 0 idle, 1 finding closed, 2 finding open

unsigned long last_ms = 0;
unsigned long last_print = 0;

// -------------------- TCP --------------------

WiFiServer server(TCP_PORT);
WiFiClient client;

// -------------------- Helpers --------------------

float wrapAngle180(float angle) {
  float wrapped = fmod(angle, 180.0f);
  if (wrapped < 0.0f) {
    wrapped += 180.0f;
  }
  return wrapped;
}

float fakeTorque(float angle) {
  if (angle <= closed_real) {
    return min(120.0f, 80.0f + abs(angle - closed_real) * 5.0f);
  }

  if (angle >= open_real) {
    return min(120.0f, 80.0f + abs(angle - open_real) * 5.0f);
  }

  float distance_to_limit = min(
    abs(angle - closed_real),
    abs(open_real - angle)
  );

  if (distance_to_limit < 10.0f) {
    return 10.0f + (10.0f - distance_to_limit) * 7.0f;
  }

  return 10.0f;
}

void randomizeEndpoints() {
  closed_real = random(-30, 0);
  open_real = closed_real + random(40, 61);
}

void sendLine(const String& line) {
  Serial.println(line);

  if (client && client.connected()) {
    client.println(line);
  }
}

void printStatus() {
  float torque = fakeTorque(door_angle);

  String state = "STATE ";
  state += "servo=" + String(servo_angle, 2);
  state += " door=" + String(door_angle, 2);
  state += " speed=" + String(speed_dps, 2);
  state += " dir=" + String(direction);
  state += " torque=" + String(torque, 2);
  state += " calibrated=" + String(calibrated ? 1 : 0);
  state += " detecting=" + String(detecting ? 1 : 0);
  state += " closed=" + String(closed_detected, 2);
  state += " open=" + String(open_detected, 2);

  sendLine(state);
}

// -------------------- Command Handling --------------------

void handleCommand(String cmd) {
  cmd.trim();

  if (cmd.length() == 0) {
    return;
  }

  if (cmd == "L") {
    direction = -1;
    sendLine("ACK LEFT");
  }
  else if (cmd == "R") {
    direction = 1;
    sendLine("ACK RIGHT");
  }
  else if (cmd == "S") {
    direction = 0;
    sendLine("ACK STOP");
  }
  else if (cmd == "+") {
    speed_dps = min(speed_dps + 10.0f, 180.0f);
    sendLine("ACK SPEED_UP");
  }
  else if (cmd == "-") {
    speed_dps = max(speed_dps - 10.0f, 10.0f);
    sendLine("ACK SPEED_DOWN");
  }
  else if (cmd == "CAL") {
    detecting = true;
    detect_armed = false;
    calibrated = false;
    detect_stage = 1;
    direction = -1;
    speed_dps = 20.0f;
    sendLine("CALIBRATION_STARTED");
  }
  else if (cmd == "RESET") {
    servo_angle = 0.0f;
    door_angle = 0.0f;
    direction = 0;
    speed_dps = 40.0f;

    closed_detected = 0.0f;
    open_detected = 0.0f;

    calibrated = false;
    detecting = false;
    detect_armed = false;
    detect_stage = 0;

    randomizeEndpoints();

    sendLine("RESET_DONE");
  }
  else if (cmd.startsWith("SPD ")) {
    speed_dps = constrain(cmd.substring(4).toFloat(), 10.0f, 180.0f);
    sendLine("ACK SPEED_SET");
  }
  else if (cmd == "?") {
    printStatus();
  }
  else {
    sendLine("ERR UNKNOWN_CMD " + cmd);
  }
}

// -------------------- Motion / Calibration --------------------

void updateMotion() {
  unsigned long now = millis();
  float dt = (now - last_ms) / 1000.0f;
  last_ms = now;

  float next_servo = servo_angle + direction * speed_dps * dt;
  float next_door = next_servo * GEAR_RATIO;

  if (calibrated && !detecting) {
    if (next_door < closed_detected) {
      next_door = closed_detected;
      next_servo = next_door / GEAR_RATIO;
      direction = 0;
      sendLine("LIMIT_REACHED CLOSED");
    }

    if (next_door > open_detected) {
      next_door = open_detected;
      next_servo = next_door / GEAR_RATIO;
      direction = 0;
      sendLine("LIMIT_REACHED OPEN");
    }
  }

  servo_angle = next_servo;
  door_angle = next_door;

  // Wokwi visual indicator only.
  // Continuous-rotation logic is stored in servo_angle.
  float visual_angle = wrapAngle180(servo_angle);
  visual_servo.write(visual_angle);
}

void update360Proxy() {
  const long target_steps = lroundf(
    servo_angle * PROXY_STEPS_PER_REVOLUTION / 360.0f
  );

  if (target_steps == proxy_step_position) {
    return;
  }

  const bool positive_rotation = target_steps > proxy_step_position;

  // A4988: LOW is counter-clockwise and HIGH is clockwise. This matches the
  // ROS top-view convention where a positive servo angle is counter-clockwise.
  digitalWrite(PROXY_DIR_PIN, positive_rotation ? LOW : HIGH);
  digitalWrite(PROXY_STEP_PIN, HIGH);
  delayMicroseconds(3);
  digitalWrite(PROXY_STEP_PIN, LOW);

  proxy_step_position += positive_rotation ? 1 : -1;
}

void updateCalibration() {
  if (!detecting) {
    return;
  }

  float torque = fakeTorque(door_angle);

  // If calibration starts outside the door range, first move away from that
  // already-loaded stop. Only a later low-to-high torque transition is a hit.
  if (!detect_armed) {
    if (torque < TORQUE_THRESHOLD) {
      detect_armed = true;
    }
    return;
  }

  if (torque >= TORQUE_THRESHOLD) {
    if (detect_stage == 1) {
      closed_detected = door_angle;
      direction = 1;
      detect_stage = 2;
      detect_armed = false;

      sendLine("CLOSED_DETECTED " + String(closed_detected, 2));
      delay(300);
    }
    else if (detect_stage == 2) {
      open_detected = door_angle;
      direction = 0;
      detecting = false;
      calibrated = true;
      detect_stage = 0;

      sendLine("OPEN_DETECTED " + String(open_detected, 2));
      sendLine("CALIBRATION_DONE");
    }
  }
}

// -------------------- Setup / Loop --------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  visual_servo.attach(SERVO_PIN);
  pinMode(PROXY_DIR_PIN, OUTPUT);
  pinMode(PROXY_STEP_PIN, OUTPUT);
  digitalWrite(PROXY_DIR_PIN, LOW);
  digitalWrite(PROXY_STEP_PIN, LOW);

  randomSeed(analogRead(0));
  randomizeEndpoints();

  last_ms = millis();

  sendLine("SMART_SERVO_READY");
  sendLine("GPIO18/19 drive the display-only 360-degree stepper proxy");
  sendLine("Commands: L R S + - CAL RESET SPD <value> ?");
  sendLine("Hidden real closed=" + String(closed_real, 2) + " open=" + String(open_real, 2));

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }

  Serial.println();
  sendLine("WiFi connected. IP=" + WiFi.localIP().toString());

  server.begin();
  sendLine("TCP server started on port " + String(TCP_PORT));
}

void loop() {
  if (!client || !client.connected()) {
    WiFiClient new_client = server.available();

    if (new_client) {
      client = new_client;
      sendLine("TCP_CLIENT_CONNECTED");
    }
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

  if (client && client.connected() && client.available()) {
    String cmd = client.readStringUntil('\n');
    handleCommand(cmd);
  }

  updateMotion();
  update360Proxy();
  updateCalibration();

  if (millis() - last_print > 100) {
    last_print = millis();
    printStatus();
  }
}
