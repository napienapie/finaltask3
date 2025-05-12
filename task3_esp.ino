#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Hardware configuration
#define PIN_ENC_A 12        // Encoder A phase
#define PIN_ENC_B 13        // Encoder B phase
#define PIN_PWM 16          // Motor PWM signal
#define PIN_DIR_A 17        // Motor direction A
#define PIN_DIR_B 5         // Motor direction B

// Motor control constants
const float PULSES_PER_ROT = 920.0;   // Encoder pulses per rotation
const int MAX_PWM_VALUE = 255;        // PWM range limit
const unsigned long SPEED_UPDATE_MS = 200; // Speed reporting period
float kpGain = 20.0;                  // PID proportional coefficient
float kiGain = 70.0;                  // PID integral coefficient
float kdGain = 5.0;                   // PID derivative coefficient

// BLE UUID definitions
#define MOTOR_SVC_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define RPM_TARGET_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define RPM_ACTUAL_UUID     "ca73b3ba-39f6-4ab3-91ae-186dc9577d99"
#define PID_KP_UUID         "d5e4b2a1-3f8c-4e5b-9a2d-7b3e8c1f0a2b"
#define PID_KI_UUID         "e6f5c3b2-4f9d-5f6c-ab3e-8c4f9d2e1b3c"
#define PID_KD_UUID         "f7e6d4c3-5fae-4e7d-9c4f-9d5e0e3f2c4d"

// Motor controller class
class MotorController {
private:
  volatile int32_t encPosition = 0;     // Current encoder count
  int32_t lastEncPosition = 0;          // Previous encoder count
  unsigned long lastPidTick = 0;        // Last PID update (us)
  unsigned long lastSpeedTick = 0;      // Last speed update (ms)

  struct ControlState {
    float setpointRpm = 0.0;    // Target speed in RPM
    float currentRpm = 0.0;     // Measured speed in RPM
    float pidError = 0.0;       // Current PID error
    float prevPidError = 0.0;   // Previous PID error
    float integralTerm = 0.0;   // Accumulated integral
    float derivativeTerm = 0.0; // Rate of error change
    float pwmSignal = 0.0;      // Output PWM value
  } state;

public:
  void updatePid() {
    unsigned long now = micros();
    float dt = (now - lastPidTick) / 1000000.0; // Time delta in seconds
    lastPidTick = now;

    state.pidError = state.setpointRpm - state.currentRpm;
    state.integralTerm += state.pidError * dt;
    state.derivativeTerm = (state.pidError - state.prevPidError) / dt;
    state.pwmSignal = kpGain * state.pidError + 
                      kiGain * state.integralTerm + 
                      kdGain * state.derivativeTerm;

    state.pwmSignal = constrain(state.pwmSignal, -MAX_PWM_VALUE, MAX_PWM_VALUE);
    state.prevPidError = state.pidError;

    applyPwm(state.pwmSignal);
  }

  void updateSpeed(BLECharacteristic *charRpm) {
    unsigned long now = millis();
    if (now - lastSpeedTick < SPEED_UPDATE_MS) return;

    int32_t encDelta = encPosition - lastEncPosition;
    float rotations = encDelta / PULSES_PER_ROT;
    float timeSec = (now - lastSpeedTick) / 1000.0;
    state.currentRpm = (rotations / timeSec) * 60.0; // RPM
    lastEncPosition = encPosition;
    lastSpeedTick = now;

    char rpmBuffer[10];
    snprintf(rpmBuffer, sizeof(rpmBuffer), "%.1f", state.currentRpm);
    charRpm->setValue((uint8_t*)rpmBuffer, strlen(rpmBuffer));
    charRpm->notify();
  }

  void incrementEncoder(int delta) {
    encPosition += delta;
  }

  void setTargetRpm(float rpm) {
    state.setpointRpm = rpm;
    state.integralTerm = 0.0; // Clear integral on setpoint change
  }

private:
  void applyPwm(float pwm) {
    int pwmAbs = abs(pwm);
    digitalWrite(PIN_DIR_A, pwm > 0 ? HIGH : LOW);
    digitalWrite(PIN_DIR_B, pwm < 0 ? HIGH : LOW);
    analogWrite(PIN_PWM, pwmAbs);
  }
};

// Global motor controller instance
MotorController motor;

// BLE characteristics
BLECharacteristic *charTargetRpm;
BLECharacteristic *charActualRpm;
BLECharacteristic *charKpGain;
BLECharacteristic *charKiGain;
BLECharacteristic *charKdGain;

// Encoder interrupt handlers
void IRAM_ATTR encAInterrupt() {
  motor.incrementEncoder(digitalRead(PIN_ENC_A) == digitalRead(PIN_ENC_B) ? 1 : -1);
}

void IRAM_ATTR encBInterrupt() {
  motor.incrementEncoder(digitalRead(PIN_ENC_A) != digitalRead(PIN_ENC_B) ? 1 : -1);
}

// BLE server callbacks
class ConnectionHandler : public BLEServerCallbacks {
  void onConnect(BLEServer* server) {
    Serial.println("Client connected via BLE");
  }
  void onDisconnect(BLEServer* server) {
    Serial.println("Client disconnected from BLE");
    BLEDevice::startAdvertising();
  }
};

// BLE characteristic callbacks
class TargetRpmHandler : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *charac) {
    String data = charac->getValue();
    if (data.length() > 0) {
      float newRpm = atof(data.c_str());
      motor.setTargetRpm(newRpm);
      Serial.printf("Updated target RPM: %.1f\n", newRpm);
    }
  }
};

class KpGainHandler : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *charac) {
    String data = charac->getValue();
    if (data.length() > 0) {
      kpGain = atof(data.c_str());
      Serial.printf("Updated Kp gain: %.1f\n", kpGain);
    }
  }
};

class KiGainHandler : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *charac) {
    String data = charac->getValue();
    if (data.length() > 0) {
      kiGain = atof(data.c_str());
      Serial.printf("Updated Ki gain: %.1f\n", kiGain);
    }
  }
};

class KdGainHandler : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *charac) {
    String data = charac->getValue();
    if (data.length() > 0) {
      kdGain = atof(data.c_str());
      Serial.printf("Updated Kd gain: %.1f\n", kdGain);
    }
  }
};

// System initialization
void setup() {
  Serial.begin(115200);
  delay(50); // Wait for serial stabilization

  // Initialize pins
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);

  // Configure encoder interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encAInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encBInterrupt, CHANGE);

  // Set up BLE
  BLEDevice::init("MotorBLE");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ConnectionHandler());
  BLEService *service = server->createService(MOTOR_SVC_UUID);

  // Configure BLE characteristics
  charTargetRpm = service->createCharacteristic(
    RPM_TARGET_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  charTargetRpm->setCallbacks(new TargetRpmHandler());

  charActualRpm = service->createCharacteristic(
    RPM_ACTUAL_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  charActualRpm->addDescriptor(new BLE2902());

  charKpGain = service->createCharacteristic(
    PID_KP_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  charKpGain->setCallbacks(new KpGainHandler());

  charKiGain = service->createCharacteristic(
    PID_KI_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  charKiGain->setCallbacks(new KiGainHandler());

  charKdGain = service->createCharacteristic(
    PID_KD_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  charKdGain->setCallbacks(new KdGainHandler());

  // Activate BLE service
  service->start();
  BLEAdvertising *advert = BLEDevice::getAdvertising();
  advert->addServiceUUID(MOTOR_SVC_UUID);
  advert->setScanResponse(true);
  advert->setMinPreferred(0x06);
  BLEDevice::startAdvertising();
  Serial.println("BLE initialized");
}

// Main loop
void loop() {
  motor.updateSpeed(charActualRpm);
  motor.updatePid();
}