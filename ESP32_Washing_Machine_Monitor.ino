#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID   "YOUR_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "Washing Machine Monitor"
#define BLYNK_AUTH_TOKEN    "YOUR_DEVICE_AUTH_TOKEN"

#include <WiFi.h>
#include <Wire.h>
#include <Preferences.h>
#include <BlynkSimpleEsp32.h>
#include <math.h>

// =====================================================
// WIFI
// =====================================================

char ssid[] = "YOUR_WIFI_NAME";
char pass[] = "YOUR_WIFI_PASSWORD";

// =====================================================
// HARDWARE PINS
// =====================================================

const int SDA_PIN = 42;
const int SCL_PIN = 41;

const int BUTTON_PIN = 4;   // A4 / GPIO4

// =====================================================
// ADXL345
// =====================================================

const uint8_t ADXL345_ADDR = 0x53;

const uint8_t REG_DEVID       = 0x00;
const uint8_t REG_BW_RATE     = 0x2C;
const uint8_t REG_POWER_CTL   = 0x2D;
const uint8_t REG_DATA_FORMAT = 0x31;
const uint8_t REG_DATAX0      = 0x32;

// Full-resolution ADXL345 scale is approximately 3.9 mg/LSB.
const float ADXL_SCALE = 0.0039f;

// =====================================================
// VIBRATION SETTINGS
// =====================================================

// Your tested bench threshold.
const float VIBRATION_THRESHOLD = 0.020f;

// 100 Hz sampling.
const unsigned long SAMPLE_INTERVAL_US = 10000;

// 100 samples = approximately 1 second.
const int SAMPLES_PER_WINDOW = 100;

// Require 3 consecutive windows before changing
// stable MOVING / STILL classification.
const int REQUIRED_WINDOWS = 3;

// =====================================================
// WASHER STATE TIMING
// =====================================================

// Require sustained activity before declaring a wash running.
const unsigned long RUN_CONFIRM_MS = 10000UL;

// After 30 seconds of quiet:
const unsigned long LOW_ACTIVITY_MS = 30000UL;

// Then require another 180 seconds quiet before warning.
const unsigned long WARNING_EXTRA_MS = 180000UL;

const unsigned long EARLY_WARNING_MS =
    LOW_ACTIVITY_MS + WARNING_EXTRA_MS;

// After 10 minutes of continuous quiet, return to IDLE.
const unsigned long IDLE_TIMEOUT_MS = 600000UL;

// =====================================================
// BUTTON SETTINGS
// =====================================================

const unsigned long BUTTON_DEBOUNCE_MS = 50UL;

const unsigned long LONG_PRESS_MS = 3000UL;

// =====================================================
// BLYNK / STORAGE
// =====================================================

BlynkTimer timer;
Preferences preferences;

// =====================================================
// WASHER STATES
// =====================================================

enum WasherState {
  STATE_IDLE = 0,
  STATE_RUNNING = 1,
  STATE_LOW_ACTIVITY = 2,
  STATE_POSSIBLY_FINISHING = 3
};

WasherState washerState = STATE_IDLE;

// =====================================================
// SENSOR VARIABLES
// =====================================================

float vibrationScore = 0.0f;

double magnitudeSum = 0.0;
double magnitudeSquaredSum = 0.0;

int sampleCount = 0;

unsigned long nextSampleUs = 0;

// =====================================================
// MOTION FILTER
// =====================================================

bool motionMoving = false;

int movingWindows = 0;
int stillWindows = 0;

// =====================================================
// WASHING STATE VARIABLES
// =====================================================

bool runActive = false;

unsigned long runCandidateSince = 0;
unsigned long lastMovingMs = 0;

bool earlyWarningSent = false;
bool earlyWarningPending = false;

// =====================================================
// WASH COUNTER
// =====================================================

uint8_t washingCycles = 0;

bool drumEventSent = false;

// =====================================================
// BUTTON VARIABLES
// =====================================================

int lastRawButtonState = HIGH;
int stableButtonState = HIGH;

unsigned long buttonDebounceStart = 0;
unsigned long buttonPressStart = 0;

bool longPressHandled = false;

// =====================================================
// NETWORK RECONNECT
// =====================================================

unsigned long lastWiFiAttempt = 0;
unsigned long lastBlynkAttempt = 0;

// =====================================================
// FORWARD DECLARATIONS
// =====================================================

bool writeADXLRegister(uint8_t reg, uint8_t value);
uint8_t readADXLRegister(uint8_t reg);
bool readAcceleration(float &x, float &y, float &z);
bool initialiseADXL345();

void sampleAccelerometer();
void processVibrationWindow(float score);
void updateWasherState();

void handleButton();
void incrementWashCycle();
void resetWashCycles();

void sendTelemetry();
void sendPendingEvents();
void maintainConnections();

// =====================================================
// BLYNK CONNECTION EVENT
// =====================================================

BLYNK_CONNECTED()
{
  Serial.println("Blynk connected.");

  // Make sure the cloud reset button is OFF.
  Blynk.virtualWrite(V5, 0);

  sendTelemetry();
  sendPendingEvents();
}

// =====================================================
// BLYNK RESET BUTTON
// =====================================================

BLYNK_WRITE(V5)
{
  int value = param.asInt();

  if (value == 1)
  {
    if (washingCycles == 15)
    {
      resetWashCycles();
    }
    else
    {
      Serial.println(
        "Remote reset ignored: counter has not reached 15."
      );
    }

    Blynk.virtualWrite(V5, 0);
  }
}

// =====================================================
// ADXL345 REGISTER WRITE
// =====================================================

bool writeADXLRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(ADXL345_ADDR);

  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

// =====================================================
// ADXL345 REGISTER READ
// =====================================================

uint8_t readADXLRegister(uint8_t reg)
{
  Wire.beginTransmission(ADXL345_ADDR);

  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
  {
    return 0xFF;
  }

  Wire.requestFrom(
    (uint8_t)ADXL345_ADDR,
    (uint8_t)1
  );

  if (Wire.available())
  {
    return Wire.read();
  }

  return 0xFF;
}

// =====================================================
// INITIALISE ADXL345
// =====================================================

bool initialiseADXL345()
{
  uint8_t deviceID = readADXLRegister(REG_DEVID);

  Serial.print("ADXL345 Device ID: 0x");
  Serial.println(deviceID, HEX);

  // ADXL345 DEVID should be 0xE5.
  if (deviceID != 0xE5)
  {
    Serial.println("ERROR: ADXL345 not detected.");
    return false;
  }

  // 100 Hz output data rate.
  writeADXLRegister(REG_BW_RATE, 0x0A);

  // Full-resolution mode, +/-4 g.
  writeADXLRegister(REG_DATA_FORMAT, 0x09);

  // Measurement mode.
  writeADXLRegister(REG_POWER_CTL, 0x08);

  delay(20);

  Serial.println("ADXL345 initialised.");

  return true;
}

// =====================================================
// READ XYZ
// =====================================================

bool readAcceleration(
  float &x,
  float &y,
  float &z
)
{
  Wire.beginTransmission(ADXL345_ADDR);

  Wire.write(REG_DATAX0);

  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  Wire.requestFrom(
    (uint8_t)ADXL345_ADDR,
    (uint8_t)6
  );

  if (Wire.available() < 6)
  {
    return false;
  }

  int16_t rawX =
    (int16_t)(Wire.read() | (Wire.read() << 8));

  int16_t rawY =
    (int16_t)(Wire.read() | (Wire.read() << 8));

  int16_t rawZ =
    (int16_t)(Wire.read() | (Wire.read() << 8));

  x = rawX * ADXL_SCALE;
  y = rawY * ADXL_SCALE;
  z = rawZ * ADXL_SCALE;

  return true;
}

// =====================================================
// NON-BLOCKING 100 HZ SENSOR SAMPLING
// =====================================================

void sampleAccelerometer()
{
  unsigned long nowUs = micros();

  if ((int32_t)(nowUs - nextSampleUs) < 0)
  {
    return;
  }

  nextSampleUs += SAMPLE_INTERVAL_US;

  // Avoid trying to rapidly catch up if the program
  // was delayed for a long time.
  if ((int32_t)(nowUs - nextSampleUs) >
      (int32_t)(SAMPLE_INTERVAL_US * 5))
  {
    nextSampleUs = nowUs + SAMPLE_INTERVAL_US;
  }

  float x;
  float y;
  float z;

  if (!readAcceleration(x, y, z))
  {
    return;
  }

  float magnitude =
    sqrtf(
      (x * x) +
      (y * y) +
      (z * z)
    );

  magnitudeSum += magnitude;

  magnitudeSquaredSum +=
    (double)magnitude *
    (double)magnitude;

  sampleCount++;

  if (sampleCount >= SAMPLES_PER_WINDOW)
  {
    double mean =
      magnitudeSum /
      sampleCount;

    double variance =
      (magnitudeSquaredSum / sampleCount) -
      (mean * mean);

    if (variance < 0.0)
    {
      variance = 0.0;
    }

    vibrationScore =
      (float)sqrt(variance);

    processVibrationWindow(vibrationScore);

    magnitudeSum = 0.0;
    magnitudeSquaredSum = 0.0;
    sampleCount = 0;
  }
}

// =====================================================
// MOVING / STILL CLASSIFICATION
// =====================================================

void processVibrationWindow(float score)
{
  if (score >= VIBRATION_THRESHOLD)
  {
    movingWindows++;

    if (movingWindows > REQUIRED_WINDOWS)
    {
      movingWindows = REQUIRED_WINDOWS;
    }

    stillWindows = 0;
  }
  else
  {
    stillWindows++;

    if (stillWindows > REQUIRED_WINDOWS)
    {
      stillWindows = REQUIRED_WINDOWS;
    }

    movingWindows = 0;
  }

  if (movingWindows >= REQUIRED_WINDOWS)
  {
    motionMoving = true;
  }

  if (stillWindows >= REQUIRED_WINDOWS)
  {
    motionMoving = false;
  }

  Serial.print("Vibration: ");
  Serial.print(vibrationScore, 4);

  Serial.print(" g | ");

  Serial.println(
    motionMoving ? "MOVING" : "STILL"
  );

  updateWasherState();
}

// =====================================================
// WASHER STATE MACHINE
// =====================================================

void updateWasherState()
{
  unsigned long now = millis();

  // ---------------------------------------------------
  // MOVEMENT DETECTED
  // ---------------------------------------------------

  if (motionMoving)
  {
    lastMovingMs = now;

    // Washer already confirmed as active.
    if (runActive)
    {
      washerState = STATE_RUNNING;
      return;
    }

    // First moving detection.
    if (runCandidateSince == 0)
    {
      runCandidateSince = now;
    }

    // Require sustained movement before accepting
    // this as a real washing cycle.
    if ((now - runCandidateSince) >= RUN_CONFIRM_MS)
    {
      runActive = true;

      washerState = STATE_RUNNING;

      lastMovingMs = now;

      earlyWarningSent = false;
      earlyWarningPending = false;

      Serial.println(
        "Washer state -> RUNNING"
      );
    }

    return;
  }

  // ---------------------------------------------------
  // NO MOVEMENT
  // ---------------------------------------------------

  runCandidateSince = 0;

  if (!runActive)
  {
    washerState = STATE_IDLE;
    return;
  }

  unsigned long quietTime =
    now - lastMovingMs;

  // ---------------------------------------------------
  // EARLY WARNING
  // ---------------------------------------------------

  if (quietTime >= EARLY_WARNING_MS)
  {
    washerState =
      STATE_POSSIBLY_FINISHING;

    if (!earlyWarningSent)
    {
      earlyWarningSent = true;

      if (Blynk.connected())
      {
        Blynk.logEvent(
          "washing_may_finish",
          "Washing machine may finish in approximately 5 minutes."
        );
      }
      else
      {
        earlyWarningPending = true;
      }

      Serial.println(
        "EARLY WARNING: Washer may finish soon."
      );
    }
  }

  // ---------------------------------------------------
  // LOW ACTIVITY
  // ---------------------------------------------------

  else if (quietTime >= LOW_ACTIVITY_MS)
  {
    washerState =
      STATE_LOW_ACTIVITY;
  }

  // ---------------------------------------------------
  // STILL PART OF ACTIVE CYCLE
  // ---------------------------------------------------

  else
  {
    washerState =
      STATE_RUNNING;
  }

  // ---------------------------------------------------
  // LONG QUIET PERIOD -> IDLE
  // ---------------------------------------------------

  if (quietTime >= IDLE_TIMEOUT_MS)
  {
    runActive = false;

    washerState =
      STATE_IDLE;

    lastMovingMs = 0;

    earlyWarningSent = false;

    // Avoid sending an obsolete warning later if
    // the network was offline for the entire period.
    earlyWarningPending = false;

    Serial.println(
      "Washer state -> IDLE"
    );
  }
}

// =====================================================
// BUTTON HANDLER
// =====================================================

void handleButton()
{
  int rawState =
    digitalRead(BUTTON_PIN);

  unsigned long now =
    millis();

  if (rawState != lastRawButtonState)
  {
    lastRawButtonState =
      rawState;

    buttonDebounceStart =
      now;
  }

  if ((now - buttonDebounceStart) >=
      BUTTON_DEBOUNCE_MS)
  {
    if (rawState != stableButtonState)
    {
      stableButtonState =
        rawState;

      // Button pressed.
      if (stableButtonState == LOW)
      {
        buttonPressStart =
          now;

        longPressHandled =
          false;
      }

      // Button released.
      else
      {
        if (!longPressHandled)
        {
          incrementWashCycle();
        }
      }
    }
  }

  // Long press.
  if (
    stableButtonState == LOW &&
    !longPressHandled &&
    (now - buttonPressStart >= LONG_PRESS_MS)
  )
  {
    longPressHandled = true;

    if (washingCycles == 15)
    {
      resetWashCycles();
    }
    else
    {
      Serial.println(
        "Long press ignored: drum-clean reset is available at 15 cycles."
      );
    }
  }
}

// =====================================================
// INCREMENT WASH COUNTER
// =====================================================

void incrementWashCycle()
{
  if (washingCycles >= 15)
  {
    Serial.println(
      "Counter already at 15."
    );

    return;
  }

  washingCycles++;

  preferences.putUChar(
    "cycles",
    washingCycles
  );

  Serial.print(
    "Washing cycles: "
  );

  Serial.println(
    washingCycles
  );

  // Reached maintenance threshold.
  if (washingCycles == 15)
  {
    drumEventSent = false;

    preferences.putBool(
      "drumSent",
      false
    );

    sendPendingEvents();
  }

  sendTelemetry();
}

// =====================================================
// RESET AFTER DRUM CLEAN
// =====================================================

void resetWashCycles()
{
  washingCycles = 0;

  drumEventSent = false;

  preferences.putUChar(
    "cycles",
    washingCycles
  );

  preferences.putBool(
    "drumSent",
    false
  );

  Serial.println(
    "Drum cleaning acknowledged."
  );

  Serial.println(
    "Washing-cycle counter reset to 0."
  );

  sendTelemetry();
}

// =====================================================
// SEND BLYNK EVENTS
// =====================================================

void sendPendingEvents()
{
  if (!Blynk.connected())
  {
    return;
  }

  // Drum-clean reminder.
  if (
    washingCycles >= 15 &&
    !drumEventSent
  )
  {
    Blynk.logEvent(
      "drum_clean_due",
      "15 washing cycles reached. Please perform a washing machine drum-cleaning cycle."
    );

    drumEventSent = true;

    preferences.putBool(
      "drumSent",
      true
    );

    Serial.println(
      "Blynk drum-clean notification sent."
    );
  }

  // Early-warning notification delayed because Wi-Fi
  // was unavailable.
  if (
    earlyWarningPending &&
    washerState ==
      STATE_POSSIBLY_FINISHING
  )
  {
    Blynk.logEvent(
      "washing_may_finish",
      "Washing machine may finish in approximately 5 minutes."
    );

    earlyWarningPending = false;

    Serial.println(
      "Pending early-warning notification sent."
    );
  }
}

// =====================================================
// BLYNK TELEMETRY
// =====================================================

void sendTelemetry()
{
  if (!Blynk.connected())
  {
    return;
  }

  unsigned long quietSeconds = 0;

  if (
    runActive &&
    !motionMoving &&
    lastMovingMs != 0
  )
  {
    quietSeconds =
      (millis() - lastMovingMs) / 1000UL;
  }

  // V0: vibration
  Blynk.virtualWrite(
    V0,
    vibrationScore
  );

  // V1: STILL / MOVING
  Blynk.virtualWrite(
    V1,
    motionMoving ? 1 : 0
  );

  // V2: cycle count
  Blynk.virtualWrite(
    V2,
    washingCycles
  );

  // V3: drum cleaning due
  Blynk.virtualWrite(
    V3,
    washingCycles >= 15 ? 1 : 0
  );

  // V4: washer status
  Blynk.virtualWrite(
    V4,
    (int)washerState
  );

  // V6: quiet time
  Blynk.virtualWrite(
    V6,
    quietSeconds
  );

  sendPendingEvents();
}

// =====================================================
// WIFI / BLYNK RECONNECT
// =====================================================

void maintainConnections()
{
  unsigned long now =
    millis();

  // ---------------------------------------------------
  // WIFI
  // ---------------------------------------------------

  if (WiFi.status() != WL_CONNECTED)
  {
    if (
      now - lastWiFiAttempt >=
      10000UL
    )
    {
      lastWiFiAttempt =
        now;

      Serial.println(
        "Attempting Wi-Fi connection..."
      );

      WiFi.disconnect();

      WiFi.begin(
        ssid,
        pass
      );
    }

    return;
  }

  // ---------------------------------------------------
  // BLYNK
  // ---------------------------------------------------

  if (!Blynk.connected())
  {
    if (
      now - lastBlynkAttempt >=
      5000UL
    )
    {
      lastBlynkAttempt =
        now;

      Serial.println(
        "Attempting Blynk connection..."
      );

      Blynk.connect(500);
    }
  }
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println(
    "================================="
  );

  Serial.println(
    "IoT Washing Machine Monitor"
  );

  Serial.println(
    "================================="
  );

  // ---------------------------------------------------
  // BUTTON
  // ---------------------------------------------------

  pinMode(
    BUTTON_PIN,
    INPUT_PULLUP
  );

  // ---------------------------------------------------
  // PERSISTENT STORAGE
  // ---------------------------------------------------

  preferences.begin(
    "washer",
    false
  );

  washingCycles =
    preferences.getUChar(
      "cycles",
      0
    );

  if (washingCycles > 15)
  {
    washingCycles = 15;

    preferences.putUChar(
      "cycles",
      15
    );
  }

  drumEventSent =
    preferences.getBool(
      "drumSent",
      false
    );

  if (washingCycles < 15)
  {
    drumEventSent = false;
  }

  Serial.print(
    "Stored washing cycles: "
  );

  Serial.println(
    washingCycles
  );

  // ---------------------------------------------------
  // I2C
  // ---------------------------------------------------

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  Wire.setClock(
    400000
  );

  // ---------------------------------------------------
  // ADXL345
  // ---------------------------------------------------

  if (!initialiseADXL345())
  {
    Serial.println(
      "STOP: Check ADXL345 wiring."
    );

    while (true)
    {
      delay(1000);
    }
  }

  // ---------------------------------------------------
  // WIFI
  // ---------------------------------------------------

  WiFi.mode(
    WIFI_STA
  );

  WiFi.begin(
    ssid,
    pass
  );

  // Configure Blynk without blocking the whole local
  // washing-machine monitor while internet is unavailable.
  Blynk.config(
    BLYNK_AUTH_TOKEN
  );

  lastWiFiAttempt =
    millis();

  lastBlynkAttempt =
    millis();

  // ---------------------------------------------------
  // TELEMETRY TIMER
  // ---------------------------------------------------

  timer.setInterval(
    5000L,
    sendTelemetry
  );

  nextSampleUs =
    micros();

  Serial.println(
    "System ready."
  );
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
  // Local functions continue working even if
  // Wi-Fi/Blynk temporarily goes offline.

  sampleAccelerometer();

  handleButton();

  maintainConnections();

  if (Blynk.connected())
  {
    Blynk.run();
  }

  timer.run();
}