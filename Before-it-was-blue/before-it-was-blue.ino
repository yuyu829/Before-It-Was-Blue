#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>

// ============================================================
// BEFORE IT WAS BLUE
// 2 TEC + 2 VIBRATION MOTOR + MAX6675 + PID + OFFLINE WEB
// ============================================================


// ============================================================
// PIN ASSIGNMENT
// ============================================================

// Vibration motors
const int PIN_VIB_1 = 25;
const int PIN_VIB_2 = 32;

// TEC MOSFET gates
const int PIN_TEC_GATE_1 = 26;
const int PIN_TEC_GATE_2 = 33;

// MAX6675
const int PIN_TC_SCK = 18;
const int PIN_TC_CS  = 27;
const int PIN_TC_SO  = 19;


// ============================================================
// WIFI
// ============================================================

const char* AP_SSID = "PorcelainDemo";
const char* AP_PASS = "12345678";

WebServer server(80);


// ============================================================
// TEMPERATURE SETTINGS
// ============================================================

const float TARGET_C = 37.5f;
const float HARD_STOP_C = 41.0f;

const unsigned long TEMP_SAMPLE_MS = 500;
const unsigned long SENSOR_STALE_MS = 1500;


// ============================================================
// TEMPERATURE STATE
// ============================================================

float tempC = NAN;

bool sensorValid = false;

unsigned long lastTempRead = 0;
unsigned long lastGoodTempRead = 0;


// ============================================================
// TEC STATE
// ============================================================

bool interactionActive = false;

bool targetReachedThisHold = false;

bool faultLatched = false;

String faultReason = "";


// ============================================================
// FIRING STAGE STATE
// ============================================================

bool vib1200Reached = false;
bool vib1300Reached = false;

bool pauseHoldActive = false;

float holdTempC = NAN;


// ============================================================
// PID SETTINGS
// ============================================================

float PID_KP = 24.0f;
float PID_KI = 0.8f;
float PID_KD = 0.2f;

const unsigned long PID_WINDOW_MS = 1000;

float pidIntegral = 0.0f;
float pidPrevError = 0.0f;

bool pidInitialized = false;

unsigned long pidPrevTime = 0;
unsigned long pidWindowStart = 0;

float pidOutputPct = 0.0f;


// ============================================================
// SIMULATED KILN
// ============================================================

float kilnTemp = 25.0f;


// ============================================================
// VIBRATION STATE
// ============================================================

enum VibState {

  VIB_IDLE,

  VIB_SINGLE_ON,

  VIB_DOUBLE_ON1,
  VIB_DOUBLE_GAP,
  VIB_DOUBLE_ON2
};

VibState vibState = VIB_IDLE;

unsigned long vibStateStarted = 0;


// ============================================================
// VIBRATION CONTROL
// ============================================================

void vibrationOn() {

  digitalWrite(PIN_VIB_1, HIGH);
  digitalWrite(PIN_VIB_2, HIGH);
}


void vibrationOff() {

  digitalWrite(PIN_VIB_1, LOW);
  digitalWrite(PIN_VIB_2, LOW);
}


void startSingleVibration() {

  if (vibState != VIB_IDLE)
    return;

  Serial.println(">>> SINGLE VIBRATION START");

  vibrationOn();

  vibState = VIB_SINGLE_ON;

  vibStateStarted = millis();
}


void startDoubleVibration() {

  if (vibState != VIB_IDLE)
    return;

  Serial.println(">>> DOUBLE VIBRATION START");

  vibrationOn();

  vibState = VIB_DOUBLE_ON1;

  vibStateStarted = millis();
}


void updateVibration() {

  unsigned long now = millis();

  switch (vibState) {

    case VIB_SINGLE_ON:

      if (now - vibStateStarted >= 500) {

        vibrationOff();

        vibState = VIB_IDLE;

        Serial.println(">>> SINGLE VIBRATION END");
      }

      break;


    case VIB_DOUBLE_ON1:

      if (now - vibStateStarted >= 350) {

        vibrationOff();

        vibState = VIB_DOUBLE_GAP;

        vibStateStarted = now;
      }

      break;


    case VIB_DOUBLE_GAP:

      if (now - vibStateStarted >= 220) {

        vibrationOn();

        vibState = VIB_DOUBLE_ON2;

        vibStateStarted = now;
      }

      break;


    case VIB_DOUBLE_ON2:

      if (now - vibStateStarted >= 350) {

        vibrationOff();

        vibState = VIB_IDLE;

        Serial.println(">>> DOUBLE VIBRATION END");
      }

      break;


    default:

      break;
  }
}


// ============================================================
// TEC CONTROL
// ============================================================

void heaterOn() {

  digitalWrite(PIN_TEC_GATE_1, HIGH);
  digitalWrite(PIN_TEC_GATE_2, HIGH);
}


void heaterOff() {

  digitalWrite(PIN_TEC_GATE_1, LOW);
  digitalWrite(PIN_TEC_GATE_2, LOW);
}


// ============================================================
// FAULT
// ============================================================

void triggerFault(const char* reason) {

  heaterOff();

  interactionActive = false;

  targetReachedThisHold = false;

  faultLatched = true;

  faultReason = reason;

  Serial.print("FAULT -> TEC OFF: ");
  Serial.println(reason);
}


// ============================================================
// MAX6675
// ============================================================

uint16_t readMAX6675Raw() {

  uint16_t raw = 0;

  digitalWrite(PIN_TC_SCK, LOW);

  digitalWrite(PIN_TC_CS, LOW);

  delayMicroseconds(10);


  for (int i = 15; i >= 0; --i) {

    digitalWrite(PIN_TC_SCK, LOW);

    delayMicroseconds(10);


    if (digitalRead(PIN_TC_SO)) {

      raw |= (uint16_t)(1U << i);
    }


    digitalWrite(PIN_TC_SCK, HIGH);

    delayMicroseconds(10);
  }


  digitalWrite(PIN_TC_CS, HIGH);

  digitalWrite(PIN_TC_SCK, LOW);


  return raw;
}


bool readTemperature(float& result) {

  uint16_t raw = readMAX6675Raw();


  // Invalid frame
  if (raw & 0x8000) {

    return false;
  }


  // Thermocouple disconnected
  if (raw & 0x0004) {

    return false;
  }


  float t =
    (((raw >> 3) & 0x0FFF))
    * 0.25f;


  if (
    t < 0.0f
    ||
    t > 100.0f
  ) {

    return false;
  }


  result = t;

  return true;
}


// ============================================================
// PID
// ============================================================

void resetPID() {

  pidIntegral = 0.0f;

  pidPrevError = 0.0f;

  pidInitialized = false;

  pidPrevTime = millis();

  pidWindowStart = millis();

  pidOutputPct = 0.0f;
}


float computePID(float setpoint) {

  unsigned long now = millis();


  float dt =
    (now - pidPrevTime)
    /
    1000.0f;


  if (
    dt <= 0.0f
    ||
    dt > 2.0f
  ) {

    dt = 0.5f;
  }


  float error =
    setpoint - tempC;


  if (!pidInitialized) {

    pidPrevError = error;

    pidInitialized = true;
  }


  pidIntegral +=
    error * dt;


  const float INTEGRAL_LIMIT = 80.0f;


  if (pidIntegral > INTEGRAL_LIMIT)
    pidIntegral = INTEGRAL_LIMIT;


  if (pidIntegral < -INTEGRAL_LIMIT)
    pidIntegral = -INTEGRAL_LIMIT;


  float derivative =
    (error - pidPrevError)
    /
    dt;


  float output =

      PID_KP * error

    +

      PID_KI * pidIntegral

    +

      PID_KD * derivative;


  if (output > 100.0f)
    output = 100.0f;


  if (output < 0.0f)
    output = 0.0f;


  pidPrevError = error;

  pidPrevTime = now;

  pidOutputPct = output;


  return output;
}


// ============================================================
// PID TIME-PROPORTIONAL OUTPUT
// ============================================================

void applyPIDOutput(float outputPct) {

  unsigned long now = millis();


  if (
    now - pidWindowStart
    >=
    PID_WINDOW_MS
  ) {

    pidWindowStart = now;
  }


  unsigned long onTime =

    (unsigned long)
    (
      PID_WINDOW_MS
      *
      outputPct
      /
      100.0f
    );


  if (
    now - pidWindowStart
    <
    onTime
  ) {

    heaterOn();

  } else {

    heaterOff();
  }
}


// ============================================================
// RUN PID
// ============================================================

void runPIDToSetpoint(float setpoint) {

  if (
    !sensorValid
    ||
    isnan(setpoint)
  ) {

    heaterOff();

    return;
  }


  if (
    tempC
    >=
    HARD_STOP_C
  ) {

    triggerFault(
      "Hard temperature cutoff reached"
    );

    return;
  }


  float output =
    computePID(setpoint);


  applyPIDOutput(output);
}


// ============================================================
// TEMPERATURE HOLD
// ============================================================

void maintainTemperature(float setpoint) {

  runPIDToSetpoint(setpoint);
}


// ============================================================
// TEC STATE LOGIC
// ============================================================

void updateTECControl() {

  unsigned long now = millis();


  // ==========================================================
  // FAULT
  // ==========================================================

  if (faultLatched) {

    heaterOff();

    return;
  }


  // ==========================================================
  // SENSOR
  // ==========================================================

  if (
    !sensorValid
    ||
    now - lastGoodTempRead
    >
    SENSOR_STALE_MS
  ) {

    heaterOff();


    if (interactionActive) {

      triggerFault(
        "Temperature sensor unavailable"
      );
    }

    return;
  }


  // ==========================================================
  // HARD STOP
  // ==========================================================

  if (
    tempC
    >=
    HARD_STOP_C
  ) {

    triggerFault(
      "Hard temperature cutoff reached"
    );

    return;
  }


  // ==========================================================
  // AFTER 1300C
  // ALWAYS COOL
  // ==========================================================

  if (vib1300Reached) {

    heaterOff();

    return;
  }


  // ==========================================================
  // BETWEEN 1200C AND 1300C
  // ==========================================================

  if (vib1200Reached) {


    // Still holding:
    // maintain temperature locked at 1200C

    if (interactionActive) {

      maintainTemperature(
        holdTempC
      );

    }

    // Released after 1200:
    // natural cooling

    else {

      heaterOff();
    }


    return;
  }


  // ==========================================================
  // BEFORE 1200C
  // ==========================================================

  if (interactionActive) {

    pauseHoldActive = false;


    if (
      tempC
      >=
      TARGET_C
    ) {

      targetReachedThisHold = true;
    }


    runPIDToSetpoint(
      TARGET_C
    );


    return;
  }


  // ==========================================================
  // RELEASE BEFORE 1200C
  // MAINTAIN CURRENT REAL TEMP
  // ==========================================================

  if (pauseHoldActive) {

    maintainTemperature(
      holdTempC
    );

    return;
  }


  heaterOff();
}


// ============================================================
// LITTLEFS
// ============================================================

String getContentType(const String& path) {

  if (path.endsWith(".html"))
    return "text/html";

  if (path.endsWith(".css"))
    return "text/css";

  if (path.endsWith(".js"))
    return "application/javascript";

  if (path.endsWith(".png"))
    return "image/png";

  if (
    path.endsWith(".jpg")
    ||
    path.endsWith(".jpeg")
  )
    return "image/jpeg";

  if (path.endsWith(".webp"))
    return "image/webp";

  if (path.endsWith(".svg"))
    return "image/svg+xml";

  if (path.endsWith(".ico"))
    return "image/x-icon";


  return "application/octet-stream";
}


bool serveFile(String path) {

  if (path.endsWith("/")) {

    path += "index.html";
  }


  if (!LittleFS.exists(path)) {

    return false;
  }


  File file =
    LittleFS.open(
      path,
      "r"
    );


  if (!file) {

    return false;
  }


  if (path.endsWith(".html")) {

    server.sendHeader(
      "Cache-Control",
      "no-cache, no-store, must-revalidate"
    );

    server.sendHeader(
      "Pragma",
      "no-cache"
    );

    server.sendHeader(
      "Expires",
      "0"
    );
  }


  else if (

    path.endsWith(".webp")
    ||
    path.endsWith(".png")
    ||
    path.endsWith(".jpg")
    ||
    path.endsWith(".jpeg")

  ) {

    server.sendHeader(
      "Cache-Control",
      "public, max-age=86400"
    );
  }


  server.streamFile(
    file,
    getContentType(path)
  );


  file.close();

  return true;
}


// ============================================================
// ROOT
// ============================================================

void handleRoot() {

  if (
    !serveFile(
      "/index.html"
    )
  ) {

    server.send(
      500,
      "text/plain",
      "index.html not found in LittleFS"
    );
  }
}


// ============================================================
// START
// ============================================================

void handleStart() {

  if (server.hasArg("t")) {

    kilnTemp =
      server
      .arg("t")
      .toFloat();
  }


  if (faultLatched) {

    heaterOff();

    server.send(
      423,
      "text/plain",
      "SAFETY LOCKED"
    );

    return;
  }


  if (

    !sensorValid

    ||

    millis()
    -
    lastGoodTempRead
    >
    SENSOR_STALE_MS

  ) {

    heaterOff();

    server.send(
      503,
      "text/plain",
      "SENSOR ERROR"
    );

    return;
  }


  if (
    tempC
    >=
    HARD_STOP_C
  ) {

    triggerFault(
      "Hard cutoff before start"
    );

    server.send(
      423,
      "text/plain",
      "SAFETY LOCKED"
    );

    return;
  }


  interactionActive = true;

  pauseHoldActive = false;

  targetReachedThisHold = false;


  resetPID();

  updateTECControl();


  Serial.print(
    "WEB HOLD | kiln="
  );

  Serial.print(kilnTemp);

  Serial.print(
    " | real="
  );

  Serial.print(tempC);


  Serial.print(
    " C | TEC1="
  );

  Serial.print(
    digitalRead(PIN_TEC_GATE_1)
    ?
    "HIGH"
    :
    "LOW"
  );


  Serial.print(
    " | TEC2="
  );

  Serial.println(
    digitalRead(PIN_TEC_GATE_2)
    ?
    "HIGH"
    :
    "LOW"
  );


  server.send(
    200,
    "text/plain",
    "START"
  );
}


// ============================================================
// STOP
// ============================================================

void handleStop() {

  if (server.hasArg("t")) {

    kilnTemp =
      server
      .arg("t")
      .toFloat();
  }


  interactionActive = false;

  targetReachedThisHold = false;


  // ==========================================================
  // RELEASE BEFORE 1200
  // HOLD CURRENT REAL TEMPERATURE
  // ==========================================================

  if (!vib1200Reached) {

    if (sensorValid) {

      holdTempC = tempC;

      pauseHoldActive = true;


      resetPID();


      Serial.print(
        "WEB RELEASE BEFORE 1200 -> HOLD TEMP at "
      );

      Serial.print(
        holdTempC,
        2
      );

      Serial.println(" C");

    } else {

      pauseHoldActive = false;

      heaterOff();


      Serial.println(
        "WEB RELEASE BEFORE 1200 -> SENSOR INVALID -> TEC OFF"
      );
    }
  }


  // ==========================================================
  // RELEASE AFTER 1200
  // COOL
  // ==========================================================

  else {

    pauseHoldActive = false;

    heaterOff();


    Serial.print(
      "WEB RELEASE AFTER 1200 -> COOL DOWN | real="
    );

    Serial.print(
      tempC,
      2
    );

    Serial.println(" C");
  }


  server.send(
    200,
    "text/plain",
    "STOP"
  );
}


// ============================================================
// KILN
// ============================================================

void handleKiln() {

  if (server.hasArg("t")) {

    kilnTemp =
      server
      .arg("t")
      .toFloat();
  }


  server.send(
    200,
    "text/plain",
    "OK"
  );
}


// ============================================================
// 1200C VIBRATION
// ============================================================

void handleVib1() {

  Serial.println(
    "WEB -> VIB 1"
  );


  startSingleVibration();


  if (!vib1200Reached) {

    vib1200Reached = true;

    pauseHoldActive = false;


    if (sensorValid) {

      holdTempC = tempC;


      resetPID();


      Serial.print(
        "1200 reached -> LOCK TEMP at "
      );

      Serial.print(
        holdTempC,
        2
      );

      Serial.println(" C");

    } else {

      heaterOff();

      Serial.println(
        "1200 reached -> SENSOR INVALID -> TEC OFF"
      );
    }
  }


  server.send(
    200,
    "text/plain",
    "VIB1"
  );
}


// ============================================================
// 1300C VIBRATION
// ============================================================

void handleVib2() {

  Serial.println(
    "WEB -> VIB 2"
  );


  startDoubleVibration();


  vib1300Reached = true;

  pauseHoldActive = false;


  heaterOff();

  resetPID();


  Serial.print(
    "1300 reached -> TEC OFF -> COOL DOWN | real="
  );

  Serial.print(
    tempC,
    2
  );

  Serial.println(" C");


  server.send(
    200,
    "text/plain",
    "VIB2"
  );
}


// ============================================================
// MANUAL VIBRATION TEST
// ============================================================

void handleTestVib() {

  Serial.println(
    "MANUAL -> BOTH MOTORS"
  );


  startSingleVibration();


  server.send(
    200,
    "text/plain",
    "VIBRATION TEST"
  );
}


// ============================================================
// RESET
// ============================================================

void handleReset() {

  interactionActive = false;

  targetReachedThisHold = false;

  kilnTemp = 25.0f;


  heaterOff();

  vibrationOff();


  vibState = VIB_IDLE;


  vib1200Reached = false;

  vib1300Reached = false;

  pauseHoldActive = false;

  holdTempC = NAN;


  resetPID();


  if (

    sensorValid

    &&

    millis()
    -
    lastGoodTempRead
    <=
    SENSOR_STALE_MS

    &&

    tempC
    <
    TARGET_C

  ) {

    faultLatched = false;

    faultReason = "";


    Serial.println(
      "SAFETY LOCK CLEARED"
    );
  }


  Serial.println(
    "SIMULATION RESET"
  );


  server.send(
    200,
    "text/plain",
    "RESET"
  );
}


// ============================================================
// STATUS
// ============================================================

void handleStatus() {

  bool sensorReady =

    sensorValid

    &&

    millis()
    -
    lastGoodTempRead
    <=
    SENSOR_STALE_MS;


  String json = "{";


  json += "\"kiln\":";

  json +=
    String(
      kilnTemp,
      1
    );


  json += ",\"real\":";


  json +=

    sensorReady

    ?

    String(
      tempC,
      2
    )

    :

    "0";


  json += ",\"sensorReady\":";

  json +=

    sensorReady
    ?
    "true"
    :
    "false";


  json += ",\"heating\":";

  json +=

    (
      digitalRead(PIN_TEC_GATE_1)
      ||
      digitalRead(PIN_TEC_GATE_2)
    )

    ?

    "true"

    :

    "false";


  json += ",\"tec1\":";

  json +=

    digitalRead(PIN_TEC_GATE_1)
    ?
    "true"
    :
    "false";


  json += ",\"tec2\":";

  json +=

    digitalRead(PIN_TEC_GATE_2)
    ?
    "true"
    :
    "false";


  json += ",\"motor1\":";

  json +=

    digitalRead(PIN_VIB_1)
    ?
    "true"
    :
    "false";


  json += ",\"motor2\":";

  json +=

    digitalRead(PIN_VIB_2)
    ?
    "true"
    :
    "false";


  json += ",\"targetReached\":";

  json +=

    targetReachedThisHold
    ?
    "true"
    :
    "false";


  json += ",\"pidOutput\":";

  json +=
    String(
      pidOutputPct,
      1
    );


  json += ",\"fault\":";

  json +=

    faultLatched
    ?
    "true"
    :
    "false";


  json += ",\"faultReason\":\"";

  json += faultReason;

  json += "\"";


  json += ",\"vibrating\":";

  json +=

    vibState != VIB_IDLE
    ?
    "true"
    :
    "false";


  json += "}";


  server.sendHeader(
    "Cache-Control",
    "no-store"
  );


  server.send(
    200,
    "application/json",
    json
  );
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(
    115200
  );


  delay(
    300
  );


  // ==========================================================
  // MOTOR 1
  // ==========================================================

  pinMode(
    PIN_VIB_1,
    OUTPUT
  );


  // ==========================================================
  // MOTOR 2
  // ==========================================================

  pinMode(
    PIN_VIB_2,
    OUTPUT
  );


  vibrationOff();


  // ==========================================================
  // TEC 1
  // ==========================================================

  pinMode(
    PIN_TEC_GATE_1,
    OUTPUT
  );


  // ==========================================================
  // TEC 2
  // ==========================================================

  pinMode(
    PIN_TEC_GATE_2,
    OUTPUT
  );


  heaterOff();


  resetPID();


  // ==========================================================
  // MAX6675
  // ==========================================================

  pinMode(
    PIN_TC_SCK,
    OUTPUT
  );


  pinMode(
    PIN_TC_CS,
    OUTPUT
  );


  pinMode(
    PIN_TC_SO,
    INPUT
  );


  digitalWrite(
    PIN_TC_CS,
    HIGH
  );


  digitalWrite(
    PIN_TC_SCK,
    LOW
  );


  Serial.println();


  Serial.println(
    "=== BEFORE IT WAS BLUE ==="
  );


  Serial.println(
    "2 TEC + 2 MOTOR + MAX6675 + PID"
  );


  Serial.print(
    "Target: "
  );

  Serial.print(
    TARGET_C
  );

  Serial.println(
    " C"
  );


  Serial.print(
    "Hard cutoff: "
  );

  Serial.print(
    HARD_STOP_C
  );

  Serial.println(
    " C"
  );


  // ==========================================================
  // LITTLEFS
  // ==========================================================

  if (
    !LittleFS.begin(
      true
    )
  ) {

    Serial.println(
      "LittleFS mount FAILED"
    );

  } else {

    Serial.println(
      "LittleFS mounted"
    );
  }


  // ==========================================================
  // WIFI AP
  // ==========================================================

  WiFi.mode(
    WIFI_AP
  );


  WiFi.softAP(
    AP_SSID,
    AP_PASS
  );


  Serial.print(
    "WiFi: "
  );

  Serial.println(
    AP_SSID
  );


  Serial.print(
    "Open: http://"
  );

  Serial.println(
    WiFi.softAPIP()
  );


  // ==========================================================
  // WEB ROUTES
  // ==========================================================

  server.on(
    "/",
    handleRoot
  );


  server.on(
    "/start",
    handleStart
  );


  server.on(
    "/stop",
    handleStop
  );


  server.on(
    "/kiln",
    handleKiln
  );


  server.on(
    "/vib1",
    handleVib1
  );


  server.on(
    "/vib2",
    handleVib2
  );


  server.on(
    "/testvib",
    handleTestVib
  );


  server.on(
    "/reset",
    handleReset
  );


  server.on(
    "/status",
    handleStatus
  );


  server.onNotFound(
    []() {

      String path =
        server.uri();


      if (
        serveFile(
          path
        )
      ) {

        return;
      }


      server.send(
        404,
        "text/plain",
        "File not found: "
        +
        path
      );
    }
  );


  server.begin();


  Serial.println(
    "Server ready"
  );
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  unsigned long now =
    millis();


  // ==========================================================
  // WEB
  // ==========================================================

  server.handleClient();


  // ==========================================================
  // TEMPERATURE
  // ==========================================================

  if (

    now
    -
    lastTempRead
    >=
    TEMP_SAMPLE_MS

  ) {

    lastTempRead = now;


    float measured = NAN;


    if (
      readTemperature(
        measured
      )
    ) {

      tempC = measured;

      sensorValid = true;

      lastGoodTempRead = now;


      Serial.print(
        "Real: "
      );

      Serial.print(
        tempC,
        2
      );


      Serial.print(
        " C | Target: "
      );

      Serial.print(
        TARGET_C,
        1
      );


      Serial.print(
        " | TEC1: "
      );

      Serial.print(
        digitalRead(PIN_TEC_GATE_1)
        ?
        "HIGH"
        :
        "LOW"
      );


      Serial.print(
        " | TEC2: "
      );

      Serial.print(
        digitalRead(PIN_TEC_GATE_2)
        ?
        "HIGH"
        :
        "LOW"
      );


      Serial.print(
        " | M1: "
      );

      Serial.print(
        digitalRead(PIN_VIB_1)
        ?
        "HIGH"
        :
        "LOW"
      );


      Serial.print(
        " | M2: "
      );

      Serial.print(
        digitalRead(PIN_VIB_2)
        ?
        "HIGH"
        :
        "LOW"
      );


      Serial.print(
        " | PID="
      );

      Serial.print(
        pidOutputPct,
        1
      );

      Serial.print(
        "% | "
      );


      if (faultLatched) {

        Serial.println(
          "FAULT"
        );
      }


      else if (vib1300Reached) {

        Serial.println(
          "COOLING AFTER 1300"
        );
      }


      else if (
        vib1200Reached
        &&
        interactionActive
      ) {

        Serial.println(
          "HOLD TEMP AFTER 1200"
        );
      }


      else if (pauseHoldActive) {

        Serial.println(
          "PAUSED - HOLDING TEMP"
        );
      }


      else if (interactionActive) {

        Serial.println(
          "HEATING"
        );
      }


      else {

        Serial.println(
          "IDLE"
        );
      }

    } else {

      sensorValid = false;

      heaterOff();


      if (interactionActive) {

        triggerFault(
          "MAX6675 read failed"
        );
      }


      Serial.println(
        "TEMP ERROR"
      );
    }
  }


  // ==========================================================
  // HARD CUT-OFF
  // ==========================================================

  if (

    sensorValid

    &&

    tempC
    >=
    HARD_STOP_C

    &&

    !faultLatched

  ) {

    triggerFault(
      "Hard temperature cutoff reached"
    );
  }


  // ==========================================================
  // TEC PID
  // ==========================================================

  updateTECControl();


  // ==========================================================
  // VIBRATION
  // ==========================================================

  updateVibration();
}