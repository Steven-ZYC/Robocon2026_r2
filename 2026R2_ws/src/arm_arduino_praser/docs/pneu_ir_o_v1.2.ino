/*
  Arduino Mega 2560
  Pneumatics + IR Sensor Bidirectional Serial Controller
  V1.2: With STATE frame timestamp and LRC/XOR checksum

  Host -> Arduino:
    [0,0,0]\n
    [1,0,0]\n
    [0,1,0]\n
    [0,0,1]\n
    [1,0,1]\n
    STATUS\n
    OFF\n
    STOP\n
    ALL_OFF\n

  Arduino -> Host:
    <STATE,t:123456,pneu:[1,0,1],ir:1,*5A>\n

  Timestamp:
    t is Arduino millis(), in milliseconds since board boot.

  LRC/XOR calculation:
    XOR all characters between '<' and ',*'
    Example payload:
      STATE,t:123456,pneu:[1,0,1],ir:1

  Pin / relay / valve mapping:
    Pneu 1 / Relay 1 / Gripper -> D5, active HIGH
    Pneu 2 / Relay 2 / Lift    -> D6, active LOW
    Pneu 3 / Relay 3 / Stopper -> D8, active HIGH

    IR sensor OUT -> D2
    LED indicator -> D13

  Baud rate:
    115200
*/

// 0 = use Serial1 on D18/D19 for host communication
// 1 = use USB Serial for host communication
#define USE_USB_SERIAL 1

#if USE_USB_SERIAL
  #define HOST_SERIAL Serial
#else
  #define HOST_SERIAL Serial1
#endif

const byte NUM_PNEU = 3;

// Keep original pneumatic / relay pins unchanged.
const byte pneuPins[NUM_PNEU] = {
  5,  // Pneu 1 / Relay 1 / Gripper
  6,  // Pneu 2 / Relay 2 / Lift
  8   // Pneu 3 / Relay 3 / Stopper
};

// true  = active LOW:  LOW = ON,  HIGH = OFF
// false = active HIGH: HIGH = ON, LOW  = OFF
const bool pneuActiveLow[NUM_PNEU] = {
  false,  // D5 active HIGH
  true,   // D6 active LOW
  false   // D8 active HIGH
};

bool pneuStates[NUM_PNEU] = {
  false,
  false,
  false
};

// IR sensor
const byte SENSOR_PIN = 2;
const byte LED_PIN = 13;

// Safety timeout.
// If host stops sending valid command for this time, all pneumatics turn OFF.
const unsigned long COMMAND_TIMEOUT_MS = 200;

// STATE frame publish interval.
// 20 ms = 50 Hz.
// If serial output is too frequent, change to 50.
const unsigned long STATE_PUBLISH_INTERVAL_MS = 20;

unsigned long lastValidCommandTime = 0;
unsigned long lastStatePublishTime = 0;

bool timeoutAlreadyHandled = false;
bool lastIrState = false;

// Serial input buffer
const byte INPUT_BUFFER_SIZE = 32;
char inputBuffer[INPUT_BUFFER_SIZE];
byte inputIndex = 0;

// Debug options
const bool ACK_EACH_VALID_COMMAND = false;
const bool PRINT_INVALID_COMMAND = false;

void setup() {
  HOST_SERIAL.begin(115200);

#if !USE_USB_SERIAL
  // USB debug output only, not used by ROS 2 host.
  Serial.begin(115200);
  Serial.println("Mega started. Host link = Serial1.");
#endif

  // Initialize pneumatic output pins
  for (byte i = 0; i < NUM_PNEU; i++) {
    pinMode(pneuPins[i], OUTPUT);
    digitalWrite(pneuPins[i], pneuOffLevel(i));
  }

  // Initialize IR sensor
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  allPneuOff();

  lastValidCommandTime = millis();
  lastStatePublishTime = millis();
  lastIrState = objectDetected();

  printBootFrame();
}

void loop() {
  readSerialNonBlocking();
  checkCommandTimeout();
  publishStateFrameNonBlocking();
}

bool objectDetected() {
  // Many NPN/open-collector sensors output LOW when detected.
  // If your sensor logic is opposite, change LOW to HIGH.
  return digitalRead(SENSOR_PIN) == LOW;
}

void publishStateFrameNonBlocking() {
  unsigned long currentTime = millis();

  if (currentTime - lastStatePublishTime < STATE_PUBLISH_INTERVAL_MS) {
    return;
  }

  lastStatePublishTime = currentTime;

  lastIrState = objectDetected();
  digitalWrite(LED_PIN, lastIrState ? HIGH : LOW);

  printStateFrame();
}

void readSerialNonBlocking() {
  while (HOST_SERIAL.available() > 0) {
    char c = HOST_SERIAL.read();

    // Ignore carriage return
    if (c == '\r') {
      continue;
    }

    // End of one command line
    if (c == '\n') {
      inputBuffer[inputIndex] = '\0';

      if (inputIndex > 0) {
        handleLine(inputBuffer);
      }

      inputIndex = 0;
      continue;
    }

    // Normal character
    if (inputIndex < INPUT_BUFFER_SIZE - 1) {
      inputBuffer[inputIndex] = c;
      inputIndex++;
    } else {
      // Buffer overflow: reset and turn all pneumatics off for safety
      inputIndex = 0;
      allPneuOff();

      if (PRINT_INVALID_COMMAND) {
        printErrorFrame("overflow");
      }
    }
  }
}

void handleLine(const char* line) {
  if (equalsCommand(line, "OFF") ||
      equalsCommand(line, "STOP") ||
      equalsCommand(line, "ALL_OFF")) {
    allPneuOff();
    registerValidCommand();

    if (ACK_EACH_VALID_COMMAND) {
      printAckFrame();
    }

    return;
  }

  if (equalsCommand(line, "STATUS")) {
    lastIrState = objectDetected();
    printStateFrame();
    return;
  }

  bool newStates[NUM_PNEU];

  if (parseListCommand(line, newStates)) {
    applyPneuStates(newStates);
    registerValidCommand();

    if (ACK_EACH_VALID_COMMAND) {
      printAckFrame();
    }
  } else {
    // Invalid command: turn all pneumatics OFF for safety.
    allPneuOff();
    registerValidCommand();

    if (PRINT_INVALID_COMMAND) {
      printErrorFrame("invalid_cmd");
    }
  }
}

bool equalsCommand(const char* line, const char* target) {
  byte i = 0;

  while (line[i] != '\0' && target[i] != '\0') {
    if (line[i] != target[i]) {
      return false;
    }
    i++;
  }

  return line[i] == '\0' && target[i] == '\0';
}

bool parseListCommand(const char* line, bool newStates[]) {
  /*
    Accepted:
      [0,0,0]
      [1,0,0]
      [0,1,0]
      [0,0,1]
      [1,0,1]
      [1, 0, 1]

    Rejected:
      100
      [1,0]
      [1,0,0,1]
      [1 true 0]
      abc[1,0,0]
  */

  int pos = 0;

  skipSpaces(line, pos);

  if (line[pos] != '[') {
    return false;
  }
  pos++;

  for (byte i = 0; i < NUM_PNEU; i++) {
    skipSpaces(line, pos);

    if (line[pos] != '0' && line[pos] != '1') {
      return false;
    }

    newStates[i] = (line[pos] == '1');
    pos++;

    skipSpaces(line, pos);

    if (i < NUM_PNEU - 1) {
      if (line[pos] != ',') {
        return false;
      }
      pos++;
    } else {
      if (line[pos] != ']') {
        return false;
      }
      pos++;
    }
  }

  skipSpaces(line, pos);

  // Nothing should remain after the closing bracket
  return line[pos] == '\0';
}

void skipSpaces(const char* line, int& pos) {
  while (line[pos] == ' ' || line[pos] == '\t') {
    pos++;
  }
}

void applyPneuStates(bool newStates[]) {
  for (byte i = 0; i < NUM_PNEU; i++) {
    pneuStates[i] = newStates[i];

    if (pneuStates[i]) {
      digitalWrite(pneuPins[i], pneuOnLevel(i));
    } else {
      digitalWrite(pneuPins[i], pneuOffLevel(i));
    }
  }
}

void allPneuOff() {
  for (byte i = 0; i < NUM_PNEU; i++) {
    pneuStates[i] = false;
    digitalWrite(pneuPins[i], pneuOffLevel(i));
  }
}

void checkCommandTimeout() {
  if (COMMAND_TIMEOUT_MS == 0) {
    return;
  }

  unsigned long currentTime = millis();

  if (!timeoutAlreadyHandled &&
      currentTime - lastValidCommandTime > COMMAND_TIMEOUT_MS) {
    allPneuOff();
    timeoutAlreadyHandled = true;
    printErrorFrame("timeout");
  }
}

void registerValidCommand() {
  lastValidCommandTime = millis();
  timeoutAlreadyHandled = false;
}

byte pneuOnLevel(byte index) {
  return pneuActiveLow[index] ? LOW : HIGH;
}

byte pneuOffLevel(byte index) {
  return pneuActiveLow[index] ? HIGH : LOW;
}

// =============================
// Frame output functions
// =============================

void printStateFrame() {
  char payload[56];

  snprintf(
    payload,
    sizeof(payload),
    "STATE,t:%lu,pneu:[%d,%d,%d],ir:%d",
    millis(),
    pneuStates[0] ? 1 : 0,
    pneuStates[1] ? 1 : 0,
    pneuStates[2] ? 1 : 0,
    lastIrState ? 1 : 0
  );

  printFrameWithLrc(payload);
}

void printBootFrame() {
  char payload[] = "BOOT,ready";
  printFrameWithLrc(payload);
}

void printAckFrame() {
  char payload[32];

  snprintf(
    payload,
    sizeof(payload),
    "ACK,pneu:[%d,%d,%d]",
    pneuStates[0] ? 1 : 0,
    pneuStates[1] ? 1 : 0,
    pneuStates[2] ? 1 : 0
  );

  printFrameWithLrc(payload);
}

void printErrorFrame(const char* reason) {
  char payload[40];

  snprintf(
    payload,
    sizeof(payload),
    "ERR,%s",
    reason
  );

  printFrameWithLrc(payload);
}

void printFrameWithLrc(const char* payload) {
  byte lrc = calcXorLrc(payload);

  HOST_SERIAL.print("<");
  HOST_SERIAL.print(payload);
  HOST_SERIAL.print(",*");

  if (lrc < 0x10) {
    HOST_SERIAL.print("0");
  }

  HOST_SERIAL.print(lrc, HEX);
  HOST_SERIAL.println(">");
}

byte calcXorLrc(const char* payload) {
  byte lrc = 0;

  for (int i = 0; payload[i] != '\0'; i++) {
    lrc ^= (byte)payload[i];
  }

  return lrc;
}
