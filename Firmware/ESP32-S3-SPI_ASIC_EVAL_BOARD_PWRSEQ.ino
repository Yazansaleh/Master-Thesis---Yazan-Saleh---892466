#include <SPI.h>

const int VSPI_MOSI = 35; // DIN_A
const int VSPI_MISO = 37;
const int VSPI_SCLK = 36; // CLKIN_A
const int VSPI_SS   = 10; // STDIN_A (CS / Store_data)

// Power rails (your pins)
static const int railPins[6] = {
  16,  // rail 1 
  14,  // rail 2
  15,  // rail 3
  6,   // rail 4
  5,   // rail 5
  9    // rail 6
};

const uint32_t spiClk = 1000000;  // 1 MHz

// --------- Robustness knobs ----------
static const bool     DEBUG_PRINT = false;   // keep false during automation
static const bool     SEND_ACK    = true;    // send 'K' after each valid frame
static const uint32_t FRAME_TIMEOUT_MS = 30; // timeout to resync
// -------------------------------------

static uint8_t buffer[45];
static uint8_t idx = 0;
static bool initMode = false;
static uint32_t lastByteMs = 0;

// Power sequencing flags (run once after first valid SPI update)
static bool powerSequenceDone = false;
static bool powerSequenceRunning = false;

static inline void flushInput() {
  while (Serial.available()) Serial.read();
}

void runPowerSequenceOnce() {
  if (powerSequenceDone || powerSequenceRunning) return;

  powerSequenceRunning = true;

  // Do the requested cycle twice:
  // - all ON
  // - OFF one-by-one with 1s delay
  for (int cycle = 0; cycle < 1; cycle++) {
    // All ON
    for (int i = 0; i < 6; i++) {
      digitalWrite(railPins[i], HIGH);
    }

    delay(1000); // optional settle with all ON

    // OFF one by one
    for (int i = 0; i < 6; i++) {
      digitalWrite(railPins[i], LOW);
      delay(500);
    }
  }
      digitalWrite(railPins[2], HIGH);
      delay(3000);
      digitalWrite(railPins[2], LOW);
      delay(3000);
      digitalWrite(railPins[2], HIGH);
      delay(3000);
      digitalWrite(railPins[2], LOW);
      delay(3000);
    

  powerSequenceDone = true;
  powerSequenceRunning = false;

  if (DEBUG_PRINT) Serial.println("[DBG] Power sequence done.");
}

void send44Bytes(uint8_t *data) {
  // Keep debug minimal; large prints can break serial timing
  if (DEBUG_PRINT) {
    Serial.print("[DBG] TX 44 bytes, first 4 = ");
    for (int i = 0; i < 4; i++) {
      Serial.print("0x");
      if (data[i] < 16) Serial.print("0");
      Serial.print(data[i], HEX);
      Serial.print(i == 3 ? "\n" : " ");
    }
  }

  SPI.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));

  digitalWrite(VSPI_SS, LOW);
  delayMicroseconds(2);

  for (int i = 0; i < 44; i++) {
    SPI.transfer(data[i]);
  }

  delayMicroseconds(2);
  digitalWrite(VSPI_SS, HIGH);

  SPI.endTransaction();

  if (DEBUG_PRINT) Serial.println("[DBG] SPI 44-byte transaction complete.");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(FRAME_TIMEOUT_MS);

  // SPI init
  SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
  pinMode(VSPI_SS, OUTPUT);
  digitalWrite(VSPI_SS, HIGH);

  // Power rail GPIOs
  for (int i = 0; i < 6; i++) {
    pinMode(railPins[i], OUTPUT);
    digitalWrite(railPins[i], LOW); // start OFF
  }

  Serial.println("Ready for 44-byte config blocks.");
}

void loop() {
  // If we started receiving a frame and it stalls -> resync
  if (idx > 0 && (millis() - lastByteMs) > FRAME_TIMEOUT_MS) {
    idx = 0;
    initMode = false;
    flushInput();
    if (DEBUG_PRINT) Serial.println("[WARN] Frame timeout -> resync");
  }

  while (Serial.available()) {
    uint8_t b = Serial.read();
    lastByteMs = millis();

    // Enter init mode on 'I' (expects 44 raw bytes)
    if (b == 'I') {
      idx = 0;
      initMode = true;
      continue;
    }

    // ---------- INIT MODE: 44 bytes then send 4x11 with CS toggling ----------
    if (initMode) {
      buffer[idx++] = b;

      if (idx == 44) {
        SPI.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));

        // Send 11 bytes × 4 with CS toggling
        for (int i = 0; i < 4; i++) {
          digitalWrite(VSPI_SS, LOW);
          delayMicroseconds(2);

          for (int j = 0; j < 11; j++) {
            SPI.transfer(buffer[i * 11 + j]);
          }

          delayMicroseconds(2);
          digitalWrite(VSPI_SS, HIGH);
          delayMicroseconds(5); // latch gap
        }

        SPI.endTransaction();

        initMode = false;
        idx = 0;

        if (SEND_ACK) Serial.write('K');

        // Run power sequencing after the first valid SPI activity
        runPowerSequenceOnce();
      }

      continue;
    }

    // ---------- NORMAL MODE: 'B' + 44 bytes ----------
    if (idx == 0) {
      if (b != 'B') {
        // ignore until we see a frame start
        continue;
      }
      buffer[idx++] = b;
      continue;
    }

    buffer[idx++] = b;

    if (idx == 45) {
      send44Bytes(buffer + 1);
      idx = 0;

      if (SEND_ACK) Serial.write('K');

      // Run power sequencing after the first valid SPI activity
      runPowerSequenceOnce();
    }
  }

  yield();
}
