// servo_mic_test.ino
// ESP32-S3 Supermini + INMP441 (I2S mic) + 9g (or bigger) hobby servo.
//
// Runs a scheduled sequence of servo moves while continuously streaming
// mic audio out over USB serial, so movement noise is captured with no
// gaps. Prints a JSON test plan first (so the PC side knows what happened
// when), then a delimiter line, then raw 16-bit PCM forever.
//
// UNTESTED — written without hardware to compile/flash against. Likely
// things to check on first flash:
//   - Arduino-ESP32 core version: this uses the legacy `driver/i2s.h` API.
//     Core v3.x may want `driver/i2s_std.h` instead — if i2s_driver_install
//     fails to compile, that's why.
//   - ESP32Servo needs a core with LEDC available on your chosen pin.
//   - Baud 921600 over native USB CDC should be plenty for 16kHz*16bit
//     (32 KB/s), but confirm your serial monitor / pyserial can hold it.
//
// Wiring (avoid ESP32-S3 strapping pins 0/3/45/46):
//   INMP441   VDD->3V3  GND->GND  L/R->GND (left ch)  WS->GPIO12  SCK->GPIO11  SD->GPIO13
//   Servo     signal->GPIO8  power from the board's 5V pin (fine for a 9g SG92R), and
//             share ground with the ESP32.

#include <ESP32Servo.h>
#include <driver/i2s.h>

#define I2S_WS    12
#define I2S_SD    13
#define I2S_SCK   11
#define SERVO_PIN 8

#define SAMPLE_RATE 16000
#define I2S_PORT    I2S_NUM_0

// ---- SET THIS before flashing: which servo type is wired up ----
static const bool TEST_POSITIONAL = true;   // 0-180 deg hobby servo
static const bool TEST_CONTINUOUS = false;  // continuous-rotation servo

Servo servo;

// ---------------- Test parameters ----------------
const int distances[]  = {5, 10, 20, 30, 45, 60, 90, 120, 150}; // degrees from home
const int stepDelays[] = {0, 5, 10, 20};                        // ms/degree; 0 = jump (servo's own max speed)
const int contSpeeds[] = {50, 100, 150, 200, 300, 400, 500};    // us offset from 1500 (stop), both directions
const int REPEATS = 3;

const uint32_t MOVE_WINDOW_MS = 700;   // analysis window per event
const uint32_t GAP_MS         = 900;   // silence between events
const int HOME_ANGLE          = 90;

// Logical events, used only for the JSON plan the PC needs for slicing.
struct LogEvent { char type; int p1; int p2; uint32_t start_ms; uint32_t dur_ms; };
LogEvent logEvents[300];
int logLen = 0;

// Raw hardware commands, purely time-triggered, no blocking.
// Sized for the worst case: 3 repeats x 9 distances x (3 stepped rates x
// (distance+2) steps + 1 jump-rate x 2 steps) = ~4986 for the positional
// sweep as configured below. Bumped with headroom since this silently
// overflowed into other globals at 2000 and looked like a hardware fault.
struct Cmd { uint32_t time_ms; bool isServoWrite; int angleOrUs; bool isMicroseconds; };
Cmd cmds[6000];
int cmdLen = 0;
uint32_t totalDuration = 0;

void haltOnOverflow(const char *what) {
  Serial.printf("FATAL: %s array full — reduce the sweep or bump its size, don't just let it wrap.\n", what);
  Serial.flush();
  while (true) delay(1000);
}

void addCmdAngle(uint32_t t, int angle) {
  if (cmdLen >= (int)(sizeof(cmds)/sizeof(cmds[0]))) haltOnOverflow("cmds");
  cmds[cmdLen++] = {t, true, angle, false};
}
void addCmdMicros(uint32_t t, int us) {
  if (cmdLen >= (int)(sizeof(cmds)/sizeof(cmds[0]))) haltOnOverflow("cmds");
  cmds[cmdLen++] = {t, true, us, true};
}

void buildPlan() {
  uint32_t t = 500; // lead-in silence

  for (int r = 0; r < REPEATS; r++) {
    if (TEST_POSITIONAL) {
      for (int d = 0; d < (int)(sizeof(distances)/sizeof(int)); d++) {
        for (int s = 0; s < (int)(sizeof(stepDelays)/sizeof(int)); s++) {
          int target = HOME_ANGLE + distances[d];
          int stepDelay = stepDelays[s];

          if (logLen >= (int)(sizeof(logEvents)/sizeof(logEvents[0]))) haltOnOverflow("logEvents");
          logEvents[logLen++] = {'P', distances[d], stepDelay, t, MOVE_WINDOW_MS};

          if (stepDelay == 0) {
            addCmdAngle(t, target);
          } else {
            for (int a = HOME_ANGLE, i = 0; a <= target; a++, i++) {
              addCmdAngle(t + i * (uint32_t)stepDelay, a);
            }
          }
          uint32_t evEnd = t + MOVE_WINDOW_MS + GAP_MS;
          addCmdAngle(evEnd - 200, HOME_ANGLE); // return home before next event
          t = evEnd;
        }
      }
    }
    if (TEST_CONTINUOUS) {
      for (int s = 0; s < (int)(sizeof(contSpeeds)/sizeof(int)); s++) {
        for (int sign = -1; sign <= 1; sign += 2) {
          int us = 1500 + sign * contSpeeds[s];
          if (logLen >= (int)(sizeof(logEvents)/sizeof(logEvents[0]))) haltOnOverflow("logEvents");
          logEvents[logLen++] = {'C', sign * contSpeeds[s], 0, t, MOVE_WINDOW_MS + 300};
          addCmdMicros(t, us);
          uint32_t evEnd = t + MOVE_WINDOW_MS + 300 + GAP_MS;
          addCmdMicros(evEnd - GAP_MS, 1500); // stop
          t = evEnd;
        }
      }
    }
  }
  totalDuration = t + 500; // trailing silence
}

void printPlanJson() {
  Serial.print("{\"sample_rate\":16000,\"events\":[");
  for (int i = 0; i < logLen; i++) {
    Serial.printf("{\"i\":%d,\"type\":\"%c\",\"p1\":%d,\"p2\":%d,\"start_ms\":%lu,\"dur_ms\":%lu}%s",
      i, logEvents[i].type, logEvents[i].p1, logEvents[i].p2,
      (unsigned long)logEvents[i].start_ms, (unsigned long)logEvents[i].dur_ms,
      (i < logLen - 1) ? "," : "");
  }
  Serial.printf("],\"total_ms\":%lu}\n", (unsigned long)totalDuration);
  Serial.println("===AUDIO_START===");
}

void i2sInit() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
}

void setup() {
  Serial.begin(921600);
  delay(1500);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 1000, 2000);
  servo.write(HOME_ANGLE);

  i2sInit();
  buildPlan();
  printPlanJson();
}

int cmdIdx = 0;
uint32_t t0;
bool started = false;
bool done = false;

void loop() {
  if (!started) { t0 = millis(); started = true; }
  uint32_t now = millis() - t0;

  // Fire any due hardware commands. Each is a single non-blocking call.
  while (cmdIdx < cmdLen && now >= cmds[cmdIdx].time_ms) {
    Cmd &c = cmds[cmdIdx];
    if (c.isMicroseconds) servo.writeMicroseconds(c.angleOrUs);
    else                  servo.write(c.angleOrUs);
    cmdIdx++;
  }

  // Stream mic audio continuously regardless of what the servo is doing.
  static int32_t raw[128];
  size_t bytesRead = 0;
  i2s_read(I2S_PORT, raw, sizeof(raw), &bytesRead, 0); // non-blocking (0 tick wait)
  int n = bytesRead / sizeof(int32_t);
  if (n > 0) {
    static int16_t out[128];
    for (int i = 0; i < n; i++) out[i] = (int16_t)(raw[i] >> 14); // INMP441 24-bit-in-32 -> 16-bit
    Serial.write((uint8_t*)out, n * sizeof(int16_t));
  }

  if (!done && now > totalDuration + 1000) {
    done = true;
    servo.writeMicroseconds(1500);
  }
}
