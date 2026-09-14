// servo_pattern.ino
// ESP32-S3 Supermini + one positional hobby servo (SG92R). No microphone.
//
// A one-voice percussion sequencer. This exists because the measurement rig
// answered its question: the servo's noise sits at a fixed ~9 kHz whatever
// speed or distance it is commanded, it is a noise band rather than a tone,
// and what actually varies with the command is loudness. Step size ran
// 44/79/148/246 RMS for 1/2/3/5 degree steps. So there is no note to play and
// no calibration table to build. There is a hit, and it has a velocity.
//
// A hit is a MOVE, since the sound only exists while the servo is travelling.
// The servo ping-pongs either side of a centre, each hit jumping to the other
// side, which keeps it inside its travel and lets a pattern loop forever.
//
// Protocol over serial at 115200, one command per line:
//   T <bpm>      tempo, 20-400
//   P <pattern>  x or X = hit at default velocity, 1-9 = hit at that velocity
//                in degrees, . or - = rest. Up to 64 slots, one slot per
//                sixteenth note.
//   V <deg>      default velocity in degrees, 1-20
//   S            start        X  stop
//   ?            status
//
// Wiring: servo signal -> GPIO8, powered from the board's 5V pin, common ground.

#include <ESP32Servo.h>

#define VERSION "1.0"
#define SERVO_PIN 8

// The servo only accepts a new position once per PWM frame. Asking for hits
// closer together than this does not play them faster, it silently drops them.
const uint32_t PWM_FRAME_MS = 20;

const int CENTRE_ANGLE = 90;
const int SAFE_MIN     = 30;   // stay well clear of the end stops
const int SAFE_MAX     = 150;
const int MAX_VELOCITY = 20;
const int MAX_SLOTS    = 64;

Servo servo;

char pattern[MAX_SLOTS + 1] = "x...x...x...x...";
int  patternLen = 16;
int  bpm = 120;
int  defaultVelocity = 4;
bool running = false;

int  slot = 0;
int  dir = 1;
uint32_t nextSlotAt = 0;

uint32_t slotIntervalMs() {
  // Four slots to the beat: a sixteenth at the given tempo.
  return (uint32_t)(60000.0 / (double)bpm / 4.0);
}

void announceTiming() {
  uint32_t iv = slotIntervalMs();
  Serial.printf("tempo %d bpm, %lu ms per slot\n", bpm, (unsigned long)iv);
  if (iv < PWM_FRAME_MS) {
    Serial.printf("WARNING: %lu ms is shorter than the servo's %lu ms PWM frame. "
                  "Hits will be dropped rather than played faster. "
                  "Max usable tempo at sixteenths is %d bpm.\n",
                  (unsigned long)iv, (unsigned long)PWM_FRAME_MS,
                  (int)(60000.0 / 4.0 / (double)PWM_FRAME_MS));
  }
}

void hit(int velocity) {
  if (velocity < 1) velocity = 1;
  if (velocity > MAX_VELOCITY) velocity = MAX_VELOCITY;
  // Jump to the other side of centre. Travel per hit is the velocity, so the
  // servo stays put on average however long the pattern runs.
  int target = CENTRE_ANGLE + dir * (velocity / 2 + velocity % 2);
  if (target < SAFE_MIN) target = SAFE_MIN;
  if (target > SAFE_MAX) target = SAFE_MAX;
  servo.write(target);
  dir = -dir;
}

void printStatus() {
  Serial.printf("pattern [%s] len %d, tempo %d bpm, velocity %d, %s\n",
                pattern, patternLen, bpm, defaultVelocity,
                running ? "running" : "stopped");
}

void handleLine(char *line) {
  while (*line == ' ') line++;
  char c = *line;
  char *arg = line + 1;
  while (*arg == ' ') arg++;

  switch (c) {
    case 'T': case 't': {
      // The ceiling is deliberately past the point the servo can keep up:
      // sixteenths above ~750 bpm ask for hits closer than the 20 ms PWM
      // frame, and hearing that limit is more use than being fenced off it.
      int v = atoi(arg);
      if (v < 20 || v > 900) { Serial.println("err: tempo must be 20-900"); return; }
      bpm = v;
      announceTiming();
      Serial.println("ok");
      return;
    }
    case 'V': case 'v': {
      int v = atoi(arg);
      if (v < 1 || v > MAX_VELOCITY) {
        Serial.printf("err: velocity must be 1-%d\n", MAX_VELOCITY); return;
      }
      defaultVelocity = v;
      Serial.println("ok");
      return;
    }
    case 'P': case 'p': {
      int n = 0;
      for (char *p = arg; *p && n < MAX_SLOTS; p++) {
        if (*p == ' ') continue;
        if (*p=='x'||*p=='X'||*p=='.'||*p=='-'||(*p>='1'&&*p<='9')) pattern[n++] = *p;
        else { Serial.printf("err: bad character '%c', use x . - or 1-9\n", *p); return; }
      }
      if (n == 0) { Serial.println("err: empty pattern"); return; }
      pattern[n] = '\0';
      patternLen = n;
      slot = 0;
      Serial.println("ok");
      printStatus();
      return;
    }
    case 'S': case 's':
      running = true; slot = 0; nextSlotAt = millis();
      announceTiming();
      Serial.println("ok running");
      return;
    case 'X':
      running = false;
      servo.write(CENTRE_ANGLE);
      Serial.println("ok stopped");
      return;
    case '?':
      printStatus();
      return;
    case '\0':
      return;
    default:
      Serial.printf("err: unknown command '%c', try T P V S X or ?\n", c);
      return;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("=== servo pattern sequencer v" VERSION " ===");
  Serial.println("One-voice servo percussion, pattern over serial: T bpm, P pattern, V deg, S, X, ?");
  Serial.println("https://github.com/sui001/Servo-notes");

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 1000, 2000);
  servo.write(CENTRE_ANGLE);

  printStatus();
  Serial.println("ok ready");
}

void loop() {
  static char buf[128];
  static int len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) { buf[len] = '\0'; handleLine(buf); len = 0; }
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }

  if (!running) return;

  uint32_t now = millis();
  if ((int32_t)(now - nextSlotAt) < 0) return;

  char c = pattern[slot];
  if (c == 'x' || c == 'X')      hit(defaultVelocity);
  else if (c >= '1' && c <= '9') hit(c - '0');

  slot = (slot + 1) % patternLen;
  nextSlotAt += slotIntervalMs();
  // If we have fallen behind (tempo faster than the servo can be driven),
  // do not try to catch up by firing a burst of hits.
  if ((int32_t)(now - nextSlotAt) > 0) nextSlotAt = now + slotIntervalMs();
}
