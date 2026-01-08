/*
   ACTIVA SMART SYSTEM - BUG FIXED VERSION
   -------------------------------------------------
   Updates:
   1. Added BT.setTimeout(50) to fix Bluetooth lag/freezing.
   2. Synced variable states perfectly with HTML.
   3. Kept all original logic (Latched, Secret Tap, Anti-Hijack).
*/

#include <SoftwareSerial.h>
#include <avr/wdt.h> // Watchdog Timer for Stability

// ================= CONFIGURATION =================

// RELAY SETTINGS (Set 'true' if using Standard 5V Relays that trigger on LOW)
const bool RELAY_ACTIVE_LOW = true; 

// TIMINGS
const int WARMUP_DELAY       = 2000;  // 2s Wait between Ignition and Run
const int LONG_PRESS_MS      = 800;   // Time to trigger "Direct Start"
const int SECRET_TIMEOUT     = 2000;  // Max time between taps

// SAFETY & BATTERY
const unsigned long AUTO_SLEEP_TIME    = 600000; // 10 Minutes -> Deep Sleep
const unsigned long HIJACK_WARN_START  = 15000;  // 15 Secs Disconnect -> Warning
const unsigned long HIJACK_KILL_TIME   = 120000; // 2 Mins Disconnect -> Kill

// PIN DEFINITIONS
const int PIN_BUTTON    = 2;  // Main Physical Button
const int PIN_IGNITION  = 3;  // Relay 1: Key Power
const int PIN_STARTER   = 4;  // Relay 2: Run Wire (LATCHED)
const int PIN_LED       = 5;  // Ring Light
const int PIN_BT_STATE  = 6;  // HC-05 State Pin
const int PIN_HORN      = 7;  // Horn
const int PIN_INDICATOR = 8;  // Indicators
const int PIN_RX        = 10; 
const int PIN_TX        = 11; 
const int PIN_BT_PWR    = 12; // Transistor for Bluetooth Power

SoftwareSerial BT(PIN_RX, PIN_TX);

// ================= VARIABLES =================

int RELAY_ON, RELAY_OFF;

// States (Synced with HTML)
bool isIgnitionOn    = false; 
bool isEngineRunning = false; 
bool isLocked        = true;   
bool isConnected     = false;
bool btPowerActive   = true;

// Secret Code
int secretStage = 0; 
unsigned long lastSecretInput = 0;

// Timers
unsigned long disconnectStartTime = 0;
unsigned long lastHeartbeatTime   = 0;
unsigned long buttonPressTime     = 0;

void setup() {
  wdt_enable(WDTO_2S); // Auto-restart if system hangs

  Serial.begin(9600);
  BT.begin(9600);
  
  // --- BUG FIX IS HERE ---
  // If the App forgets to send \n, Arduino waits 1 second by default.
  // We change this to 50ms so it reacts INSTANTLY even without \n.
  BT.setTimeout(50); 
  
  // Configure Relays
  if (RELAY_ACTIVE_LOW) {
    RELAY_ON = LOW; RELAY_OFF = HIGH;
  } else {
    RELAY_ON = HIGH; RELAY_OFF = LOW;
  }

  // Set Outputs
  pinMode(PIN_IGNITION, OUTPUT); digitalWrite(PIN_IGNITION, RELAY_OFF);
  pinMode(PIN_STARTER,  OUTPUT); digitalWrite(PIN_STARTER,  RELAY_OFF);
  pinMode(PIN_HORN,     OUTPUT); digitalWrite(PIN_HORN,     RELAY_OFF);
  pinMode(PIN_INDICATOR, OUTPUT); digitalWrite(PIN_INDICATOR, RELAY_OFF);
  pinMode(PIN_LED,      OUTPUT); digitalWrite(PIN_LED,      LOW);
  pinMode(PIN_BT_PWR,   OUTPUT); digitalWrite(PIN_BT_PWR,   HIGH); // BT ON

  // Set Inputs
  pinMode(PIN_BUTTON, INPUT_PULLUP); 
  pinMode(PIN_BT_STATE, INPUT);

  // Boot Confirmation
  digitalWrite(PIN_LED, HIGH); delay(200); digitalWrite(PIN_LED, LOW);
  Serial.println("SYSTEM READY - BUG FIXED VERSION");
}

void loop() {
  wdt_reset(); 
  unsigned long now = millis();

  // 1. Check Connectivity & Sleep
  checkConnection(now);
  
  // 2. Handle Inputs
  handleButton(now);
  handleBluetoothCommands();

  // 3. Status Updates
  sendHeartbeat(now, false);
  updateBreathingLED(now);
}

// =============================================================
//                    CONTROL LOGIC
// =============================================================

void toggleIgnition(bool state) {
  if (state) {
    digitalWrite(PIN_IGNITION, RELAY_ON);
    isIgnitionOn = true;
    blinkIndicators(1); 
    Serial.println("IGNITION ON");
  } else {
    // Turning off Ignition kills Engine too
    digitalWrite(PIN_STARTER, RELAY_OFF);
    digitalWrite(PIN_IGNITION, RELAY_OFF);
    isEngineRunning = false;
    isIgnitionOn = false;
    Serial.println("ALL OFF");
  }
  sendHeartbeat(millis(), true); // Force update to App
}

void toggleEngineRun(bool state) {
  if (state) {
    // Safety: Ignition must be ON first
    if (!isIgnitionOn) digitalWrite(PIN_IGNITION, RELAY_ON); 
    isIgnitionOn = true;

    digitalWrite(PIN_STARTER, RELAY_ON); // Latched
    isEngineRunning = true;
    digitalWrite(PIN_LED, HIGH); // Solid Light
    Serial.println("ENGINE RUN: ON");
  } else {
    digitalWrite(PIN_STARTER, RELAY_OFF);
    isEngineRunning = false;
    Serial.println("ENGINE RUN: OFF");
  }
  sendHeartbeat(millis(), true); // Force update to App
}

void fullShutdown() {
  toggleIgnition(false); // Kills everything
  playAnimation("GOODBYE");
}

void autoStartSequence() {
  if (!isIgnitionOn) toggleIgnition(true);
  
  // Visual Wait (Fuel Pump Prime)
  for(int i=0; i<20; i++) {
    wdt_reset();
    analogWrite(PIN_LED, i*10); 
    delay(50); 
  }

  toggleEngineRun(true);
}

// --- BUTTON & SECRET CODE ---

void handleButton(unsigned long now) {
  static int lastBtn = HIGH;
  int btn = digitalRead(PIN_BUTTON);

  // Wake Up Bluetooth if Sleeping
  if (btn == LOW && !btPowerActive) {
    wakeUpBluetooth();
    delay(500); 
    return;
  }

  // Press Start
  if (lastBtn == HIGH && btn == LOW) {
    buttonPressTime = now;
  }

  // Press Release
  if (lastBtn == LOW && btn == HIGH) {
    unsigned long duration = now - buttonPressTime;
    if (duration > 50) { // Debounce
      if (duration < LONG_PRESS_MS) handleTap(now);
      else handleLongPress(now);
    }
  }
  lastBtn = btn;

  // Timeout Secret Code
  if (secretStage > 0 && (now - lastSecretInput > SECRET_TIMEOUT)) {
    secretStage = 0; 
    if(!isLocked) quickFlashLED(1); 
  }
}

void handleTap(unsigned long now) {
  // LOCKED: Process Secret Code
  if (isLocked) {
    lastSecretInput = now;
    if (secretStage == 0) secretStage = 1;      // Tap 1
    else if (secretStage == 1) secretStage = 2; // Tap 2
    else secretStage = 0; // Fail
    
    digitalWrite(PIN_LED, HIGH); delay(50); digitalWrite(PIN_LED, LOW);
    return;
  }

  // UNLOCKED: Normal Controls
  if (!isIgnitionOn) {
    toggleIgnition(true);
  } else if (isIgnitionOn && !isEngineRunning) {
    toggleEngineRun(true);
  } else {
    fullShutdown();
  }
}

void handleLongPress(unsigned long now) {
  // LOCKED: Unlock Sequence
  if (isLocked) {
    if (secretStage == 2) { 
      isLocked = false;
      secretStage = 0;
      playAnimation("WELCOME");
      toggleIgnition(true); 
    } else {
      secretStage = 0;
      triggerAlarmShort(); 
    }
    return;
  }

  // UNLOCKED: Direct Actions
  if (!isEngineRunning) {
    autoStartSequence(); 
  } else {
    fullShutdown(); 
  }
}

// =============================================================
//              CONNECTION & SAFETY SYSTEM
// =============================================================

void checkConnection(unsigned long now) {
  bool pinState = digitalRead(PIN_BT_STATE);
  
  // Also consider us "Connected" if we recently received valid data
  bool dataActive = (now - lastHeartbeatTime < 2000) && isConnected; 

  if (pinState == HIGH) { // HC-05 State Pin is reliable
    if (!isConnected) {
      isConnected = true;
      disconnectStartTime = 0;
      playAnimation("WELCOME");
    }
    disconnectStartTime = 0; 
  } 
  else {
    // --- DISCONNECTED ---
    if (isConnected) {
      isConnected = false;
      disconnectStartTime = now;
      playAnimation("GOODBYE");
    }

    if (disconnectStartTime > 0) {
      unsigned long offlineTime = now - disconnectStartTime;

      // 1. SLEEP (If Engine OFF + 10 mins)
      if (!isIgnitionOn && offlineTime > AUTO_SLEEP_TIME && btPowerActive) {
        digitalWrite(PIN_BT_PWR, LOW);
        btPowerActive = false;
        digitalWrite(PIN_LED, LOW);
      }

      // 2. ANTI-HIJACK (If Engine ON)
      if (isEngineRunning) {
        if (offlineTime > HIJACK_KILL_TIME) {
          triggerTheftMode(); 
        } 
        else if (offlineTime > HIJACK_WARN_START) {
          // Warning Beeps
          if ((now / 500) % 4 == 0) { 
             digitalWrite(PIN_HORN, RELAY_ON); 
             digitalWrite(PIN_INDICATOR, RELAY_ON);
          } else {
             digitalWrite(PIN_HORN, RELAY_OFF);
             digitalWrite(PIN_INDICATOR, RELAY_OFF);
          }
        }
      }
    }
  }
}

void handleBluetoothCommands() {
  if (BT.available()) {
    // Read command with timeout safety (Fixes newline bug)
    String cmd = BT.readStringUntil('\n');
    cmd.trim(); 
    cmd.toUpperCase();

    if (cmd.length() > 0) {
      if (cmd == "START")      autoStartSequence();
      else if (cmd == "STOP")  fullShutdown();
      else if (cmd == "FIND")  playAnimation("FIND");
      else if (cmd == "LOCK")  { fullShutdown(); isLocked = true; quickFlashLED(3); }
      else if (cmd == "UNLOCK") { isLocked = false; playAnimation("WELCOME"); }
    }
  }
}

void sendHeartbeat(unsigned long now, bool force) {
  // Sends status to App every 2 seconds or on change
  if (force || (now - lastHeartbeatTime > 2000)) {
    if (isEngineRunning) BT.print("ENGINE_ON\n");
    else if (isIgnitionOn) BT.print("IGNITION_ON\n");
    else BT.print("ENGINE_OFF\n");
    lastHeartbeatTime = now;
  }
}

// =============================================================
//                    ANIMATIONS & UTILS
// =============================================================

void wakeUpBluetooth() {
  digitalWrite(PIN_BT_PWR, HIGH);
  btPowerActive = true;
  disconnectStartTime = millis(); 
  quickFlashLED(2);
}

void triggerTheftMode() {
  fullShutdown();
  isLocked = true;
  for(int i=0; i<10; i++) {
    digitalWrite(PIN_HORN, RELAY_ON);
    digitalWrite(PIN_INDICATOR, RELAY_ON);
    delay(200);
    digitalWrite(PIN_HORN, RELAY_OFF);
    digitalWrite(PIN_INDICATOR, RELAY_OFF);
    delay(200);
    wdt_reset();
  }
}

void playAnimation(String type) {
  if (type == "WELCOME") {
    blinkIndicators(2); 
  } 
  else if (type == "GOODBYE") {
    digitalWrite(PIN_INDICATOR, RELAY_ON);
    delay(800);
    digitalWrite(PIN_INDICATOR, RELAY_OFF);
  }
  else if (type == "FIND") {
    for(int i=0; i<3; i++) {
      digitalWrite(PIN_INDICATOR, RELAY_ON);
      digitalWrite(PIN_HORN, RELAY_ON);
      delay(300);
      digitalWrite(PIN_INDICATOR, RELAY_OFF);
      digitalWrite(PIN_HORN, RELAY_OFF);
      delay(300);
      wdt_reset();
    }
  }
}

void updateBreathingLED(unsigned long now) {
  if (isIgnitionOn || isEngineRunning) return; 
  if (!isConnected && !btPowerActive) return; 

  float val = (exp(sin(now/2000.0*PI)) - 0.36787944)*108.0;
  analogWrite(PIN_LED, val);
}

void blinkIndicators(int count) {
  for(int i=0; i<count; i++) {
    digitalWrite(PIN_INDICATOR, RELAY_ON); delay(200);
    digitalWrite(PIN_INDICATOR, RELAY_OFF); delay(200);
  }
}

void quickFlashLED(int count) {
  for(int i=0; i<count; i++) {
    digitalWrite(PIN_LED, HIGH); delay(100);
    digitalWrite(PIN_LED, LOW); delay(100);
  }
}

void triggerAlarmShort() {
    digitalWrite(PIN_HORN, RELAY_ON); delay(150);
    digitalWrite(PIN_HORN, RELAY_OFF);
}